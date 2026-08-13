#include "odrive_can_node.hpp"
#include "odrive_enums.h"
#include "epoll_event_loop.hpp"
#include "byte_swap.hpp"
#include <sys/eventfd.h>
#include <chrono>
#include <cmath>

enum CmdId : uint32_t {
    kHeartbeat = 0x001,            // ControllerStatus  - publisher
    kGetError = 0x003,             // SystemStatus      - publisher
    kSetAxisState = 0x007,         // SetAxisState      - service
    kGetEncoderEstimates = 0x009,  // ControllerStatus  - publisher
    kSetControllerMode = 0x00b,    // ControlMessage    - subscriber
    kSetInputPos,                  // ControlMessage    - subscriber
    kSetInputVel,                  // ControlMessage    - subscriber
    kSetInputTorque,               // ControlMessage    - subscriber
    kSetTrajVelLimit = 0x011,      // ControlMessage    - subscriber
    kSetTrajAccelLimits = 0x012,   // ControlMessage    - subscriber
    kGetIq = 0x014,                // ControllerStatus  - publisher
    kGetTemp,                      // SystemStatus      - publisher
    kGetBusVoltageCurrent = 0x017, // SystemStatus      - publisher
    kClearErrors = 0x018,          // ClearErrors       - service
    kGetTorques = 0x01c,           // ControllerStatus  - publisher
};

enum ControlMode : uint64_t {
    kVoltageControl,
    kTorqueControl,
    kVelocityControl,
    kPositionControl,
};

constexpr uint32_t kTrapTrajInputMode = 5;

ODriveCanNode::ODriveCanNode(const std::string& node_name) : rclcpp::Node(node_name) {
    
    rclcpp::Node::declare_parameter<std::string>("interface", "can0");
    rclcpp::Node::declare_parameter<uint16_t>("node_id", 0);
    rclcpp::Node::declare_parameter<bool>("axis_idle_on_shutdown", false);
    rclcpp::Node::declare_parameter<int>("heartbeat_timeout_ms", 1000);
    rclcpp::Node::declare_parameter<int>("request_timeout_ms", 2000);
    rclcpp::Node::declare_parameter<double>("trap_vel_limit", 0.0);
    rclcpp::Node::declare_parameter<double>("trap_accel_limit", 0.0);
    rclcpp::Node::declare_parameter<double>("trap_decel_limit", 0.0);

    rclcpp::QoS ctrl_stat_qos(rclcpp::KeepAll{});
    ctrl_publisher_ = rclcpp::Node::create_publisher<ControllerStatus>("controller_status", ctrl_stat_qos);
    
    rclcpp::QoS odrv_stat_qos(rclcpp::KeepAll{});
    odrv_publisher_ = rclcpp::Node::create_publisher<ODriveStatus>("odrive_status", odrv_stat_qos);

    rclcpp::QoS ctrl_msg_qos(rclcpp::KeepAll{});
    subscriber_ = rclcpp::Node::create_subscription<ControlMessage>("control_message", ctrl_msg_qos, std::bind(&ODriveCanNode::subscriber_callback, this, _1));

    rclcpp::QoS srv_qos(rclcpp::KeepAll{});

#if RCLCPP_VERSION_MAJOR >= 18 
    // For ros2 jazzy and above. 
    // PR about deprecation of get_rmw_qos_profile: 
    //  - https://github.com/ros2/rclcpp/pull/713
    //  - https://github.com/ros2/rclcpp/pull/1969
    auto srv_qos_profile = srv_qos;
#else
    auto srv_qos_profile = srv_qos.get_rmw_qos_profile();
#endif

    service_ = rclcpp::Node::create_service<AxisState>("request_axis_state", std::bind(&ODriveCanNode::service_callback, this, _1, _2), srv_qos_profile);
    service_clear_errors_ = rclcpp::Node::create_service<Empty>("clear_errors", std::bind(&ODriveCanNode::service_clear_errors_callback, this, _1, _2), srv_qos_profile);
}

void ODriveCanNode::deinit() {
    if (axis_idle_on_shutdown_) {
        struct can_frame frame = {};
        frame.can_id = node_id_ << 5 | CmdId::kSetAxisState;
        write_le<uint32_t>(ODriveAxisState::AXIS_STATE_IDLE, frame.data);
        frame.can_dlc = 4;
        can_intf_.send_can_frame(frame);
    }

    sub_evt_.deinit();
    srv_evt_.deinit();
    can_intf_.deinit();
}

bool ODriveCanNode::init(EpollEventLoop* event_loop) {

    node_id_ = rclcpp::Node::get_parameter("node_id").as_int();
    axis_idle_on_shutdown_ = rclcpp::Node::get_parameter("axis_idle_on_shutdown").as_bool();
    const auto heartbeat_timeout_ms = rclcpp::Node::get_parameter("heartbeat_timeout_ms").as_int();
    const auto request_timeout_ms = rclcpp::Node::get_parameter("request_timeout_ms").as_int();
    trap_vel_limit_ = rclcpp::Node::get_parameter("trap_vel_limit").as_double();
    trap_accel_limit_ = rclcpp::Node::get_parameter("trap_accel_limit").as_double();
    trap_decel_limit_ = rclcpp::Node::get_parameter("trap_decel_limit").as_double();
    std::string interface = rclcpp::Node::get_parameter("interface").as_string();

    if (heartbeat_timeout_ms <= 0 || request_timeout_ms <= 0) {
        RCLCPP_ERROR(
            rclcpp::Node::get_logger(),
            "heartbeat_timeout_ms and request_timeout_ms must be positive"
        );
        return false;
    }
    heartbeat_timeout_ = std::chrono::milliseconds(heartbeat_timeout_ms);
    request_timeout_ = std::chrono::milliseconds(request_timeout_ms);

    const bool has_any_trap_limit =
        trap_vel_limit_ != 0.0 || trap_accel_limit_ != 0.0 || trap_decel_limit_ != 0.0;
    const bool has_all_trap_limits =
        trap_vel_limit_ > 0.0 && trap_accel_limit_ > 0.0 && trap_decel_limit_ > 0.0;
    if (has_any_trap_limit && !has_all_trap_limits) {
        RCLCPP_ERROR(
            rclcpp::Node::get_logger(),
            "trap_vel_limit, trap_accel_limit and trap_decel_limit must all be positive or all be zero"
        );
        return false;
    }
    trap_traj_limits_enabled_ = has_all_trap_limits;

    if (!can_intf_.init(interface, event_loop, std::bind(&ODriveCanNode::recv_callback, this, _1))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize socket can interface: %s", interface.c_str());
        return false;
    }
    if (!sub_evt_.init(event_loop, std::bind(&ODriveCanNode::ctrl_msg_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize subscriber event");
        return false;
    }
    if (!srv_evt_.init(event_loop, std::bind(&ODriveCanNode::request_state_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize service event");
        return false;
    }
    if (!srv_clear_errors_evt_.init(event_loop, std::bind(&ODriveCanNode::request_clear_errors_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize clear errors service event");
        return false;
    }
    RCLCPP_INFO(rclcpp::Node::get_logger(), "node_id: %d", node_id_);
    RCLCPP_INFO(rclcpp::Node::get_logger(), "interface: %s", interface.c_str());
    return true;
}

void ODriveCanNode::recv_callback(const can_frame& frame) {

    if(((frame.can_id >> 5) & 0x3F) != node_id_) return;

    switch(frame.can_id & 0x1F) {
        case CmdId::kHeartbeat: {
            if (!verify_length("kHeartbeat", 8, frame.can_dlc)) break;
            const auto now = std::chrono::steady_clock::now();
            bool connection_recovered;
            {
                std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
                connection_recovered =
                    !heartbeat_received_ || now - last_heartbeat_ > heartbeat_timeout_;
                ctrl_stat_.active_errors    = read_le<uint32_t>(frame.data + 0);
                ctrl_stat_.axis_state        = read_le<uint8_t>(frame.data + 4);
                ctrl_stat_.procedure_result  = read_le<uint8_t>(frame.data + 5);
                ctrl_stat_.trajectory_done_flag = read_le<bool>(frame.data + 6);
                ctrl_pub_flag_ |= 0b0001;
                heartbeat_received_ = true;
                last_heartbeat_ = now;
                ++heartbeat_generation_;
            }
            fresh_heartbeat_.notify_all();

            if (connection_recovered) {
                std::lock_guard<std::mutex> guard(axis_state_mutex_);
                axis_state_sent_ = false;
                clear_errors_requested_ = false;
            }
            try_send_axis_state_request(ctrl_stat_.active_errors);
            break;
        }
        case CmdId::kGetError: {
            if (!verify_length("kGetError", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.active_errors = read_le<uint32_t>(frame.data + 0);
            odrv_stat_.disarm_reason = read_le<uint32_t>(frame.data + 4);
            odrv_pub_flag_ |= 0b001;
            break;
        }
        case CmdId::kGetEncoderEstimates: {
            if (!verify_length("kGetEncoderEstimates", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.pos_estimate = read_le<float>(frame.data + 0);
            ctrl_stat_.vel_estimate = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b0010;
            break;
        }
        case CmdId::kGetIq: {
            if (!verify_length("kGetIq", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.iq_setpoint = read_le<float>(frame.data + 0);
            ctrl_stat_.iq_measured = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b0100;
            break;
        }
        case CmdId::kGetTemp: {
            if (!verify_length("kGetTemp", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.fet_temperature   = read_le<float>(frame.data + 0);
            odrv_stat_.motor_temperature = read_le<float>(frame.data + 4);
            odrv_pub_flag_ |= 0b010;
            break;
        }
        case CmdId::kGetBusVoltageCurrent: {
            if (!verify_length("kGetBusVoltageCurrent", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.bus_voltage = read_le<float>(frame.data + 0);
            odrv_stat_.bus_current = read_le<float>(frame.data + 4);
            odrv_pub_flag_ |= 0b100;
            break;
        }
        case CmdId::kGetTorques: {
            if (!verify_length("kGetTorques", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.torque_target   = read_le<float>(frame.data + 0);
            ctrl_stat_.torque_estimate = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b1000; 
            break;
        }
        case CmdId::kSetAxisState:
        case CmdId::kSetControllerMode:
        case CmdId::kSetInputPos:
        case CmdId::kSetInputVel:
        case CmdId::kSetInputTorque:
        case CmdId::kSetTrajVelLimit:
        case CmdId::kSetTrajAccelLimits:
        case CmdId::kClearErrors: {
            break; // Ignore commands coming from another master/host on the bus
        }
        default: {
            RCLCPP_WARN(rclcpp::Node::get_logger(), "Received unused message: ID = 0x%x", (frame.can_id & 0x1F));
            break;
        }
    }
    
    if (ctrl_pub_flag_ == 0b1111) {
        ctrl_publisher_->publish(ctrl_stat_);
        ctrl_pub_flag_ = 0;
    }
    
    if (odrv_pub_flag_ == 0b111) {
        odrv_publisher_->publish(odrv_stat_);
        odrv_pub_flag_ = 0;
    }
}

void ODriveCanNode::subscriber_callback(const ControlMessage::SharedPtr msg) {
    std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
    ctrl_msg_ = *msg;
    sub_evt_.set();
}

void ODriveCanNode::service_callback(const std::shared_ptr<AxisState::Request> request, std::shared_ptr<AxisState::Response> response) {
    {
        std::unique_lock<std::mutex> guard(axis_state_mutex_);
        axis_state_ = request->axis_requested_state;
        axis_state_requested_ = true;
        axis_state_sent_ = false;
        clear_errors_requested_ = false;
        RCLCPP_INFO(rclcpp::Node::get_logger(), "requesting axis state: %d", axis_state_);
    }

    uint64_t initial_heartbeat_generation;
    bool heartbeat_available;
    {
        std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
        initial_heartbeat_generation = heartbeat_generation_;
        heartbeat_available =
            heartbeat_received_ &&
            std::chrono::steady_clock::now() - last_heartbeat_ <= heartbeat_timeout_;
    }
    if (heartbeat_available) {
        uint32_t active_errors;
        {
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            active_errors = ctrl_stat_.active_errors;
        }
        try_send_axis_state_request(active_errors);
    } else {
        RCLCPP_INFO(
            rclcpp::Node::get_logger(),
            "Waiting for heartbeat from ODrive node %u before sending axis state",
            node_id_
        );
    }

    // Wait for a heartbeat confirming CLOSED_LOOP_CONTROL. For other requested
    // states, wait for the procedure to complete (procedure_result != BUSY).
    std::unique_lock<std::mutex> guard(ctrl_stat_mutex_); // define lock for controller status
    const bool completed = fresh_heartbeat_.wait_for(guard, request_timeout_, [this, initial_heartbeat_generation, &request]() {
        if (heartbeat_generation_ <= initial_heartbeat_generation) {
            return false;
        }
        bool is_busy = this->ctrl_stat_.procedure_result == ODriveProcedureResult::PROCEDURE_RESULT_BUSY;
        bool requested_closed_loop = request->axis_requested_state == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
        bool reached_requested_state = this->ctrl_stat_.axis_state == request->axis_requested_state;
        bool complete = requested_closed_loop ? reached_requested_state : !is_busy;
        return complete;
        }); // wait for procedure_result

    if (!completed) {
        RCLCPP_WARN(
            rclcpp::Node::get_logger(),
            "Timed out after %ld ms waiting for ODrive node %u; the request will be retried when heartbeat communication resumes",
            request_timeout_.count(),
            node_id_
        );
    }
    
    response->axis_state = ctrl_stat_.axis_state;
    response->active_errors = ctrl_stat_.active_errors;
    response->procedure_result = ctrl_stat_.procedure_result;
}

void ODriveCanNode::try_send_axis_state_request(uint32_t active_errors) {
    std::lock_guard<std::mutex> guard(axis_state_mutex_);
    if (!axis_state_requested_ || axis_state_sent_) {
        return;
    }

    if (active_errors & ODriveError::ODRIVE_ERROR_INITIALIZING) {
        return;
    }

    if (active_errors != ODriveError::ODRIVE_ERROR_NONE) {
        if (!clear_errors_requested_) {
            RCLCPP_WARN(
                rclcpp::Node::get_logger(),
                "ODrive node %u reported active_errors=0x%08x; clearing errors before sending axis state",
                node_id_,
                active_errors
            );
            srv_clear_errors_evt_.set();
            clear_errors_requested_ = true;
        }
        return;
    }

    RCLCPP_INFO(
        rclcpp::Node::get_logger(),
        "Sending requested axis state %u to ODrive node %u",
        axis_state_,
        node_id_
    );
    axis_state_sent_ = true;
    clear_errors_requested_ = false;
    srv_evt_.set();
}

void ODriveCanNode::service_clear_errors_callback(const std::shared_ptr<Empty::Request> /*request*/, std::shared_ptr<Empty::Response> /*response*/) {
    RCLCPP_INFO(rclcpp::Node::get_logger(), "clearing errors");
    srv_clear_errors_evt_.set();
}

void ODriveCanNode::request_state_callback() {
    uint32_t axis_state;
    {
        std::unique_lock<std::mutex> guard(axis_state_mutex_);
        axis_state = axis_state_;
    }

    struct can_frame frame;

    if (axis_state != 0) {
        // Clear errors if requested state is not IDLE
        frame.can_id = node_id_ << 5 | CmdId::kClearErrors;
        write_le<uint8_t>(0, frame.data);
        frame.can_dlc = 1;
        can_intf_.send_can_frame(frame);
    }

    // Set state
    frame.can_id = node_id_ << 5 | CmdId::kSetAxisState;
    write_le<uint32_t>(axis_state, frame.data);
    frame.can_dlc = 4;
    can_intf_.send_can_frame(frame);
}

void ODriveCanNode::request_clear_errors_callback() {
    struct can_frame frame = {};
    frame.can_id = node_id_ << 5 | CmdId::kClearErrors;
    write_le<uint8_t>(0, frame.data);
    frame.can_dlc = 1;
    can_intf_.send_can_frame(frame);
}

void ODriveCanNode::ctrl_msg_callback() {

    uint32_t control_mode;
    uint32_t input_mode;
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | kSetControllerMode;
    {
        std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
        write_le<uint32_t>(ctrl_msg_.control_mode, frame.data);
        write_le<uint32_t>(ctrl_msg_.input_mode,   frame.data + 4);
        control_mode = ctrl_msg_.control_mode;
        input_mode = ctrl_msg_.input_mode;
    }

    if (input_mode == kTrapTrajInputMode && trap_traj_limits_enabled_) {
        struct can_frame limit_frame = {};
        limit_frame.can_id = node_id_ << 5 | CmdId::kSetTrajVelLimit;
        write_le<float>(static_cast<float>(trap_vel_limit_ / (2 * M_PI)), limit_frame.data);
        limit_frame.can_dlc = 4;
        can_intf_.send_can_frame(limit_frame);

        limit_frame = can_frame{};
        limit_frame.can_id = node_id_ << 5 | CmdId::kSetTrajAccelLimits;
        write_le<float>(static_cast<float>(trap_accel_limit_ / (2 * M_PI)), limit_frame.data);
        write_le<float>(static_cast<float>(trap_decel_limit_ / (2 * M_PI)), limit_frame.data + 4);
        limit_frame.can_dlc = 8;
        can_intf_.send_can_frame(limit_frame);
    }
    frame.can_dlc = 8;
    can_intf_.send_can_frame(frame);
    
    frame = can_frame{};
    switch (control_mode) {
        case ControlMode::kVoltageControl: {
            RCLCPP_ERROR(rclcpp::Node::get_logger(), "Voltage Control Mode (0) is not currently supported");
            return;
        }
        case ControlMode::kTorqueControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_torque");
            frame.can_id = node_id_ << 5 | kSetInputTorque;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_torque, frame.data);
            frame.can_dlc = 4;
            break;
        }
        case ControlMode::kVelocityControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_vel");
            frame.can_id = node_id_ << 5 | kSetInputVel;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_vel,       frame.data);
            write_le<float>(ctrl_msg_.input_torque, frame.data + 4);
            frame.can_dlc = 8;
            break;
        }
        case ControlMode::kPositionControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_pos");
            frame.can_id = node_id_ << 5 | kSetInputPos;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_pos,  frame.data);
            write_le<int8_t>(((int8_t)((ctrl_msg_.input_vel) * 1000)),    frame.data + 4);
            write_le<int8_t>(((int8_t)((ctrl_msg_.input_torque) * 1000)), frame.data + 6);
            frame.can_dlc = 8;
            break;
        }    
        default: 
            RCLCPP_ERROR(rclcpp::Node::get_logger(), "unsupported control_mode: %d", control_mode);
            return;
    }

    can_intf_.send_can_frame(frame);
}

inline bool ODriveCanNode::verify_length(const std::string&name, uint8_t expected, uint8_t length) {
    bool valid = expected == length;
    RCLCPP_DEBUG(rclcpp::Node::get_logger(), "received %s", name.c_str());
    if (!valid) RCLCPP_WARN(rclcpp::Node::get_logger(), "Incorrect %s frame length: %d != %d", name.c_str(), length, expected);
    return valid;
}
