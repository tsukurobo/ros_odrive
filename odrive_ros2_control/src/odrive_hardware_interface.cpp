
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_enums.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "socket_can.hpp"
#include <chrono>

namespace odrive_ros2_control {

class Axis;

class ODriveHardwareInterface final : public hardware_interface::SystemInterface {
public:
    using return_type = hardware_interface::return_type;
    using State = rclcpp_lifecycle::State;

    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    CallbackReturn on_configure(const State& previous_state) override;
    CallbackReturn on_cleanup(const State& previous_state) override;
    CallbackReturn on_activate(const State& previous_state) override;
    CallbackReturn on_deactivate(const State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
    ) override;

    return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
    return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
    void on_can_msg(const can_frame& frame);
    void set_axis_command_mode(Axis& axis);
    void start_index_search(Axis& axis);
    void update_index_search(Axis& axis);
    void update_connection(Axis& axis);

    bool active_ = false;
    EpollEventLoop event_loop_;
    std::vector<Axis> axes_;
    std::string can_intf_name_;
    SocketCanIntf can_intf_;
    rclcpp::Time timestamp_;
    std::chrono::milliseconds heartbeat_timeout_{1000};
};

struct Axis {
    Axis(SocketCanIntf* can_intf, uint32_t node_id) : can_intf_(can_intf), node_id_(node_id) {}

    void on_can_msg(const rclcpp::Time& timestamp, const can_frame& frame);

    void on_can_msg();

    SocketCanIntf* can_intf_;
    uint32_t node_id_;

    bool index_search_on_activate_ = false;
    bool index_search_requested_ = false;
    bool index_search_started_ = false;
    bool ready_for_control_ = true;
    bool heartbeat_received_ = false;
    bool connection_recovered_ = false;
    bool initialization_pending_ = false;
    bool clear_errors_requested_ = false;
    std::chrono::steady_clock::time_point last_heartbeat_;
    uint32_t axis_error_ = 0;
    uint8_t axis_state_ = AXIS_STATE_UNDEFINED;
    uint8_t procedure_result_ = PROCEDURE_RESULT_SUCCESS;

    // Commands (ros2_control => ODrives)
    double pos_setpoint_ = 0.0f; // [rad]
    double vel_setpoint_ = 0.0f; // [rad/s]
    double torque_setpoint_ = 0.0f; // [Nm]

    // State (ODrives => ros2_control)
    // rclcpp::Time encoder_estimates_timestamp_;
    // uint32_t axis_error_ = 0;
    // uint8_t axis_state_ = 0;
    // uint8_t procedure_result_ = 0;
    // uint8_t trajectory_done_flag_ = 0;
    double pos_estimate_ = NAN; // [rad]
    double vel_estimate_ = NAN; // [rad/s]
    double iq_setpoint_ = NAN; // [A]
    double iq_measured_ = NAN; // [A]
    double torque_target_ = NAN; // [Nm]
    double torque_estimate_ = NAN; // [Nm]
    // uint32_t active_errors_ = 0;
    // uint32_t disarm_reason_ = 0;
    // double fet_temperature_ = NAN;
    // double motor_temperature_ = NAN;
    // double bus_voltage_ = NAN;
    // double bus_current_ = NAN;

    // Indicates which controller inputs are enabled. This is configured by the
    // controller that sits on top of this hardware interface. Multiple inputs
    // can be enabled at the same time, in this case the non-primary inputs are
    // used as feedforward terms.
    // This implicitly defines the ODrive's control mode.
    bool pos_input_enabled_ = false;
    bool vel_input_enabled_ = false;
    bool torque_input_enabled_ = false;

    // Optional ODrive-side trapezoidal trajectory limits. The hardware
    // interface uses SI units; CAN Simple uses revolutions.
    double trap_vel_limit_ = 0.0; // [rad/s]
    double trap_accel_limit_ = 0.0; // [rad/s^2]
    double trap_decel_limit_ = 0.0; // [rad/s^2]
    bool trap_traj_enabled_ = false;

    template <typename T>
    void send(const T& msg) const {
        struct can_frame frame{};
        frame.can_id = node_id_ << 5 | msg.cmd_id;
        frame.can_dlc = msg.msg_length;
        msg.encode_buf(frame.data);

        can_intf_->send_can_frame(frame);
    }
};

} // namespace odrive_ros2_control

using namespace odrive_ros2_control;

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn ODriveHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    can_intf_name_ = info_.hardware_parameters["can"];
    if (const auto timeout = info_.hardware_parameters.find("heartbeat_timeout_ms");
        timeout != info_.hardware_parameters.end()) {
        const int timeout_ms = std::stoi(timeout->second);
        if (timeout_ms <= 0) {
            RCLCPP_ERROR(
                rclcpp::get_logger("ODriveHardwareInterface"),
                "heartbeat_timeout_ms must be positive"
            );
            return CallbackReturn::ERROR;
        }
        heartbeat_timeout_ = std::chrono::milliseconds(timeout_ms);
    }

    for (auto& joint : info_.joints) {
        axes_.emplace_back(&can_intf_, std::stoi(joint.parameters.at("node_id")));
        Axis& axis = axes_.back();
        const auto index_search = joint.parameters.find("index_search_on_activate");
        axis.index_search_on_activate_ =
            index_search != joint.parameters.end() &&
            (index_search->second == "true" || index_search->second == "1");
        axis.ready_for_control_ = !axis.index_search_on_activate_;

        const auto trap_vel = joint.parameters.find("trap_vel_limit");
        const auto trap_accel = joint.parameters.find("trap_accel_limit");
        const auto trap_decel = joint.parameters.find("trap_decel_limit");
        const bool has_any_trap_limit =
            trap_vel != joint.parameters.end() || trap_accel != joint.parameters.end() ||
            trap_decel != joint.parameters.end();
        const bool has_all_trap_limits =
            trap_vel != joint.parameters.end() && trap_accel != joint.parameters.end() &&
            trap_decel != joint.parameters.end();
        if (has_any_trap_limit && !has_all_trap_limits) {
            RCLCPP_ERROR(
                rclcpp::get_logger("ODriveHardwareInterface"),
                "Joint %s must specify trap_vel_limit, trap_accel_limit and trap_decel_limit together",
                joint.name.c_str()
            );
            return CallbackReturn::ERROR;
        }
        if (has_all_trap_limits) {
            axis.trap_vel_limit_ = std::stod(trap_vel->second);
            axis.trap_accel_limit_ = std::stod(trap_accel->second);
            axis.trap_decel_limit_ = std::stod(trap_decel->second);
            if (axis.trap_vel_limit_ <= 0.0 || axis.trap_accel_limit_ <= 0.0 ||
                axis.trap_decel_limit_ <= 0.0) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("ODriveHardwareInterface"),
                    "Trapezoidal trajectory limits for joint %s must be positive",
                    joint.name.c_str()
                );
                return CallbackReturn::ERROR;
            }
            axis.trap_traj_enabled_ = true;
        }
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_configure(const State&) {
    if (!can_intf_.init(can_intf_name_, &event_loop_, std::bind(&ODriveHardwareInterface::on_can_msg, this, _1))) {
        RCLCPP_ERROR(
            rclcpp::get_logger("ODriveHardwareInterface"),
            "Failed to initialize SocketCAN on %s",
            can_intf_name_.c_str()
        );
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Initialized SocketCAN on %s", can_intf_name_.c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_cleanup(const State&) {
    can_intf_.deinit();
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_activate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "activating ODrives...");

    // This can be called several seconds before the controller finishes starting.
    // Therefore we enable the ODrives only in perform_command_mode_switch().

    active_ = true;
    for (auto& axis : axes_) {
        if (!axis.heartbeat_received_) {
            RCLCPP_INFO(
                rclcpp::get_logger("ODriveHardwareInterface"),
                "Waiting for heartbeat from ODrive node %u before initialization",
                axis.node_id_
            );
            continue;
        }

        // If a heartbeat arrived while the hardware was inactive, activation
        // handles it here. Do not handle the same connection event again in read().
        axis.connection_recovered_ = false;
        axis.initialization_pending_ = true;
        update_connection(axis);
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_deactivate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "deactivating ODrives...");

    active_ = false;
    for (auto& axis : axes_) {
        set_axis_command_mode(axis);
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_target_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_estimate_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_estimate_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            "iq_setpoint",
            &axes_[i].iq_setpoint_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            "iq_measured",
            &axes_[i].iq_measured_
        ));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ODriveHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_setpoint_
        ));
    }

    return command_interfaces;
}

return_type ODriveHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces
) {
    for (size_t i = 0; i < axes_.size(); ++i) {
        Axis& axis = axes_[i];
        std::array<std::pair<std::string, bool*>, 3> interfaces = {
            {{info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION, &axis.pos_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY, &axis.vel_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT, &axis.torque_input_enabled_}}};

        bool mode_switch = false;

        for (const std::string& key : stop_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = false;
                    mode_switch = true;
                }
            }
        }

        for (const std::string& key : start_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = true;
                    mode_switch = true;
                }
            }
        }

        if (mode_switch) {
            set_axis_command_mode(axis);
        }
    }

    return return_type::OK;
}

return_type ODriveHardwareInterface::read(const rclcpp::Time& timestamp, const rclcpp::Duration&) {
    timestamp_ = timestamp;

    while (can_intf_.read_nonblocking()) {
        // repeat until CAN interface has no more messages
    }

    for (auto& axis : axes_) {
        update_connection(axis);
        update_index_search(axis);
    }

    return return_type::OK;
}

void ODriveHardwareInterface::update_connection(Axis& axis) {
    if (axis.heartbeat_received_ &&
        std::chrono::steady_clock::now() - axis.last_heartbeat_ > heartbeat_timeout_) {
        RCLCPP_WARN(
            rclcpp::get_logger("ODriveHardwareInterface"),
            "Heartbeat timed out for ODrive node %u; waiting for it to reconnect",
            axis.node_id_
        );
        axis.heartbeat_received_ = false;
        axis.connection_recovered_ = false;
        axis.initialization_pending_ = false;
        axis.clear_errors_requested_ = false;
        axis.index_search_requested_ = false;
        axis.index_search_started_ = false;
        axis.ready_for_control_ = !axis.index_search_on_activate_;
    }

    if (axis.connection_recovered_) {
        axis.connection_recovered_ = false;
        axis.initialization_pending_ = true;
        axis.clear_errors_requested_ = false;
        RCLCPP_INFO(
            rclcpp::get_logger("ODriveHardwareInterface"),
            "Heartbeat received from ODrive node %u",
            axis.node_id_
        );
    }

    if (!axis.initialization_pending_ || !active_) {
        return;
    }

    // The first heartbeat after power-up can still report INITIALIZING. Sending
    // an axis-state request during boot can disarm the procedure and leave the
    // ODrive's LED red, so wait for initialization to finish.
    if (axis.axis_error_ & ODRIVE_ERROR_INITIALIZING) {
        return;
    }

    // Clear a persistent error first, then wait for a subsequent heartbeat to
    // confirm that it was cleared before starting index search.
    if (axis.axis_error_ != ODRIVE_ERROR_NONE) {
        if (!axis.clear_errors_requested_) {
            RCLCPP_WARN(
                rclcpp::get_logger("ODriveHardwareInterface"),
                "ODrive node %u reported axis_error=0x%08x; clearing errors before initialization",
                axis.node_id_,
                axis.axis_error_
            );
            Clear_Errors_msg_t clear_error_msg;
            clear_error_msg.Identify = 0;
            axis.send(clear_error_msg);
            axis.clear_errors_requested_ = true;
        }
        return;
    }

    axis.initialization_pending_ = false;
    axis.clear_errors_requested_ = false;
    if (axis.index_search_on_activate_) {
        start_index_search(axis);
    } else {
        set_axis_command_mode(axis);
    }
}

return_type ODriveHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
    for (auto& axis : axes_) {
        if (!axis.ready_for_control_) {
            continue;
        }
        // Send the CAN message that fits the set of enabled setpoints
        if (axis.pos_input_enabled_) {
            Set_Input_Pos_msg_t msg;
            msg.Input_Pos = axis.pos_setpoint_ / (2 * M_PI);
            msg.Vel_FF = axis.vel_input_enabled_ ? (axis.vel_setpoint_ / (2 * M_PI)) : 0.0f;
            msg.Torque_FF = axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;
            axis.send(msg);
        } else if (axis.vel_input_enabled_) {
            Set_Input_Vel_msg_t msg;
            msg.Input_Vel = axis.vel_setpoint_ / (2 * M_PI);
            msg.Input_Torque_FF = axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;
            axis.send(msg);
        } else if (axis.torque_input_enabled_) {
            Set_Input_Torque_msg_t msg;
            msg.Input_Torque = axis.torque_setpoint_;
            axis.send(msg);
        } else {
            // no control enabled - don't send any setpoint
        }
    }

    return return_type::OK;
}

void ODriveHardwareInterface::on_can_msg(const can_frame& frame) {
    for (auto& axis : axes_) {
        if ((frame.can_id >> 5) == axis.node_id_) {
            axis.on_can_msg(timestamp_, frame);
        }
    }
}

void ODriveHardwareInterface::start_index_search(Axis& axis) {
    RCLCPP_INFO(
        rclcpp::get_logger("ODriveHardwareInterface"),
        "Starting encoder index search for ODrive node %u",
        axis.node_id_
    );

    Clear_Errors_msg_t clear_error_msg;
    clear_error_msg.Identify = 0;
    axis.send(clear_error_msg);

    Set_Axis_State_msg_t state_msg;
    state_msg.Axis_Requested_State = AXIS_STATE_ENCODER_INDEX_SEARCH;
    axis.send(state_msg);

    axis.index_search_requested_ = true;
    axis.index_search_started_ = false;
    axis.ready_for_control_ = false;
}

void ODriveHardwareInterface::update_index_search(Axis& axis) {
    if (!axis.index_search_requested_ || !axis.heartbeat_received_) {
        return;
    }

    if (axis.axis_state_ == AXIS_STATE_ENCODER_INDEX_SEARCH) {
        axis.index_search_started_ = true;
        return;
    }

    if (!axis.index_search_started_) {
        return;
    }

    if (axis.axis_state_ == AXIS_STATE_IDLE) {
        axis.index_search_requested_ = false;
        if (axis.axis_error_ == ODRIVE_ERROR_NONE &&
            axis.procedure_result_ == PROCEDURE_RESULT_SUCCESS) {
            axis.ready_for_control_ = true;
            RCLCPP_INFO(
                rclcpp::get_logger("ODriveHardwareInterface"),
                "Encoder index search completed for ODrive node %u",
                axis.node_id_
            );
            set_axis_command_mode(axis);
        } else {
            RCLCPP_ERROR(
                rclcpp::get_logger("ODriveHardwareInterface"),
                "Encoder index search failed for ODrive node %u: axis_error=0x%08x, procedure_result=%u",
                axis.node_id_,
                axis.axis_error_,
                axis.procedure_result_
            );
        }
    }
}

void ODriveHardwareInterface::set_axis_command_mode(Axis& axis) {
    if (!active_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Interface inactive. Setting axis to idle.");
        Set_Axis_State_msg_t idle_msg;
        idle_msg.Axis_Requested_State = AXIS_STATE_IDLE;
        axis.send(idle_msg);
        return;
    }

    if (!axis.ready_for_control_) {
        RCLCPP_INFO(
            rclcpp::get_logger("ODriveHardwareInterface"),
            "Deferring control mode for ODrive node %u until encoder index search completes",
            axis.node_id_
        );
        return;
    }

    Set_Controller_Mode_msg_t control_msg;
    Clear_Errors_msg_t clear_error_msg;
    Set_Axis_State_msg_t state_msg;

    clear_error_msg.Identify = 0;
    control_msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
    state_msg.Axis_Requested_State = AXIS_STATE_CLOSED_LOOP_CONTROL;

    if (axis.pos_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting to position control.");
        control_msg.Control_Mode = CONTROL_MODE_POSITION_CONTROL;
        if (axis.trap_traj_enabled_) {
            Set_Traj_Vel_Limit_msg_t vel_limit_msg;
            vel_limit_msg.Traj_Vel_Limit = axis.trap_vel_limit_ / (2 * M_PI);
            axis.send(vel_limit_msg);

            Set_Traj_Accel_Limits_msg_t accel_limits_msg;
            accel_limits_msg.Traj_Accel_Limit = axis.trap_accel_limit_ / (2 * M_PI);
            accel_limits_msg.Traj_Decel_Limit = axis.trap_decel_limit_ / (2 * M_PI);
            axis.send(accel_limits_msg);

            control_msg.Input_Mode = INPUT_MODE_TRAP_TRAJ;
            RCLCPP_INFO(
                rclcpp::get_logger("ODriveHardwareInterface"),
                "Using trapezoidal trajectory: velocity=%.3f rad/s, acceleration=%.3f rad/s^2, deceleration=%.3f rad/s^2",
                axis.trap_vel_limit_,
                axis.trap_accel_limit_,
                axis.trap_decel_limit_
            );
        }
    } else if (axis.vel_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting to velocity control.");
        control_msg.Control_Mode = CONTROL_MODE_VELOCITY_CONTROL;
    } else if (axis.torque_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting to torque control.");
        control_msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "No control mode specified. Setting to idle.");
        state_msg.Axis_Requested_State = AXIS_STATE_IDLE;
        axis.send(state_msg);
        return;
    }

    axis.send(control_msg);
    axis.send(clear_error_msg);
    axis.send(state_msg);
}

void Axis::on_can_msg(const rclcpp::Time&, const can_frame& frame) {
    uint8_t cmd = frame.can_id & 0x1f;

    auto try_decode = [&]<typename TMsg>(TMsg& msg) {
        if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
            RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"), "message %d too short", cmd);
            return false;
        }
        msg.decode_buf(frame.data);
        return true;
    };

    switch (cmd) {
        case Heartbeat_msg_t::cmd_id: {
            if (Heartbeat_msg_t msg; try_decode(msg)) {
                axis_error_ = msg.Axis_Error;
                axis_state_ = msg.Axis_State;
                procedure_result_ = msg.Procedure_Result;
                last_heartbeat_ = std::chrono::steady_clock::now();
                if (!heartbeat_received_) {
                    connection_recovered_ = true;
                }
                heartbeat_received_ = true;
            }
        } break;
        case Get_Encoder_Estimates_msg_t::cmd_id: {
            if (Get_Encoder_Estimates_msg_t msg; try_decode(msg)) {
                pos_estimate_ = msg.Pos_Estimate * (2 * M_PI);
                vel_estimate_ = msg.Vel_Estimate * (2 * M_PI);
            }
        } break;
        case Get_Torques_msg_t::cmd_id: {
            if (Get_Torques_msg_t msg; try_decode(msg)) {
                torque_target_ = msg.Torque_Target;
                torque_estimate_ = msg.Torque_Estimate;
            }
        } break;
        case Get_Iq_msg_t::cmd_id: {
            if (Get_Iq_msg_t msg; try_decode(msg)) {
                iq_setpoint_ = msg.Iq_Setpoint;
                iq_measured_ = msg.Iq_Measured;
            }
        } break;
            // silently ignore unimplemented command IDs
    }
}

PLUGINLIB_EXPORT_CLASS(odrive_ros2_control::ODriveHardwareInterface, hardware_interface::SystemInterface)
