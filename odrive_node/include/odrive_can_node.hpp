#ifndef ODRIVE_CAN_NODE_HPP
#define ODRIVE_CAN_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>
#include "odrive_can/msg/o_drive_status.hpp"
#include "odrive_can/msg/controller_status.hpp"
#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/srv/axis_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include "socket_can.hpp"

#include <mutex>
#include <condition_variable>
#include <array>
#include <algorithm>
#include <chrono>
#include <linux/can.h>
#include <linux/can/raw.h>

using std::placeholders::_1;
using std::placeholders::_2;

using ODriveStatus = odrive_can::msg::ODriveStatus;
using ControllerStatus = odrive_can::msg::ControllerStatus;
using ControlMessage = odrive_can::msg::ControlMessage;

using AxisState = odrive_can::srv::AxisState;
using Empty = std_srvs::srv::Empty;

class ODriveCanNode : public rclcpp::Node {
public:
    ODriveCanNode(const std::string& node_name);
    bool init(EpollEventLoop* event_loop); 
    void deinit();
private:
    void recv_callback(const can_frame& frame);
    void subscriber_callback(const ControlMessage::SharedPtr msg);
    void service_callback(const std::shared_ptr<AxisState::Request> request, std::shared_ptr<AxisState::Response> response);
    void service_clear_errors_callback(const std::shared_ptr<Empty::Request> request, std::shared_ptr<Empty::Response> response);
    void request_state_callback();
    void request_clear_errors_callback();
    void ctrl_msg_callback();
    void try_send_axis_state_request(uint32_t active_errors);
    inline bool verify_length(const std::string&name, uint8_t expected, uint8_t length);
    
    uint16_t node_id_;
    bool axis_idle_on_shutdown_;
    SocketCanIntf can_intf_ = SocketCanIntf();
    
    short int ctrl_pub_flag_ = 0;
    std::mutex ctrl_stat_mutex_;
    ControllerStatus ctrl_stat_ = ControllerStatus();
    rclcpp::Publisher<ControllerStatus>::SharedPtr ctrl_publisher_;
    
    short int odrv_pub_flag_ = 0;
    std::mutex odrv_stat_mutex_;
    ODriveStatus odrv_stat_ = ODriveStatus();
    rclcpp::Publisher<ODriveStatus>::SharedPtr odrv_publisher_;

    EpollEvent sub_evt_;
    std::mutex ctrl_msg_mutex_;
    ControlMessage ctrl_msg_ = ControlMessage();
    rclcpp::Subscription<ControlMessage>::SharedPtr subscriber_;

    EpollEvent srv_evt_;
    uint32_t axis_state_ = 0;
    bool axis_state_requested_ = false;
    bool axis_state_sent_ = false;
    bool clear_errors_requested_ = false;
    std::mutex axis_state_mutex_;
    std::condition_variable fresh_heartbeat_;
    uint64_t heartbeat_generation_ = 0;
    bool heartbeat_received_ = false;
    std::chrono::steady_clock::time_point last_heartbeat_;
    std::chrono::milliseconds heartbeat_timeout_{1000};
    std::chrono::milliseconds request_timeout_{2000};
    rclcpp::Service<AxisState>::SharedPtr service_;

    double trap_vel_limit_ = 0.0;
    double trap_accel_limit_ = 0.0;
    double trap_decel_limit_ = 0.0;
    bool trap_traj_limits_enabled_ = false;

    EpollEvent srv_clear_errors_evt_;
    rclcpp::Service<Empty>::SharedPtr service_clear_errors_;

};

#endif // ODRIVE_CAN_NODE_HPP
