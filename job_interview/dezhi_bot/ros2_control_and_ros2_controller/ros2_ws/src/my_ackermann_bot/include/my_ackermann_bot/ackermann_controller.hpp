#ifndef MY_ACKERMANN_BOT__ACKERMANN_CONTROLLER_HPP_
#define MY_ACKERMANN_BOT__ACKERMANN_CONTROLLER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"

namespace my_ackermann_bot
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AckermannController : public controller_interface::ControllerInterface
{
public:
  AckermannController() = default;

  // ROS 2 Foxy 控制器初始化接口
  controller_interface::return_type init(const std::string & controller_name) override;

  // 声明向硬件下发的命令接口与读取的状态接口
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // 生命周期回调
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // ROS 2 Foxy 实时控制主循环
  controller_interface::return_type update() override;

private:
  // 关节名称
  std::string front_left_steer_joint_;
  std::string front_right_steer_joint_;
  std::string rear_left_wheel_joint_;
  std::string rear_right_wheel_joint_;

  // 车辆几何参数
  double wheelbase_{0.32};        // 轴距 L (m)
  double track_width_{0.26};      // 轮距 w (m)
  double wheel_radius_{0.05};     // 驱动轮半径 r (m)
  double max_steer_angle_{0.6};   // 最大前轮转向角 (rad)
  double cmd_vel_timeout_{0.5};   // 速度指令超时自动停车时间 (s)
  double max_linear_velocity_{1.5};
  double max_angular_velocity_{3.0};
  bool allow_stationary_steering_{true};

  // 实时安全速度缓冲区与时间戳
  struct Command
  {
    Command() : twist() {}
    geometry_msgs::msg::Twist twist;
    std::chrono::steady_clock::time_point stamp;
    bool valid{false};
    uint64_t epoch{0};
  };
  realtime_tools::RealtimeBuffer<Command> rt_cmd_buffer_;
  std::atomic<bool> accepting_commands_{false};
  std::atomic<uint64_t> activation_epoch_{0};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

}  // namespace my_ackermann_bot

#endif  // MY_ACKERMANN_BOT__ACKERMANN_CONTROLLER_HPP_
