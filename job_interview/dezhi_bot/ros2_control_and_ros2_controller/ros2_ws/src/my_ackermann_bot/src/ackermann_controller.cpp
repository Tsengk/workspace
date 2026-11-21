#include "my_ackermann_bot/ackermann_controller.hpp"

#include <algorithm>
#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_ackermann_bot
{

controller_interface::return_type AckermannController::init(const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  // 声明 YAML 中可配置的参数
  auto_declare<std::string>("front_left_steer_joint", "front_left_steer_joint");
  auto_declare<std::string>("front_right_steer_joint", "front_right_steer_joint");
  auto_declare<std::string>("rear_left_wheel_joint", "rear_left_wheel_joint");
  auto_declare<std::string>("rear_right_wheel_joint", "rear_right_wheel_joint");

  auto_declare<double>("wheelbase", 0.32);
  auto_declare<double>("track_width", 0.26);
  auto_declare<double>("wheel_radius", 0.05);
  auto_declare<double>("max_steer_angle", 0.6);
  auto_declare<double>("cmd_vel_timeout", 0.5);

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
AckermannController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // 按固定顺序向 Resource Manager 申请 4 个命令接口
  conf.names = {
    front_left_steer_joint_ + "/" + hardware_interface::HW_IF_POSITION,
    front_right_steer_joint_ + "/" + hardware_interface::HW_IF_POSITION,
    rear_left_wheel_joint_ + "/" + hardware_interface::HW_IF_VELOCITY,
    rear_right_wheel_joint_ + "/" + hardware_interface::HW_IF_VELOCITY
  };
  return conf;
}

controller_interface::InterfaceConfiguration
AckermannController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names = {
    front_left_steer_joint_ + "/" + hardware_interface::HW_IF_POSITION,
    front_right_steer_joint_ + "/" + hardware_interface::HW_IF_POSITION,
    rear_left_wheel_joint_ + "/" + hardware_interface::HW_IF_VELOCITY,
    rear_right_wheel_joint_ + "/" + hardware_interface::HW_IF_VELOCITY
  };
  return conf;
}

CallbackReturn AckermannController::on_configure(const rclcpp_lifecycle::State &)
{
  front_left_steer_joint_ = get_node()->get_parameter("front_left_steer_joint").as_string();
  front_right_steer_joint_ = get_node()->get_parameter("front_right_steer_joint").as_string();
  rear_left_wheel_joint_ = get_node()->get_parameter("rear_left_wheel_joint").as_string();
  rear_right_wheel_joint_ = get_node()->get_parameter("rear_right_wheel_joint").as_string();

  wheelbase_ = get_node()->get_parameter("wheelbase").as_double();
  track_width_ = get_node()->get_parameter("track_width").as_double();
  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
  max_steer_angle_ = get_node()->get_parameter("max_steer_angle").as_double();
  cmd_vel_timeout_ = get_node()->get_parameter("cmd_vel_timeout").as_double();

  rt_cmd_buffer_.writeFromNonRT(geometry_msgs::msg::Twist());

  // 订阅 /ackermann_controller/cmd_vel 话题
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      rt_cmd_buffer_.writeFromNonRT(*msg);
      last_cmd_ns_.store(get_node()->now().nanoseconds());
    });

  RCLCPP_INFO(
    get_node()->get_logger(),
    "AckermannController configured (wheelbase=%.3f, track_width=%.3f, wheel_radius=%.3f)",
    wheelbase_, track_width_, wheel_radius_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn AckermannController::on_activate(const rclcpp_lifecycle::State &)
{
  for (auto & cmd_if : command_interfaces_) {
    cmd_if.set_value(0.0);
  }
  last_cmd_ns_.store(get_node()->now().nanoseconds());
  return CallbackReturn::SUCCESS;
}

CallbackReturn AckermannController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & cmd_if : command_interfaces_) {
    cmd_if.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type AckermannController::update()
{
  const auto cmd = *rt_cmd_buffer_.readFromRT();
  double v = cmd.linear.x;
  double w = cmd.angular.z;

  // 超时安全保护：超过 cmd_vel_timeout 未收到新指令则归零
  const int64_t elapsed_ns = get_node()->now().nanoseconds() - last_cmd_ns_.load();
  if (elapsed_ns > static_cast<int64_t>(cmd_vel_timeout_ * 1e9)) {
    v = 0.0;
    w = 0.0;
  }

  double left_steer = 0.0;
  double right_steer = 0.0;
  double left_wheel_vel = 0.0;
  double right_wheel_vel = 0.0;

  if (std::abs(w) < 1e-5) {
    // 直线行驶
    left_steer = 0.0;
    right_steer = 0.0;
    left_wheel_vel = v / wheel_radius_;
    right_wheel_vel = v / wheel_radius_;
  } else if (std::abs(v) < 1e-4) {
    // 零车速仅打方向（方便遥控测试观察前轮转向）
    left_steer = std::copysign(max_steer_angle_, w);
    right_steer = std::copysign(max_steer_angle_, w);
    left_wheel_vel = 0.0;
    right_wheel_vel = 0.0;
  } else {
    // 阿克曼转向几何解算
    double R_r = v / w;
    double center_steer = std::atan(wheelbase_ / R_r);
    center_steer = std::max(-max_steer_angle_, std::min(max_steer_angle_, center_steer));

    // 根据限幅后的等价转角反推实际后轴转弯半径 R_r
    R_r = wheelbase_ / std::tan(center_steer);
    const double effective_w = v / R_r;

    left_steer = std::atan(wheelbase_ / (R_r - track_width_ / 2.0));
    right_steer = std::atan(wheelbase_ / (R_r + track_width_ / 2.0));

    left_wheel_vel = (effective_w * (R_r - track_width_ / 2.0)) / wheel_radius_;
    right_wheel_vel = (effective_w * (R_r + track_width_ / 2.0)) / wheel_radius_;
  }

  // 将计算结果写入 Command Interface，传递给 GazeboSimSystem
  command_interfaces_[0].set_value(left_steer);
  command_interfaces_[1].set_value(right_steer);
  command_interfaces_[2].set_value(left_wheel_vel);
  command_interfaces_[3].set_value(right_wheel_vel);

  return controller_interface::return_type::OK;
}

}  // namespace my_ackermann_bot

PLUGINLIB_EXPORT_CLASS(
  my_ackermann_bot::AckermannController,
  controller_interface::ControllerInterface)