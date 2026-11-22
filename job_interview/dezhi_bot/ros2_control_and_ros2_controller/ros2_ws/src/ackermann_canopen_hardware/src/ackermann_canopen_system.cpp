#include "ackermann_canopen_hardware/ackermann_canopen_system.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>
#include <unordered_map>

#include "canopen_interfaces/srv/co_read.hpp"
#include "canopen_interfaces/srv/co_target_double.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace ackermann_canopen_hardware
{
namespace
{
using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
using Trigger = std_srvs::srv::Trigger;
using Target = canopen_interfaces::srv::COTargetDouble;
using Read = canopen_interfaces::srv::CORead;
using Parameters = std::unordered_map<std::string, std::string>;
constexpr size_t joint_count = 4;

std::string required(const Parameters & parameters, const std::string & key)
{
  const auto found = parameters.find(key);
  if (found == parameters.end() || found->second.empty()) {
    throw std::invalid_argument("Missing hardware parameter: " + key);
  }
  return found->second;
}

double number(const Parameters & parameters, const std::string & key)
{
  const auto text = required(parameters, key);
  size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (consumed != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument("Invalid finite number: " + key);
  }
  return value;
}

bool boolean(const Parameters & parameters, const std::string & key)
{
  const auto text = required(parameters, key);
  if (text == "true" || text == "True" || text == "1") {
    return true;
  }
  if (text == "false" || text == "False" || text == "0") {
    return false;
  }
  throw std::invalid_argument("Invalid boolean: " + key);
}

std::chrono::milliseconds duration(const Parameters & parameters, const std::string & key)
{
  const double value = number(parameters, key);
  if (value < 1.0 || value > 60000.0 || std::floor(value) != value) {
    throw std::invalid_argument("Expected integer milliseconds in [1, 60000]: " + key);
  }
  return std::chrono::milliseconds(static_cast<int64_t>(value));
}

bool fresh(Time stamp, Time now, std::chrono::milliseconds timeout)
{
  return stamp != Time{} && now - stamp <= timeout;
}
}

class AckermannCanopenSystem::Impl
{
public:
  struct Motor
  {
    std::string name;
    std::string node;
    std::string command_interface;
    double position_scale;
    double velocity_scale;
    double offset;
    double direction;
    double lower;
    double upper;
    double max_velocity;
    double max_acceleration;
    uint8_t mode;
    double last_sent = 0.0;
    bool target_pending = false;
    bool read_pending = false;
    bool reading_mode = false;
    bool halt_sent = false;
    bool halt_pending = false;
    Time target_sent;
    Time read_sent;
    Time halt_stamp;
    Time next_poll;
    rclcpp::Client<Target>::SharedPtr target;
    rclcpp::Client<Read>::SharedPtr sdo_read;
    rclcpp::Client<Trigger>::SharedPtr halt;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub;
  };

  struct Feedback
  {
    std::array<double, joint_count> positions;
    std::array<double, joint_count> velocities;
    std::array<Time, joint_count> position_stamp{};
    std::array<Time, joint_count> velocity_stamp{};
    std::array<Time, joint_count> status_stamp{};
    std::array<Time, joint_count> mode_stamp{};
    std::array<uint16_t, joint_count> status_word{};
    std::array<uint8_t, joint_count> mode{};
  };

  explicit Impl(const hardware_interface::HardwareInfo & info)
  : name(info.name)
  {
    allow_motion = boolean(info.hardware_parameters, "enable_motion");
    calibrated = boolean(info.hardware_parameters, "calibration_confirmed");
    command_period = duration(info.hardware_parameters, "command_period_ms");
    feedback_timeout = duration(info.hardware_parameters, "feedback_timeout_ms");
    service_timeout = duration(info.hardware_parameters, "service_timeout_ms");
    command_timeout = duration(info.hardware_parameters, "command_timeout_ms");
    status_period = duration(info.hardware_parameters, "status_poll_ms");
    startup_timeout = duration(info.hardware_parameters, "startup_timeout_ms");
    if (feedback_timeout <= status_period * 2 || command_timeout <= command_period) {
      throw std::invalid_argument("Timeouts must exceed feedback polling / command periods");
    }
    if (info.joints.size() != joint_count) {
      throw std::invalid_argument("Expected two independent steering and two drive joints");
    }
    std::set<std::string> nodes;
    std::set<std::string> names;
    for (size_t index = 0; index < joint_count; ++index) {
      const auto & joint = info.joints[index];
      auto & motor = motors[index];
      motor.name = joint.name;
      motor.command_interface = index < 2 ? "position" : "velocity";
      if (!names.insert(joint.name).second || joint.command_interfaces.size() != 1 ||
        joint.command_interfaces.front().name != motor.command_interface)
      {
        throw std::invalid_argument("Joint order must be steering, steering, drive, drive");
      }
      std::set<std::string> states;
      for (const auto & state : joint.state_interfaces) {
        states.insert(state.name);
      }
      if (joint.state_interfaces.size() != 2 || states != std::set<std::string>{
          "position", "velocity"})
      {
        throw std::invalid_argument("Every joint must export position and velocity feedback");
      }
      const auto & parameters = joint.parameters;
      motor.node = required(parameters, "driver_node");
      if (motor.node.front() != '/' || motor.node.back() == '/' ||
        !nodes.insert(motor.node).second)
      {
        throw std::invalid_argument("driver_node must be an absolute, unique node path");
      }
      motor.position_scale = number(parameters, "position_scale");
      motor.velocity_scale = number(parameters, "velocity_scale");
      motor.offset = number(parameters, "position_offset");
      motor.direction = number(parameters, "direction");
      motor.lower = number(parameters, "min_position");
      motor.upper = number(parameters, "max_position");
      motor.max_velocity = number(parameters, "max_velocity");
      motor.max_acceleration = number(parameters, "max_acceleration");
      const auto mode = number(parameters, "operation_mode");
      if (motor.position_scale <= 0.0 || motor.velocity_scale <= 0.0 ||
        std::abs(motor.direction) != 1.0 || motor.lower >= motor.upper ||
        motor.max_velocity <= 0.0 || motor.max_acceleration <= 0.0 ||
        mode != (index < 2 ? 1.0 : 3.0))
      {
        throw std::invalid_argument("Invalid calibration, limits or PP/PV operation_mode");
      }
      motor.mode = static_cast<uint8_t>(mode);
    }
    const double unknown = std::numeric_limits<double>::quiet_NaN();
    positions.fill(unknown);
    velocities.fill(unknown);
    feedback.positions.fill(unknown);
    feedback.velocities.fill(unknown);
    commands.fill(0.0);
  }

  Feedback feedback_copy()
  {
    std::lock_guard<std::mutex> lock(feedback_mutex);
    return feedback;
  }

  bool feedback_ready(const Feedback & sample, Time now, bool require_enabled) const
  {
    for (size_t index = 0; index < joint_count; ++index) {
      if (!fresh(sample.position_stamp[index], now, feedback_timeout) ||
        !fresh(sample.velocity_stamp[index], now, feedback_timeout) ||
        !fresh(sample.status_stamp[index], now, feedback_timeout) ||
        !fresh(sample.mode_stamp[index], now, feedback_timeout) ||
        !std::isfinite(sample.positions[index]) || !std::isfinite(sample.velocities[index]))
      {
        return false;
      }
      if (require_enabled && ((sample.status_word[index] & 0x006f) != 0x0027 ||
        sample.mode[index] != motors[index].mode))
      {
        return false;
      }
    }
    return true;
  }

  void publish_state(const std::string & message)
  {
    std_msgs::msg::Bool armed_message;
    armed_message.data = armed.load();
    armed_pub->publish(armed_message);
    std_msgs::msg::String status_message;
    status_message.data = message;
    status_pub->publish(status_message);
  }

  void halt_all()
  {
    for (auto & motor : motors) {
      if (!motor.halt_sent && motor.halt->service_is_ready()) {
        motor.halt_sent = true;
        motor.halt_pending = true;
        motor.halt_stamp = Clock::now();
        try {
          const size_t index = static_cast<size_t>(&motor - motors.data());
          motor.halt->async_send_request(std::make_shared<Trigger::Request>(),
            [this, index](rclcpp::Client<Trigger>::SharedFuture) {
              motors[index].halt_pending = false;
            });
        } catch (const std::exception & error) {
          RCLCPP_ERROR(node->get_logger(), "Halt request failed: %s", error.what());
        }
      }
    }
  }

  void latch_fault(const std::string & reason)
  {
    armed.store(false);
    if (!fault.exchange(true)) {
      RCLCPP_ERROR(node->get_logger(), "%s; restart bringup after correcting the fault", reason.c_str());
      publish_state("FAULT: " + reason);
    }
    halt_all();
  }

  void arm(Trigger::Response & response)
  {
    if (!allow_motion || !calibrated) {
      response.message = "Motion denied: enable_motion and calibration_confirmed must both be true";
      return;
    }
    if (fault.load()) {
      response.message = "Fault latched; restart bringup after correcting the fault";
      return;
    }
    if (armed.load()) {
      response.success = true;
      response.message = "Already armed";
      return;
    }
    const auto now = Clock::now();
    const auto sample = feedback_copy();
    if (!feedback_ready(sample, now, true)) {
      response.message = "Require fresh feedback, Operation Enabled and PP/PV modes on all motors";
      return;
    }
    std::lock_guard<std::mutex> lock(command_mutex);
    if (!fresh(command_stamp, now, command_timeout)) {
      response.message = "Controller commands are missing or stale";
      return;
    }
    for (size_t index = 0; index < joint_count; ++index) {
      auto & motor = motors[index];
      if (!motor.target->service_is_ready() || !motor.halt->service_is_ready() ||
        motor.target_pending || motor.halt_pending ||
        (index < 2 && (std::abs(latest_commands[index] - sample.positions[index]) > 0.05 ||
        sample.positions[index] < motor.lower || sample.positions[index] > motor.upper)) ||
        (index >= 2 && (std::abs(latest_commands[index]) > 0.001 ||
        std::abs(sample.velocities[index]) > 0.01)))
      {
        response.message = "Arming requires ready services, stationary drives and matching steering targets";
        return;
      }
    }
    for (size_t index = 0; index < joint_count; ++index) {
      motors[index].last_sent = index < 2 ? sample.positions[index] : 0.0;
      motors[index].halt_sent = false;
    }
    last_batch = now;
    armed.store(true);
    response.success = true;
    response.message = "Armed; hardware emergency stop and drive watchdog remain mandatory";
    publish_state("ARMED");
  }

  void poll_status(size_t index, Time now)
  {
    auto & motor = motors[index];
    if (motor.read_pending || now < motor.next_poll || !motor.sdo_read->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<Read::Request>();
    request->index = motor.reading_mode ? 0x6061 : 0x6041;
    request->subindex = 0;
    request->type = motor.reading_mode ? 8 : 16;
    motor.read_pending = true;
    motor.read_sent = now;
    motor.next_poll = now + status_period;
    motor.sdo_read->async_send_request(request,
      [this, index](rclcpp::Client<Read>::SharedFuture future) {
        auto & current = motors[index];
        current.read_pending = false;
        const auto response = future.get();
        if (!response->success) {
          if (armed.load()) {
            latch_fault("SDO liveness/status read failed for " + current.name);
          }
          return;
        }
        {
          std::lock_guard<std::mutex> lock(feedback_mutex);
          if (current.reading_mode) {
            feedback.mode[index] = static_cast<uint8_t>(response->data);
            feedback.mode_stamp[index] = Clock::now();
          } else {
            feedback.status_word[index] = static_cast<uint16_t>(response->data);
            feedback.status_stamp[index] = Clock::now();
          }
        }
        current.reading_mode = !current.reading_mode;
      });
  }

  void tick()
  {
    try {
      tick_checked();
    } catch (const std::exception & error) {
      latch_fault(std::string("CANopen request error: ") + error.what());
    }
  }

  void tick_checked()
  {
    const auto now = Clock::now();
    if (halt_requested.exchange(false)) {
      armed.store(false);
      halt_all();
    }
    if (fault_requested.load()) {
      latch_fault("Invalid joint command or stale realtime command stream");
    }
    for (const auto & motor : motors) {
      if ((motor.target_pending && now - motor.target_sent > service_timeout) ||
        (motor.read_pending && now - motor.read_sent > service_timeout) ||
        (motor.halt_pending && now - motor.halt_stamp > service_timeout))
      {
        latch_fault("Service timeout for " + motor.name);
      }
    }
    if (fault.load()) {
      halt_all();
      return;
    }
    for (size_t index = 0; index < joint_count; ++index) {
      poll_status(index, now);
    }
    if (!armed.load()) {
      return;
    }
    if (!feedback_ready(feedback_copy(), now, true)) {
      latch_fault("Feedback/status expired or a drive left its requested operating mode");
      return;
    }
    std::array<double, joint_count> targets;
    {
      std::lock_guard<std::mutex> lock(command_mutex);
      if (!fresh(command_stamp, now, command_timeout)) {
        latch_fault("Controller command stream timed out");
        return;
      }
      targets = latest_commands;
    }
    for (const auto & motor : motors) {
      if (motor.target_pending) {
        return;
      }
    }
    const double elapsed = std::min(0.1, std::chrono::duration<double>(now - last_batch).count());
    std::array<double, joint_count> native_targets;
    for (size_t index = 0; index < joint_count; ++index) {
      auto & motor = motors[index];
      const double rate = index < 2 ? motor.max_velocity : motor.max_acceleration;
      targets[index] = std::max(motor.last_sent - rate * elapsed,
          std::min(motor.last_sent + rate * elapsed, targets[index]));
      native_targets[index] = index < 2 ?
        motor.offset + motor.direction * motor.position_scale * targets[index] :
        motor.direction * motor.velocity_scale * targets[index];
      if (!std::isfinite(native_targets[index]) ||
        native_targets[index] < std::numeric_limits<int32_t>::min() ||
        native_targets[index] > std::numeric_limits<int32_t>::max())
      {
        latch_fault("Native target exceeds signed 32-bit drive range");
        return;
      }
    }
    last_batch = now;
    for (size_t index = 0; index < joint_count; ++index) {
      auto & motor = motors[index];
      if (!motor.target->service_is_ready()) {
        latch_fault("Target service disappeared for " + motor.name);
        return;
      }
      auto request = std::make_shared<Target::Request>();
      request->target = native_targets[index];
      motor.last_sent = targets[index];
      motor.target_pending = true;
      motor.target_sent = now;
      motor.target->async_send_request(request,
        [this, index](rclcpp::Client<Target>::SharedFuture future) {
          motors[index].target_pending = false;
          if (!future.get()->success) {
            latch_fault("Drive rejected target for " + motors[index].name);
          }
        });
    }
  }

  void setup_node()
  {
    node = std::make_shared<rclcpp::Node>(
      "ackermann_canopen_hardware", rclcpp::NodeOptions().use_global_arguments(false));
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    const auto latched = rclcpp::QoS(1).transient_local();
    armed_pub = node->create_publisher<std_msgs::msg::Bool>("~/armed", latched);
    status_pub = node->create_publisher<std_msgs::msg::String>("~/status", latched);
    arm_service = node->create_service<Trigger>("~/arm",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {arm(*response);});
    disarm_service = node->create_service<Trigger>("~/disarm",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        armed.store(false);
        halt_all();
        response->success = true;
        response->message = "Halt requested; confirm physical stop independently";
        publish_state(fault.load() ? "FAULT (disarmed)" : "DISARMED");
      });
    for (size_t index = 0; index < joint_count; ++index) {
      auto & motor = motors[index];
      motor.target = node->create_client<Target>(motor.node + "/target");
      motor.sdo_read = node->create_client<Read>(motor.node + "/sdo_read");
      motor.halt = node->create_client<Trigger>(motor.node + "/halt");
      motor.position_sub = node->create_subscription<std_msgs::msg::Float64>(
        motor.node + "/actual_position", 1,
        [this, index](std_msgs::msg::Float64::SharedPtr message) {
          const auto & mapping = motors[index];
          const double position = (message->data - mapping.offset) /
            (mapping.direction * mapping.position_scale);
          std::lock_guard<std::mutex> lock(feedback_mutex);
          feedback.positions[index] = position;
          feedback.position_stamp[index] = std::isfinite(position) ? Clock::now() : Time{};
        });
      motor.velocity_sub = node->create_subscription<std_msgs::msg::Float64>(
        motor.node + "/actual_speed", 1,
        [this, index](std_msgs::msg::Float64::SharedPtr message) {
          const auto & mapping = motors[index];
          const double velocity = message->data / (mapping.direction * mapping.velocity_scale);
          std::lock_guard<std::mutex> lock(feedback_mutex);
          feedback.velocities[index] = velocity;
          feedback.velocity_stamp[index] = std::isfinite(velocity) ? Clock::now() : Time{};
        });
    }
    timer = node->create_wall_timer(command_period, [this]() {tick();});
    publish_state("DISARMED");
  }

  void shutdown()
  {
    if (worker.joinable()) {
      armed.store(false);
      halt_requested.store(true);
      std::this_thread::sleep_for(command_period * 2);
      executor->cancel();
      worker.join();
      executor->remove_node(node);
    }
    hardware_status.store(hardware_interface::status::STOPPED);
  }

  std::string name;
  std::array<Motor, joint_count> motors;
  std::array<double, joint_count> positions;
  std::array<double, joint_count> velocities;
  std::array<double, joint_count> commands;
  std::array<double, joint_count> latest_commands{};
  Feedback feedback;
  std::mutex feedback_mutex;
  std::mutex command_mutex;
  Time command_stamp;
  Time last_batch;
  bool allow_motion;
  bool calibrated;
  std::chrono::milliseconds command_period{20};
  std::chrono::milliseconds feedback_timeout{500};
  std::chrono::milliseconds service_timeout{500};
  std::chrono::milliseconds command_timeout{250};
  std::chrono::milliseconds status_period{100};
  std::chrono::milliseconds startup_timeout{15000};
  std::atomic<bool> armed{false};
  std::atomic<bool> fault{false};
  std::atomic<bool> fault_requested{false};
  std::atomic<bool> halt_requested{false};
  std::atomic<hardware_interface::status> hardware_status{hardware_interface::status::CONFIGURED};
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Service<Trigger>::SharedPtr arm_service;
  rclcpp::Service<Trigger>::SharedPtr disarm_service;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub;
  std::thread worker;
};

AckermannCanopenSystem::AckermannCanopenSystem() = default;

AckermannCanopenSystem::~AckermannCanopenSystem()
{
  stop();
}

hardware_interface::return_type AckermannCanopenSystem::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (impl_) {
    impl_->shutdown();
  }
  try {
    impl_.reset(new Impl(info));
    return hardware_interface::return_type::OK;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("ackermann_canopen_hardware"), "%s", error.what());
    impl_.reset();
    return hardware_interface::return_type::ERROR;
  }
}

std::vector<hardware_interface::StateInterface> AckermannCanopenSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  if (impl_) {
    for (size_t index = 0; index < joint_count; ++index) {
      interfaces.emplace_back(impl_->motors[index].name, "position", &impl_->positions[index]);
      interfaces.emplace_back(impl_->motors[index].name, "velocity", &impl_->velocities[index]);
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> AckermannCanopenSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  if (impl_) {
    for (size_t index = 0; index < joint_count; ++index) {
      interfaces.emplace_back(impl_->motors[index].name, impl_->motors[index].command_interface,
        &impl_->commands[index]);
    }
  }
  return interfaces;
}

hardware_interface::return_type AckermannCanopenSystem::start()
{
  if (!impl_ || impl_->worker.joinable() || impl_->fault.load() ||
    impl_->hardware_status.load() == hardware_interface::status::STOPPED)
  {
    return hardware_interface::return_type::ERROR;
  }
  impl_->setup_node();
  impl_->worker = std::thread([this]() {impl_->executor->spin();});
  const auto deadline = Clock::now() + impl_->startup_timeout;
  while (rclcpp::ok() && Clock::now() < deadline && !impl_->fault.load()) {
    if (impl_->feedback_ready(impl_->feedback_copy(), Clock::now(), false)) {
      read();
      for (size_t index = 0; index < joint_count; ++index) {
        impl_->commands[index] = index < 2 ? impl_->positions[index] : 0.0;
      }
      impl_->hardware_status.store(hardware_interface::status::STARTED);
      return hardware_interface::return_type::OK;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_ERROR(impl_->node->get_logger(), "No fresh feedback/status from all four drivers before startup timeout");
  impl_->shutdown();
  return hardware_interface::return_type::ERROR;
}

hardware_interface::return_type AckermannCanopenSystem::stop()
{
  if (impl_) {
    impl_->shutdown();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AckermannCanopenSystem::read()
{
  if (!impl_) {
    return hardware_interface::return_type::ERROR;
  }
  std::unique_lock<std::mutex> lock(impl_->feedback_mutex, std::try_to_lock);
  if (lock.owns_lock()) {
    impl_->positions = impl_->feedback.positions;
    impl_->velocities = impl_->feedback.velocities;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AckermannCanopenSystem::write()
{
  if (!impl_) {
    return hardware_interface::return_type::ERROR;
  }
  for (size_t index = 0; index < joint_count; ++index) {
    const auto & motor = impl_->motors[index];
    const double command = impl_->commands[index];
    if (!std::isfinite(command) ||
      (index < 2 && (command < motor.lower || command > motor.upper)) ||
      (index >= 2 && std::abs(command) > motor.max_velocity))
    {
      if (impl_->armed.load()) {
        impl_->fault_requested.store(true);
      }
      return hardware_interface::return_type::ERROR;
    }
  }
  std::unique_lock<std::mutex> lock(impl_->command_mutex, std::try_to_lock);
  if (lock.owns_lock()) {
    impl_->latest_commands = impl_->commands;
    impl_->command_stamp = Clock::now();
  }
  return hardware_interface::return_type::OK;
}

std::string AckermannCanopenSystem::get_name() const
{
  return impl_ ? impl_->name : std::string();
}

hardware_interface::status AckermannCanopenSystem::get_status() const
{
  return impl_ ? impl_->hardware_status.load() : hardware_interface::status::UNKNOWN;
}
}

PLUGINLIB_EXPORT_CLASS(
  ackermann_canopen_hardware::AckermannCanopenSystem, hardware_interface::SystemInterface)
