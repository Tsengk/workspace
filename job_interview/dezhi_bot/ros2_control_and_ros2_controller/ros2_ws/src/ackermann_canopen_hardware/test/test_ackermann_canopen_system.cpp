#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <thread>

#include "ackermann_canopen_hardware/ackermann_canopen_system.hpp"
#include "canopen_interfaces/srv/co_read.hpp"
#include "canopen_interfaces/srv/co_target_double.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{
using System = ackermann_canopen_hardware::AckermannCanopenSystem;
using Read = canopen_interfaces::srv::CORead;
using Target = canopen_interfaces::srv::COTargetDouble;
using Trigger = std_srvs::srv::Trigger;
using Return = hardware_interface::return_type;

class HardwareTest : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    int argument_count = 0;
    rclcpp::init(argument_count, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    static size_t instance = 0;
    const auto prefix = "/test_motor_" + std::to_string(++instance) + "_";
    server = std::make_shared<rclcpp::Node>("fake_motor_server");
    client_node = std::make_shared<rclcpp::Node>("hardware_test_client");
    client_executor.add_node(client_node);
    info.name = "test_canopen_system";
    info.hardware_parameters = {
      {"enable_motion", "true"}, {"calibration_confirmed", "true"},
      {"command_period_ms", "20"}, {"status_poll_ms", "40"},
      {"feedback_timeout_ms", "400"}, {"service_timeout_ms", "300"},
      {"command_timeout_ms", "200"}, {"startup_timeout_ms", "3000"}};
    for (size_t index = 0; index < 4; ++index) {
      hardware_interface::ComponentInfo joint;
      joint.name = "joint_" + std::to_string(index);
      hardware_interface::InterfaceInfo command;
      command.name = index < 2 ? "position" : "velocity";
      joint.command_interfaces.push_back(command);
      for (const auto & name : {"position", "velocity"}) {
        hardware_interface::InterfaceInfo state;
        state.name = name;
        joint.state_interfaces.push_back(state);
      }
      const auto node = prefix + std::to_string(index);
      joint.parameters = {
        {"driver_node", node}, {"position_scale", "1000"}, {"velocity_scale", "100"},
        {"position_offset", "100"}, {"direction", index % 2 ? "-1" : "1"},
        {"min_position", "-0.6"}, {"max_position", "0.6"},
        {"max_velocity", "10"}, {"max_acceleration", "1000"},
        {"operation_mode", index < 2 ? "1" : "3"}};
      info.joints.push_back(joint);
      raw_positions[index] = index < 2 ? (index % 2 ? 0.0 : 200.0) : 100.0;
      modes[index] = index < 2 ? 1 : 3;
      position_pubs[index] = server->create_publisher<std_msgs::msg::Float64>(
        node + "/actual_position", 1);
      velocity_pubs[index] = server->create_publisher<std_msgs::msg::Float64>(
        node + "/actual_speed", 1);
      target_services[index] = server->create_service<Target>(node + "/target",
        [this, index](Target::Request::SharedPtr request, Target::Response::SharedPtr response) {
          if (delay_targets.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(400));
          }
          std::lock_guard<std::mutex> lock(server_mutex);
          last_targets[index] = request->target;
          ++target_count;
          if (index < 2) {
            raw_positions[index] = request->target;
          } else {
            raw_velocities[index] = request->target;
          }
          response->success = !reject_targets.load();
        });
      read_services[index] = server->create_service<Read>(node + "/sdo_read",
        [this, index](Read::Request::SharedPtr request, Read::Response::SharedPtr response) {
          std::lock_guard<std::mutex> lock(server_mutex);
          response->success = !fail_sdo.load();
          response->data = request->index == 0x6041 ? 0x0027 : modes[index];
        });
      halt_services[index] = server->create_service<Trigger>(node + "/halt",
        [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
          ++halt_count;
          response->success = false;
        });
    }
    publisher_timer = server->create_wall_timer(std::chrono::milliseconds(10), [this]() {
      if (!publish_feedback.load()) {
        return;
      }
      std::lock_guard<std::mutex> lock(server_mutex);
      for (size_t index = 0; index < 4; ++index) {
        std_msgs::msg::Float64 position;
        position.data = raw_positions[index];
        std_msgs::msg::Float64 velocity;
        velocity.data = raw_velocities[index];
        position_pubs[index]->publish(position);
        velocity_pubs[index]->publish(velocity);
      }
    });
    server_executor.add_node(server);
    server_worker = std::thread([this]() {server_executor.spin();});
  }

  void TearDown() override
  {
    hardware.stop();
    server_executor.cancel();
    server_worker.join();
    server_executor.remove_node(server);
    client_executor.remove_node(client_node);
  }

  void start_hardware()
  {
    ASSERT_EQ(hardware.configure(info), Return::OK);
    ASSERT_EQ(hardware.start(), Return::OK);
    states = hardware.export_state_interfaces();
    commands = hardware.export_command_interfaces();
    ASSERT_EQ(states.size(), 8u);
    ASSERT_EQ(commands.size(), 4u);
    ASSERT_EQ(hardware.write(), Return::OK);
  }

  bool wait_until(const std::function<bool()> & predicate, bool pump_commands = true,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(2000))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      hardware.read();
      if (pump_commands) {
        hardware.write();
      }
      client_executor.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
  }

  Trigger::Response::SharedPtr trigger(const std::string & action)
  {
    const auto client = client_node->create_client<Trigger>(
      "/ackermann_canopen_hardware/" + action);
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      ADD_FAILURE() << "Bridge service did not appear";
      return std::make_shared<Trigger::Response>();
    }
    hardware.write();
    auto future = client->async_send_request(std::make_shared<Trigger::Request>());
    if (!wait_until([&future]() {
        return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
      }))
    {
      ADD_FAILURE() << "Bridge service did not respond";
      return std::make_shared<Trigger::Response>();
    }
    return future.get();
  }

  System hardware;
  hardware_interface::HardwareInfo info;
  std::vector<hardware_interface::StateInterface> states;
  std::vector<hardware_interface::CommandInterface> commands;
  rclcpp::Node::SharedPtr server;
  rclcpp::Node::SharedPtr client_node;
  rclcpp::executors::SingleThreadedExecutor server_executor;
  rclcpp::executors::SingleThreadedExecutor client_executor;
  std::thread server_worker;
  rclcpp::TimerBase::SharedPtr publisher_timer;
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> position_pubs;
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> velocity_pubs;
  std::array<rclcpp::Service<Target>::SharedPtr, 4> target_services;
  std::array<rclcpp::Service<Read>::SharedPtr, 4> read_services;
  std::array<rclcpp::Service<Trigger>::SharedPtr, 4> halt_services;
  std::array<double, 4> raw_positions{};
  std::array<double, 4> raw_velocities{};
  std::array<double, 4> last_targets{};
  std::array<uint8_t, 4> modes{};
  std::mutex server_mutex;
  std::atomic<size_t> target_count{0};
  std::atomic<size_t> halt_count{0};
  std::atomic<bool> fail_sdo{false};
  std::atomic<bool> reject_targets{false};
  std::atomic<bool> publish_feedback{true};
  std::atomic<bool> delay_targets{false};
};

TEST_F(HardwareTest, rejects_invalid_topology_and_calibration_numbers)
{
  auto invalid = info;
  invalid.joints.pop_back();
  EXPECT_EQ(hardware.configure(invalid), Return::ERROR);
  invalid = info;
  invalid.joints[0].parameters["position_scale"] = "0";
  EXPECT_EQ(hardware.configure(invalid), Return::ERROR);
  invalid = info;
  invalid.joints[1].parameters["driver_node"] = invalid.joints[0].parameters["driver_node"];
  EXPECT_EQ(hardware.configure(invalid), Return::ERROR);
}

TEST_F(HardwareTest, motion_disabled_never_sends_targets)
{
  info.hardware_parameters["enable_motion"] = "false";
  start_hardware();
  EXPECT_FALSE(trigger("arm")->success);
  EXPECT_EQ(target_count.load(), 0u);
}

TEST_F(HardwareTest, uncalibrated_mapping_cannot_arm)
{
  info.hardware_parameters["calibration_confirmed"] = "false";
  start_hardware();
  EXPECT_FALSE(trigger("arm")->success);
  EXPECT_EQ(target_count.load(), 0u);
}

TEST_F(HardwareTest, requires_correct_mode)
{
  modes[2] = 1;
  start_hardware();
  EXPECT_FALSE(trigger("arm")->success);
  EXPECT_EQ(target_count.load(), 0u);
}

TEST_F(HardwareTest, converts_units_and_disarms_all_motors)
{
  start_hardware();
  EXPECT_NEAR(states[0].get_value(), 0.1, 1e-9);
  EXPECT_NEAR(states[2].get_value(), 0.1, 1e-9);
  ASSERT_TRUE(trigger("arm")->success);
  commands[0].set_value(0.2);
  commands[1].set_value(0.2);
  commands[2].set_value(0.5);
  commands[3].set_value(0.5);
  ASSERT_TRUE(wait_until([this]() {
    std::lock_guard<std::mutex> lock(server_mutex);
    return std::abs(last_targets[0] - 300.0) < 1e-6 &&
           std::abs(last_targets[1] + 100.0) < 1e-6 &&
           std::abs(last_targets[2] - 50.0) < 1e-6 &&
           std::abs(last_targets[3] + 50.0) < 1e-6;
  }));
  ASSERT_TRUE(wait_until([this]() {return std::abs(states[7].get_value() - 0.5) < 1e-6;}));
  EXPECT_TRUE(trigger("disarm")->success);
  EXPECT_TRUE(wait_until([this]() {return halt_count.load() == 4;}));
}

TEST_F(HardwareTest, sdo_liveness_failure_latches_fault)
{
  start_hardware();
  ASSERT_TRUE(trigger("arm")->success);
  fail_sdo.store(true);
  ASSERT_TRUE(wait_until([this]() {return halt_count.load() == 4;}));
  EXPECT_FALSE(trigger("arm")->success);
}

TEST_F(HardwareTest, invalid_command_latches_fault)
{
  start_hardware();
  ASSERT_TRUE(trigger("arm")->success);
  commands[2].set_value(std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(hardware.write(), Return::ERROR);
  EXPECT_TRUE(wait_until([this]() {return halt_count.load() == 4;}, false));
  EXPECT_FALSE(trigger("arm")->success);
}

TEST_F(HardwareTest, missing_control_loop_halts_even_while_feedback_is_alive)
{
  start_hardware();
  ASSERT_TRUE(trigger("arm")->success);
  EXPECT_TRUE(wait_until([this]() {return halt_count.load() == 4;}, false));
  EXPECT_FALSE(trigger("arm")->success);
}

TEST_F(HardwareTest, stale_feedback_halts_even_while_sdo_is_alive)
{
  start_hardware();
  ASSERT_TRUE(trigger("arm")->success);
  publish_feedback.store(false);
  EXPECT_TRUE(wait_until([this]() {return halt_count.load() == 4;}));
  EXPECT_FALSE(trigger("arm")->success);
}

TEST_F(HardwareTest, rejected_target_latches_fault)
{
  start_hardware();
  ASSERT_TRUE(trigger("arm")->success);
  reject_targets.store(true);
  EXPECT_TRUE(wait_until([this]() {return halt_count.load() == 4;}));
  EXPECT_FALSE(trigger("arm")->success);
}

TEST_F(HardwareTest, service_timeout_does_not_accumulate_target_requests)
{
  start_hardware();
  delay_targets.store(true);
  ASSERT_TRUE(trigger("arm")->success);
  EXPECT_TRUE(wait_until([this]() {return halt_count.load() == 4;}, true,
    std::chrono::milliseconds(3000)));
  EXPECT_LE(target_count.load(), 4u);
  EXPECT_FALSE(trigger("arm")->success);
}
}
