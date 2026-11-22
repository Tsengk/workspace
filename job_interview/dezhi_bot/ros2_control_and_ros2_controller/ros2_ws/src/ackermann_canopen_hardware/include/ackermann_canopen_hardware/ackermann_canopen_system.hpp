#ifndef ACKERMANN_CANOPEN_HARDWARE__ACKERMANN_CANOPEN_SYSTEM_HPP_
#define ACKERMANN_CANOPEN_HARDWARE__ACKERMANN_CANOPEN_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"

namespace ackermann_canopen_hardware
{
class AckermannCanopenSystem : public hardware_interface::SystemInterface
{
public:
  AckermannCanopenSystem();
  ~AckermannCanopenSystem() override;
  hardware_interface::return_type configure(
    const hardware_interface::HardwareInfo & info) override;//接收URDF中 <ros2_control> 解析出来的硬件配置
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type start() override;
  hardware_interface::return_type stop() override;
  hardware_interface::return_type read() override;
  hardware_interface::return_type write() override;//读取最新的command_interfaces_数组
  std::string get_name() const override;
  hardware_interface::status get_status() const override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}

#endif
