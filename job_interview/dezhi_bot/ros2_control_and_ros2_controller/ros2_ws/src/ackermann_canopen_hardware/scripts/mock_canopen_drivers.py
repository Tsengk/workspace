#!/usr/bin/env python3
import math
import time

import rclpy
import yaml
from canopen_interfaces.srv import CORead, COTargetDouble
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


class MockCanopenDrivers(Node):
    def __init__(self):
        super().__init__("mock_canopen_drivers")
        self.declare_parameter("motor_config", "")
        with open(self.get_parameter("motor_config").value, encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        self.motors = []
        self.service_handles = []
        for mapping in config["joints"].values():
            motor = dict(mapping)
            motor.update(position=mapping["position_offset"], velocity=0.0, status=0x0027)
            prefix = motor["driver_node"]
            motor["position_pub"] = self.create_publisher(Float64, prefix + "/actual_position", 1)
            motor["velocity_pub"] = self.create_publisher(Float64, prefix + "/actual_speed", 1)
            self.service_handles.append(self.create_service(
                COTargetDouble, prefix + "/target", self.target_callback(motor)))
            self.service_handles.append(self.create_service(
                CORead, prefix + "/sdo_read", self.read_callback(motor)))
            for action in ("init", "recover", "halt", "position_mode", "velocity_mode"):
                self.service_handles.append(self.create_service(
                    Trigger, prefix + "/" + action, self.trigger_callback(motor, action)))
            self.motors.append(motor)
        self.previous = time.monotonic()
        self.timer = self.create_timer(0.02, self.publish_feedback)
        self.get_logger().warning("MOCK CANopen services only: no CAN adapter or real motor is accessed")

    @staticmethod
    def target_callback(motor):
        def callback(request, response):
            response.success = (
                motor["status"] == 0x0027
                and math.isfinite(request.target)
                and -(2 ** 31) <= request.target <= 2 ** 31 - 1
                and motor["operation_mode"] in (1, 3)
            )
            if response.success:
                if motor["operation_mode"] == 1:
                    motor["position"] = float(int(request.target))
                else:
                    motor["velocity"] = float(int(request.target))
            return response
        return callback

    @staticmethod
    def read_callback(motor):
        def callback(request, response):
            values = {
                0x6041: motor["status"],
                0x6061: motor["operation_mode"],
                0x6064: int(motor["position"]) & 0xFFFFFFFF,
                0x606C: int(motor["velocity"]) & 0xFFFFFFFF,
            }
            response.success = request.subindex == 0 and request.index in values
            if response.success:
                response.data = values[request.index]
            return response
        return callback

    @staticmethod
    def trigger_callback(motor, action):
        def callback(request, response):
            if action in ("init", "recover"):
                motor["status"] = 0x0027
            elif action == "halt":
                motor["status"] = 0x0007
                motor["velocity"] = 0.0
            elif action == "position_mode":
                motor["operation_mode"] = 1
            elif action == "velocity_mode":
                motor["operation_mode"] = 3
            response.success = True
            response.message = "Mock action: " + action
            return response
        return callback

    def publish_feedback(self):
        now = time.monotonic()
        elapsed = min(now - self.previous, 0.1)
        self.previous = now
        for motor in self.motors:
            if motor["operation_mode"] == 3:
                motor["position"] += (
                    motor["velocity"] * motor["position_scale"]
                    / motor["velocity_scale"] * elapsed
                )
            motor["position_pub"].publish(Float64(data=motor["position"]))
            motor["velocity_pub"].publish(Float64(data=motor["velocity"]))


def main():
    rclpy.init()
    node = MockCanopenDrivers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
