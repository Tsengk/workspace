import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
import rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class MockBringupTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="ackermann-mock-test-")
        self.output_path = Path(self.temporary.name) / "launch.log"
        self.output = self.output_path.open("w", encoding="utf-8")
        os.environ["ROS_DOMAIN_ID"] = str(40 + os.getpid() % 140)
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        environment = dict(os.environ)
        environment["ROS_LOG_DIR"] = self.temporary.name
        self.process = subprocess.Popen(
            ["ros2", "launch", "my_ackermann_bot", "ackermann.launch.py",
             "mode:=mock", "enable_motion:=true"],
            stdout=self.output, stderr=subprocess.STDOUT,
            env=environment, start_new_session=True)
        rclpy.init()
        self.node = rclpy.create_node("mock_bringup_test")
        self.joints = {}
        self.subscription = self.node.create_subscription(
            JointState, "/joint_states", self.feedback, 10)
        self.publisher = self.node.create_publisher(
            Twist, "/ackermann_controller/cmd_vel", 10)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=5)
        self.output.close()
        print(self.output_path.read_text(encoding="utf-8"))
        self.temporary.cleanup()

    def feedback(self, message):
        for index, name in enumerate(message.name):
            self.joints[name] = (message.position[index], message.velocity[index])

    def wait_for(self, predicate, timeout=10.0, command=None):
        deadline = time.monotonic() + timeout
        next_publish = 0.0
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                self.fail("Mock launch exited: " + self.output_path.read_text(encoding="utf-8"))
            if command is not None and time.monotonic() >= next_publish:
                self.publisher.publish(command)
                next_publish = time.monotonic() + 0.05
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if predicate():
                return
        self.fail("Timed out waiting for mock bringup condition")

    def call(self, name, service_type):
        client = self.node.create_client(service_type, name)
        self.assertTrue(client.wait_for_service(timeout_sec=20.0), name)
        future = client.call_async(service_type.Request())
        self.wait_for(future.done)
        response = future.result()
        self.node.destroy_client(client)
        return response

    def test_end_to_end_motion_limits_timeout_and_disarm(self):
        deadline = time.monotonic() + 20.0
        while True:
            response = self.call("/controller_manager/list_controllers", ListControllers)
            states = {controller.name: controller.state for controller in response.controller}
            if states.get("ackermann_controller") == "active" and states.get(
                    "joint_state_broadcaster") == "active":
                break
            self.assertLess(time.monotonic(), deadline, states)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.wait_for(lambda: len(self.joints) >= 4)
        response = self.call("/ackermann_canopen_hardware/arm", Trigger)
        self.assertTrue(response.success, response.message)
        turn = Twist()
        turn.linear.x = 0.1
        turn.angular.z = 10.0
        self.wait_for(
            lambda: self.joints.get("rear_left_wheel_joint", (0.0, 0.0))[1] > 1.0
            and self.joints.get("front_left_steer_joint", (0.0, 0.0))[0] > 0.59,
            timeout=6.0, command=turn)
        for name in ("front_left_steer_joint", "front_right_steer_joint"):
            self.assertLessEqual(abs(self.joints[name][0]), 0.6001)
        self.wait_for(lambda: all(
            abs(self.joints[name][1]) < 0.001
            for name in ("rear_left_wheel_joint", "rear_right_wheel_joint")), timeout=4.0)
        self.assertGreater(self.joints["front_left_steer_joint"][0], 0.59)
        reverse = Twist()
        reverse.linear.x = -0.1
        reverse.angular.z = 0.2
        self.wait_for(
            lambda: all(self.joints[name][1] < -1.0 for name in (
                "rear_left_wheel_joint", "rear_right_wheel_joint"))
            and self.joints["front_right_steer_joint"][0] < -0.59,
            timeout=6.0, command=reverse)
        for name in ("front_left_steer_joint", "front_right_steer_joint"):
            self.assertLessEqual(abs(self.joints[name][0]), 0.6001)
        straight = Twist()
        straight.linear.x = 0.1
        self.wait_for(
            lambda: all(abs(self.joints[name][0]) < 0.001 for name in (
                "front_left_steer_joint", "front_right_steer_joint"))
            and all(self.joints[name][1] > 1.0 for name in (
                "rear_left_wheel_joint", "rear_right_wheel_joint")),
            timeout=6.0, command=straight)
        self.wait_for(lambda: all(
            abs(self.joints[name][1]) < 0.001
            for name in ("rear_left_wheel_joint", "rear_right_wheel_joint")), timeout=4.0)
        response = self.call("/ackermann_canopen_hardware/disarm", Trigger)
        self.assertTrue(response.success, response.message)


if __name__ == "__main__":
    unittest.main()
