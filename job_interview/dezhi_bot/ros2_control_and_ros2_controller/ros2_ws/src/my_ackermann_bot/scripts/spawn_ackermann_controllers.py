#!/usr/bin/env python3
from pathlib import Path
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
from controller_manager_msgs.srv import ListControllers
import rclpy


def main():
    rclpy.init()
    node = rclpy.create_node("ackermann_controller_startup")
    node.declare_parameter("start_controller", False)
    node.declare_parameter("timeout", 60.0)
    activate = node.get_parameter("start_controller").value
    timeout = node.get_parameter("timeout").value
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    try:
        deadline = time.monotonic() + timeout
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("Controller manager services did not become available")
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(
            node, future, timeout_sec=max(0.0, deadline - time.monotonic()))
        if not future.done() or future.result() is None:
            raise RuntimeError("Controller manager did not respond before startup timeout")
    except (RuntimeError, KeyboardInterrupt) as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    spawner = Path(get_package_prefix("controller_manager")) / "lib/controller_manager/spawner.py"
    try:
        for controller in ("joint_state_broadcaster", "ackermann_controller"):
            arguments = [sys.executable, str(spawner), controller,
                         "--controller-manager", "/controller_manager",
                         "--controller-manager-timeout", str(int(timeout))]
            if controller == "ackermann_controller" and not activate:
                arguments.append("--stopped")
            subprocess.run(arguments, check=True)
    except (subprocess.CalledProcessError, KeyboardInterrupt):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
