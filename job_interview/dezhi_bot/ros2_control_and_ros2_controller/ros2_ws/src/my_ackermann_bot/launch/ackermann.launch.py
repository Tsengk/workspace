import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context):
    share = get_package_share_directory("my_ackermann_bot")
    mode = LaunchConfiguration("mode").perform(context)
    if mode == "gazebo":
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "gazebo_ackermann.launch.py")))]
    if mode not in ("mock", "canopen"):
        raise RuntimeError("mode must be gazebo, mock, or canopen")
    enable_motion = LaunchConfiguration("enable_motion").perform(context).lower()
    if enable_motion not in ("true", "false"):
        raise RuntimeError("enable_motion must be true or false")
    start_controller = LaunchConfiguration("start_controller").perform(context).lower()
    if start_controller not in ("auto", "true", "false"):
        raise RuntimeError("start_controller must be auto, true or false")
    activate_controller = mode == "mock" if start_controller == "auto" else start_controller == "true"
    motor_config = LaunchConfiguration("motor_config").perform(context)
    if not motor_config:
        motor_config = os.path.join(share, "config", (
            "mock_canopen_motors.yaml" if mode == "mock" else "canopen_motors.yaml"))
    controllers = LaunchConfiguration("controllers_file").perform(context)
    if not controllers:
        controllers = os.path.join(share, "config", "ackermann_controllers_real.yaml")
    for filename in (motor_config, controllers):
        if not os.path.isfile(filename):
            raise RuntimeError("Configuration file does not exist: " + filename)
    description = xacro.process_file(
        os.path.join(share, "urdf", "ackermann_car.urdf.xacro"),
        mappings={"hardware_mode": "canopen", "motor_config": motor_config,
                  "enable_motion": enable_motion}).toxml()
    robot_description = {"robot_description": description, "use_sim_time": False}
    actions = []
    if mode == "mock":
        actions.append(Node(
            package="ackermann_canopen_hardware", executable="mock_canopen_drivers.py",
            parameters=[{"motor_config": motor_config}], output="screen"))
    else:
        bus_config = LaunchConfiguration("bus_config").perform(context)
        master_dcf = LaunchConfiguration("master_dcf").perform(context)
        master_bin = LaunchConfiguration("master_bin").perform(context)
        for filename in (bus_config, master_dcf):
            if not filename or not os.path.isfile(filename):
                raise RuntimeError("canopen mode requires prepared bus_config and master_dcf files")
        if master_bin and not os.path.isfile(master_bin):
            raise RuntimeError("master_bin does not exist: " + master_bin)
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("canopen_core"), "launch", "canopen.launch.py")),
            launch_arguments={
                "bus_config": bus_config,
                "master_config": master_dcf,
                "master_bin": master_bin,
                "can_interface_name": LaunchConfiguration("can_interface").perform(context),
            }.items()))
    actions.extend([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[robot_description], output="screen"),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[robot_description, controllers], output="screen"),
    ])
    actions.append(Node(
        package="my_ackermann_bot", executable="spawn_ackermann_controllers.py",
        parameters=[{"start_controller": activate_controller, "timeout": 60.0}],
        output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="gazebo"),
        DeclareLaunchArgument("enable_motion", default_value="false"),
        DeclareLaunchArgument("start_controller", default_value="auto"),
        DeclareLaunchArgument("motor_config", default_value=""),
        DeclareLaunchArgument("controllers_file", default_value=""),
        DeclareLaunchArgument("bus_config", default_value=""),
        DeclareLaunchArgument("master_dcf", default_value=""),
        DeclareLaunchArgument("master_bin", default_value=""),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        OpaqueFunction(function=launch_setup),
    ])
