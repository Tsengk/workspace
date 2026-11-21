import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("my_ackermann_bot")
    xacro_file = os.path.join(pkg_share, "urdf", "ackermann_car.urdf.xacro")

    robot_description = {
        "robot_description": ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        )
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "ackermann_car", "-z", "0.05"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    ackermann_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["ackermann_controller", "--controller-manager", "/controller_manager"],
    )

    # 模型生成后延迟 2 秒激活控制器，避免 Gazebo 插件尚未就绪导致 spawner 超时
    delay_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[joint_state_broadcaster_spawner, ackermann_controller_spawner],
                )
            ],
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delay_controllers_after_spawn,
    ])