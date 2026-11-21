import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("my_diff_bot")
    xacro_file = os.path.join(pkg_share, "urdf", "my_diff_bot.urdf.xacro")

    # 1. 解析 Xacro，显式指定 value_type=str 避免 Foxy 参数解析报错
    robot_description = {
        "robot_description": ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    }

    # 2. 发布机器人 TF 状态
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # 3. 启动 Gazebo Classic 仿真器（自动发布 /clock，无需额外桥接节点）
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        )
    )

    # 4. 使用 gazebo_ros 的 spawn_entity.py 生成小车模型
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "my_diff_bot",
            "-z", "0.05",
        ],
        output="screen",
    )

    # 5. 加载并激活控制器（Foxy 中可执行文件名为 spawner.py）
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_drive_base_controller", "--controller-manager", "/controller_manager"],
    )

    # 确保模型成功生成进 Gazebo 后，再激活控制器
    delay_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                joint_state_broadcaster_spawner,
                diff_drive_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delay_controllers_after_spawn,
    ])