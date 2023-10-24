import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("add_gripper", default_value="true"))
    ld.add_action(DeclareLaunchArgument("robot_ip", default_value="192.168.1.196"))
    ld.add_action(DeclareLaunchArgument("use_fake_hardware", default_value="true"))

    add_gripper = LaunchConfiguration("add_gripper")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('xarm_moveit_config'), 'launch/xarm7_moveit_realmove.launch.py')
        ),
        launch_arguments={
            'add_gripper': add_gripper,
            'robot_ip': robot_ip,
        }.items(),
        condition=UnlessCondition(use_fake_hardware)
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('xarm_moveit_config'), 'launch/xarm7_moveit_fake.launch.py')
        ),
        launch_arguments={
            'add_gripper': add_gripper,
            'robot_ip': robot_ip,
        }.items(),
        condition=IfCondition(use_fake_hardware)
    ))

    ld.add_action(Node(
        package="xarm_demo",
        executable="demo",
        output="screen",
    ))

    return ld