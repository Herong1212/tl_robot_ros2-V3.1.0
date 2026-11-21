import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    driver_config = os.path.join(
        get_package_share_directory("tl_driver"), "config", "tl_tcb610_config.yaml"
    )
    teleop_launch = os.path.join(
        get_package_share_directory("tl_teleop"), "launch", "tl_teleop_6axis.launch.py"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_ip",
                default_value="192.168.1.13",
                description="TCB610 控制器 IP",
            ),
            DeclareLaunchArgument("gripper_enabled", default_value="false"),
            DeclareLaunchArgument(
                "gripper_port",
                default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            ),
            Node(
                package="tl_driver",
                executable="tl_driver",
                name="tl_driver",
                output="screen",
                parameters=[
                    driver_config,
                    {
                        "arm_ip": LaunchConfiguration("arm_ip"),
                        # Slow temperature queries share the IK connection.
                        "joint_monitor_rate": 0.0,
                        "feedback_rate": 100.0,
                    },
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(teleop_launch),
                launch_arguments={
                    "gripper_enabled": LaunchConfiguration("gripper_enabled"),
                    "gripper_port": LaunchConfiguration("gripper_port"),
                }.items(),
            ),
        ]
    )
