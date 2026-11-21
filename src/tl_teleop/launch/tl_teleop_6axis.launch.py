import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("tl_teleop")
    config_path = os.path.join(pkg_share, "config", "tl_teleop_6axis_config.yaml")
    gripper_config = os.path.join(pkg_share, "config", "tl_gripper_config.yaml")
    gripper_enabled = LaunchConfiguration("gripper_enabled")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gripper_enabled", default_value="false",
                description="Enable the USB/RS485 Lebai gripper and Trigger control",
            ),
            DeclareLaunchArgument(
                "gripper_port",
                default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
                description="Lebai gripper serial port",
            ),
            Node(
                package="tl_teleop", executable="tl_gripper",
                name="tl_gripper_node", output="screen",
                condition=IfCondition(gripper_enabled),
                parameters=[gripper_config, {"port": LaunchConfiguration("gripper_port")}],
            ),
            Node(
                package="tl_teleop",
                executable="tl_teleop",
                name="tl_teleop_node",
                output="screen",
                parameters=[config_path, {
                    "gripper_enabled": ParameterValue(gripper_enabled, value_type=bool),
                }],
            )
        ]
    )
