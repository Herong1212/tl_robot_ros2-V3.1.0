import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# NOTE 供 ROS launch 框架调用的函数，用来描述“启动哪些程序、传什么参数”
def generate_launch_description():

    arm_type = "tcb610"
    use_sim = "false"

    # * 子 launch1：启动 tl_driver.launch.py
    tl_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("tl_driver"),
                "launch",
                "tl_driver.launch.py",
            )
        ),
        launch_arguments={
            "arm_type": arm_type,
        }.items(),
    )

    # * 子 launch2：启动 tl_description.launch.py
    tl_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("tl_description"),
                "launch",
                "tl_description.launch.py",
            )
        ),
        launch_arguments={"arm_type": arm_type, "use_sim": use_sim}.items(),
    )

    # NOTE 函数返回一份“启动描述”，告诉 launch 框架接下来需要做什么
    return LaunchDescription(
        [
            tl_driver,
            tl_description,
        ]
    )
