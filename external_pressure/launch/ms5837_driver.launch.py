from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ms5837_driver_auv_node = Node(
        package="ms5837_driver",
        executable="ms5837_driver_node",
        name="ms5837_driver_node",
        namespace="nautilus",
        output="screen",
    )

    return LaunchDescription([ms5837_driver_auv_node])
