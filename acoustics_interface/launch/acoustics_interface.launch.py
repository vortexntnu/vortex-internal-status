from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    acoustics_node = Node(
        package="acoustics_interface",
        executable="acoustics_ros_node",
        name="acoustics_ros_node",
        namespace="nautilus",
        output="screen",
    )

    return LaunchDescription([acoustics_node])
