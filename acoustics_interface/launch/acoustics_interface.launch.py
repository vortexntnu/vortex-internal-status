from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    acoustics_node = Node(
        package="acoustics_interface",
        executable="acoustics_interface",
        name="acoustics_interface",
        namespace="nautilus",
        output="screen",
    )

    return LaunchDescription([acoustics_node])
