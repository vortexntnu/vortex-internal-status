from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    can_interface_node = Node(
        package="can_interface_node",
        executable="can_interface_node",
        name="can_interface_node",
        namespace="nautilus",
        output="screen",
    )

    return LaunchDescription([can_interface_node])
