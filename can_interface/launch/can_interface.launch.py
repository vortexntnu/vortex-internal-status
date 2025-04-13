from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    thruster_interface_auv_node = Node(
        package="can_interface",
        executable="can_interface_node",
        name="can_interface_node",
        namespace="orca",
        output="screen",
    )

    return LaunchDescription([thruster_interface_auv_node])
