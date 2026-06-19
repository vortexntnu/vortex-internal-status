from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='drone_single.yaml',
        description='YAML config file for drone node'
    )

    config_file = PathJoinSubstitution([
        FindPackageShare('acoustic_modem'),
        'config',
        LaunchConfiguration('config')
    ])

    return LaunchDescription([
        config_arg,
        Node(
            package='acoustic_modem',
            executable='am_drone_node',
            name='drone_node',
            parameters=[config_file]
        )
    ])