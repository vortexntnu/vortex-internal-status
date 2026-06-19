from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='base_single.yaml',
        description='YAML config file for base node'
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
            executable='am_base_node',
            name='base_node',
            parameters=[config_file]
        )
    ])