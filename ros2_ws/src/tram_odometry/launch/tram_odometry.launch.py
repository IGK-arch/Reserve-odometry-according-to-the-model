from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('tram_odometry'), 'config', 'default.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('vehicle_id', default_value='30618'),
        DeclareLaunchArgument('route_direction', default_value='auto'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='tram_odometry',
            executable='odometry_node',
            name='tram_odometry',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'vehicle_id': ParameterValue(LaunchConfiguration('vehicle_id'), value_type=int),
                 'route_direction': LaunchConfiguration('route_direction'),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
        ),
    ])
