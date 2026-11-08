import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('chuyun_navigation2')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    base_frame = LaunchConfiguration('base_frame')
    laser_frame = LaunchConfiguration('laser_frame')
    publish_laser_tf = LaunchConfiguration('publish_laser_tf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(package_dir, 'maps', 'room.yaml')
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_dir, 'config', 'lite_navigation.yaml')
        ),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('laser_frame', default_value='laser_frame'),
        DeclareLaunchArgument(
            'publish_laser_tf',
            default_value='true',
            description=(
                'Publish the Chuyun fixed base-to-laser transform. Set false '
                'when robot_state_publisher already publishes it.'
            )
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chuyun_laser_static_tf',
            output='screen',
            condition=IfCondition(publish_laser_tf),
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.151',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', base_frame,
                '--child-frame-id', laser_frame,
            ],
        ),
        Node(
            package='chuyun_navigation2',
            executable='lite_navigation_node',
            name='lite_navigation',
            output='screen',
            parameters=[
                params_file,
                {'map_yaml': map_yaml, 'base_frame': base_frame},
            ],
        ),
    ])
