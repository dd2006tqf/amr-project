import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_dir = get_package_share_directory('chuyun_navigation2')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(package_dir, 'maps', 'room.yaml')
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_dir, 'config', 'lite_navigation.yaml')
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_dir, 'launch', 'lite_navigation.launch.py')
            ),
            launch_arguments={
                'map': map_yaml,
                'params_file': params_file,
            }.items(),
        ),
    ])
