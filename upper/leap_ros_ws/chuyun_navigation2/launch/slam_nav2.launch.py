from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    nav_dir = get_package_share_directory('chuyun_navigation2')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_dir, 'launch', 'slam_toolbox_localization_nav2.launch.py')
            )
        ),
    ])
