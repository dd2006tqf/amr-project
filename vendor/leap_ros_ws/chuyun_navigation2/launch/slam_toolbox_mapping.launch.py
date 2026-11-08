import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    chuyun_navigation2_dir = get_package_share_directory('chuyun_navigation2')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_params = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(chuyun_navigation2_dir, 'config', 'mapper_params_online_async_1hz.yaml')
    )
    nav2_params = LaunchConfiguration(
        'nav2_params_file',
        default=os.path.join(chuyun_navigation2_dir, 'param', 'xuegebot.yaml')
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use simulation clock if true'),
        DeclareLaunchArgument('slam_params_file', default_value=slam_params, description='Full path to slam_toolbox mapper params file'),
        DeclareLaunchArgument('nav2_params_file', default_value=nav2_params, description='Full path to Nav2 parameter file'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params,
            }.items(),
        ),
    ])
