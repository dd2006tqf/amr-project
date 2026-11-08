import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    chuyun_navigation2_dir = get_package_share_directory('chuyun_navigation2')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration(
        'map',
        default=os.path.join(chuyun_navigation2_dir, 'maps', 'room.yaml')
    )
    nav2_param_path = LaunchConfiguration(
        'params_file',
        default=os.path.join(chuyun_navigation2_dir, 'param', 'xuegebot.yaml')
    )
    slam_params = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(slam_toolbox_share, 'config', 'mapper_params_localization.yaml')
    )

    configured_params = RewrittenYaml(
        source_file=nav2_param_path,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_path,
        },
        convert_types=True,
    )

    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use simulation clock if true'),
        DeclareLaunchArgument('map', default_value=map_yaml_path, description='Full path to map file to load'),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path, description='Full path to Nav2 parameter file'),
        DeclareLaunchArgument('slam_params_file', default_value=slam_params, description='Full path to slam_toolbox localization params file'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_share, 'launch', 'localization_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params,
            }.items(),
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': lifecycle_nodes},
            ],
        ),
    ])
