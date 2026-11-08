from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_base_controller'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    return LaunchDescription([
        controller_manager,
        diff_drive_spawner,
        joint_state_broadcaster_spawner,
    ])
