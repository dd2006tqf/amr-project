import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    dispatcher_node = LifecycleNode(
        package='amr_dispatcher_ros',
        executable='dispatcher_node',
        name='dispatcher_node',
        namespace='',
        output='screen',
        parameters=[{
            'queue_capacity': 128,
            'comparator': 'priority_fifo',
            'schedule_rate_hz': 10.0,
            'deadlock_check_rate_hz': 1.0,
            'max_active_missions': 4,
        }]
    )

    safety_gate_node = Node(
        package='amr_dispatcher_ros',
        executable='safety_gate_node',
        name='safety_gate_node',
        output='screen',
        parameters=[{
            'max_linear_x_mps': 1.5,
            'max_linear_y_mps': 1.0,
            'max_angular_z_radps': 2.0,
            'heartbeat_timeout_ms': 500,
            'soft_timeout_ms': 2000,
        }]
    )

    chassis_driver_node = Node(
        package='amr_dispatcher_ros',
        executable='chassis_driver_node',
        name='chassis_driver_node',
        output='screen',
        parameters=[{
            'serial_device': '/dev/ttyUSB0',
            'serial_baud': 115200,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'publish_tf': True,
            'poll_rate_hz': 50.0,
            'health_rate_hz': 2.0,
        }]
    )

    path_tracker_node = Node(
        package='amr_dispatcher_ros',
        executable='path_tracker_node',
        name='path_tracker_node',
        output='screen',
        parameters=[{
            'controller_type': 'pure_pursuit',
            'lookahead_distance': 0.6,
            'target_speed': 0.5,
            'control_rate_hz': 20.0,
        }]
    )

    rest_gateway_node = Node(
        package='amr_dispatcher_tools',
        executable='rest_gateway',
        name='rest_gateway_node',
        output='screen',
        parameters=[{'port': 8080}]
    )

    return LaunchDescription([
        dispatcher_node,
        safety_gate_node,
        chassis_driver_node,
        path_tracker_node,
        rest_gateway_node,
    ])
