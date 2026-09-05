from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    chassis_driver_node = Node(
        package='amr_dispatcher_ros',
        executable='chassis_driver_node',
        name='chassis_driver_node',
        output='screen',
        parameters=[{
            'serial_device': '/dev/ttyUSB0',
            'serial_baud': 115200,
            'poll_rate_hz': 50.0,
            'health_rate_hz': 2.0,
        }]
    )

    safety_gate_node = Node(
        package='amr_dispatcher_ros',
        executable='safety_gate_node',
        name='safety_gate_node',
        output='screen'
    )

    link_health_pub = Node(
        package='amr_dispatcher_ros',
        executable='link_health_publisher',
        name='link_health_publisher',
        output='screen'
    )

    return LaunchDescription([
        chassis_driver_node,
        safety_gate_node,
        link_health_pub,
    ])
