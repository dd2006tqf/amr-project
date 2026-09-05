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
            'queue_capacity': 64,
            'comparator': 'priority_fifo',
            'schedule_rate_hz': 10.0,
            'deadlock_check_rate_hz': 1.0,
            'max_active_missions': 2,
        }]
    )

    safety_gate_node = Node(
        package='amr_dispatcher_ros',
        executable='safety_gate_node',
        name='safety_gate_node',
        output='screen'
    )

    return LaunchDescription([
        dispatcher_node,
        safety_gate_node,
    ])
