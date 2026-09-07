import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.actions import RegisterEventHandler, EmitEvent
import lifecycle_msgs.msg

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

    # 声明自动触发 configure 事件
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == dispatcher_node,
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # 当 configure 成功后自动触发 activate 事件
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=dispatcher_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node == dispatcher_node,
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
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

    dispatcher_visualizer_node = Node(
        package='amr_dispatcher_ros',
        executable='dispatcher_visualizer_node',
        name='dispatcher_visualizer_node',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'marker_topic': '/dispatcher/markers',
            'stations_file': 'config/stations.yaml'
        }]
    )

    lifecycle_coordinator_node = Node(
        package='amr_dispatcher_ros',
        executable='lifecycle_coordinator_node',
        name='lifecycle_coordinator_node',
        output='screen',
        parameters=[{
            'auto_bringup': True,
            'check_period_ms': 500
        }]
    )

    return LaunchDescription([
        dispatcher_node,
        configure_event,
        activate_event,
        safety_gate_node,
        chassis_driver_node,
        path_tracker_node,
        rest_gateway_node,
        dispatcher_visualizer_node,
        lifecycle_coordinator_node,
    ])
