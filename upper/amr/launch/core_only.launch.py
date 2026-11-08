from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.actions import RegisterEventHandler, EmitEvent
import lifecycle_msgs.msg

def generate_launch_description():
    # B7：与 full_system 同源的参数文件——core_only 用内联覆盖
    # （容量 64 / 并发 2），仍走 yaml + 覆盖的读取路径
    ros_share = get_package_share_directory('amr_dispatcher_ros')
    dispatcher_yaml = os.path.join(ros_share, 'config', 'dispatcher.yaml')
    stations_file_share = os.path.join(ros_share, 'config', 'stations.yaml')
    safety_yaml = os.path.join(ros_share, 'config', 'safety.yaml')

    dispatcher_node = LifecycleNode(
        package='amr_dispatcher_ros',
        executable='dispatcher_node',
        name='dispatcher_node',
        namespace='',
        output='screen',
        parameters=[
            dispatcher_yaml,
            {
                # core_only 尺寸更小：容量/并发收紧、站点表 share 绝对路径
                'queue_capacity': 64,
                'max_active_missions': 2,
                'stations_file': stations_file_share,
            },
        ]
    )

    # B7 补齐：core_only 之前缺 configure/activate 事件，节点永远停在 unconfigured。
    # 复用 full_system 的同款模式：触发 configure，成功后 on_state_transition -> activate。
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == dispatcher_node,
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

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
        parameters=[safety_yaml]
    )

    return LaunchDescription([
        dispatcher_node,
        configure_event,
        activate_event,
        safety_gate_node,
    ])
