import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
import lifecycle_msgs.msg

def generate_launch_description():
    # 可覆盖参数，默认值与改动前保持一致，不影响本机开发
    serial_device = LaunchConfiguration('serial_device')
    health_rate_hz = LaunchConfiguration('health_rate_hz')
    # Task 8：可选地图导入链路（默认关闭，完全向后兼容）
    use_map_server = LaunchConfiguration('use_map_server')
    map_yaml = LaunchConfiguration('map_yaml')
    markers_file = LaunchConfiguration('markers_file')
    # C1：站点目录路径可覆盖，默认值随 use_map_server 在静态/生成目录间切换
    stations_file = LaunchConfiguration('stations_file')
    # Task 4：VDA 5050 MQTT 网关（默认关闭，完全向后兼容）
    use_vda5050 = LaunchConfiguration('use_vda5050')
    mqtt_broker_host = LaunchConfiguration('mqtt_broker_host')
    mqtt_port = LaunchConfiguration('mqtt_port')
    # M5：网关身份与协议参数必须可从 launch 暴露——两实例同 serial 会撞 client_id
    manufacturer = LaunchConfiguration('manufacturer')
    serial_number = LaunchConfiguration('serial_number')
    protocol_version = LaunchConfiguration('protocol_version')
    heartbeat_ms = LaunchConfiguration('heartbeat_ms')

    declared_arguments = [
        DeclareLaunchArgument(
            'serial_device',
            default_value='/dev/ttyUSB0',
            description='底盘串口设备路径（服务器部署时由入口脚本传入 PTY 路径）',
        ),
        DeclareLaunchArgument(
            'health_rate_hz',
            default_value='5.0',
            description='底盘链路健康/心跳发布频率。安全门的 heartbeat_timeout_ms '
                        '默认 500ms，5Hz 留足 2.5 倍心跳余量避免看门狗误触发（B1）。',
        ),
        DeclareLaunchArgument(
            'use_map_server',
            default_value='false',
            description='为 true 时拉起 nav2_map_server 发布 /map 并启动 map_ingest_node '
                        '自动生成 stations.yaml 与拓扑 Marker（默认 false，保持向后兼容）。',
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value='',
            description='nav2 map_server 的地图 yaml 路径（use_map_server:=true 时必填，'
                        '指向含 map.yaml/map.pgm 的地图包目录内 yaml 文件）。',
        ),
        DeclareLaunchArgument(
            'markers_file',
            default_value='',
            description='站点语义 anchors yaml（id/type/x/y/yaw 列表），供 map_ingest_node '
                        '吸附生成业务站点；为空时 map_ingest_node 等待有效文件。',
        ),
        DeclareLaunchArgument(
            'use_vda5050',
            default_value='false',
            description='为 true 时拉起 vda5050_gateway_node（VDA 5050 <-> ROS 2 MQTT 网关，'
                        '订阅 uagv/v2/+/+/{state,connection} 并发布 /fleet/* 话题）；'
                        '默认 false 保持向后兼容。',
        ),
        DeclareLaunchArgument(
            'mqtt_broker_host',
            default_value='127.0.0.1',
            description='VDA 5050 网关连接的 MQTT Broker 地址（容器部署可传 broker 容器名）。',
        ),
        DeclareLaunchArgument(
            'mqtt_port',
            default_value='1883',
            description='VDA 5050 网关连接的 MQTT Broker 端口。',
        ),
        DeclareLaunchArgument(
            'manufacturer',
            default_value='RobotManufacturer',
            description='VDA 5050 网关身份 manufacturer：决定 MQTT topic 前缀与入站'
                        '消息厂商匹配；多网关同机部署时各实例必须不同（M5）。',
        ),
        DeclareLaunchArgument(
            'serial_number',
            default_value='default',
            description='VDA 5050 网关 serialNumber：参与 client_id'
                        '（amr_dispatcher_gateway_{serial}）与自身心跳过滤，'
                        '两实例取值相同会互相顶掉 MQTT 连接（M5）。',
        ),
        DeclareLaunchArgument(
            'protocol_version',
            default_value='v2',
            description='VDA 5050 协议版本：v2 -> uagv/v2 topic 前缀，'
                        'v3 -> vda5050/v3 topic 前缀。',
        ),
        DeclareLaunchArgument(
            'heartbeat_ms',
            default_value='5000',
            description='网关 connection 心跳（retained ONLINE）发布周期，毫秒。',
        ),
    ]

    # B7：节点参数文件集中放 share/amr_dispatcher_ros/config/，launch 只负责注入；
    # 字典条目叠在 yaml 之后，方便运行时按 LaunchConfiguration 覆盖。
    ros_share = get_package_share_directory('amr_dispatcher_ros')
    stations_file_share = os.path.join(ros_share, 'config', 'stations.yaml')
    dispatcher_yaml = os.path.join(ros_share, 'config', 'dispatcher.yaml')
    chassis_yaml = os.path.join(ros_share, 'config', 'chassis.yaml')
    safety_yaml = os.path.join(ros_share, 'config', 'safety.yaml')
    # Task 8：map_ingest_node 的生成产物目录（节点负责 create_directories，
    # tmp+rename 原子写出后由 StationCatalogReloader 按 mtime 热重载）。
    generated_stations_share = os.path.join(ros_share, 'config', 'generated', 'stations.yaml')

    # C1：stations_file 默认值随 use_map_server 切换——开启地图导入后，调度与
    # 可视化必须监听 map_ingest 的生成目录，否则 StationCatalogReloader 一直
    # 盯着旧的静态 config/stations.yaml，生成链路与调度链路脱节。
    # 判定与 IfCondition 同款布尔词表，保证 use_map_server:=1/True 等写法一致。
    stations_file_default = PythonExpression([
        "'", generated_stations_share, "' if str('", use_map_server,
        "').lower() in ('true', '1', 'yes', 'on') else '", stations_file_share, "'",
    ])
    declared_arguments.append(
        DeclareLaunchArgument(
            'stations_file',
            default_value=stations_file_default,
            description='站点目录 YAML 路径。默认 use_map_server:=true 时指向 '
                        'config/generated/stations.yaml（map_ingest 生成物，闭环热重载），'
                        '否则指向静态 config/stations.yaml；可显式覆盖。',
        )
    )

    dispatcher_node = LifecycleNode(
        package='amr_dispatcher_ros',
        executable='dispatcher_node',
        name='dispatcher_node',
        namespace='',
        output='screen',
        parameters=[
            dispatcher_yaml,
            # stations_file 用 share 绝对路径：摆脱 colcon 工作区 cwd 依赖；
            # C1：默认随 use_map_server 指向 map_ingest 生成目录（见上方参数声明）。
            {'stations_file': stations_file},
        ]
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
        parameters=[safety_yaml]
    )

    chassis_driver_node = Node(
        package='amr_dispatcher_ros',
        executable='chassis_driver_node',
        name='chassis_driver_node',
        output='screen',
        parameters=[
            chassis_yaml,
            {
                # LaunchConfiguration 产出字符串，需显式声明为 double 以免参数类型不匹配；
                # serial_device 同理保留 CLI 覆盖位。
                'serial_device': serial_device,
                'health_rate_hz': ParameterValue(health_rate_hz, value_type=float),
            },
        ]
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

    # B3：navigate_sequence_bridge_node 入 full_system——/navigate_sequence action
    # 由 bridge 提供，dispatcher 的 navigation_backend:=action 才能接通。
    navigate_sequence_bridge_node = Node(
        package='amr_dispatcher_ros',
        executable='navigate_sequence_bridge_node',
        name='navigate_sequence_bridge_node',
        output='screen',
        parameters=[{
            'use_nav2': False,
            'nav2_action_name': '/navigate_to_pose',
            'plan_topic': '/plan',
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
            # 与 dispatcher 一致：stations_file 用 share 绝对路径（B7），
            # 且 C1 下默认随 use_map_server 指向生成目录。
            'stations_file': stations_file,
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

    # ---- Task 8：可选地图导入链路（use_map_server:=true 时启用，默认关闭） ----
    # nav2_map_server 按官方格式解析 map.yaml/map.pgm 并以 latched 语义发布 /map。
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        condition=IfCondition(use_map_server),
        parameters=[{
            'yaml_filename': map_yaml,
            'topic_name': 'map',
            'frame_id': 'map',
        }],
    )

    map_server_configure_event = EmitEvent(
        condition=IfCondition(use_map_server),
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == map_server_node,
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    map_server_activate_event = RegisterEventHandler(
        condition=IfCondition(use_map_server),
        event_handler=OnStateTransition(
            target_lifecycle_node=map_server_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node == map_server_node,
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    # map_ingest_node：订阅 /map，跑核心库算法流水线，原子写出
    # share/config/generated/stations.yaml 并发布 RViz 拓扑 Marker。
    map_ingest_node = Node(
        package='amr_dispatcher_ros',
        executable='map_ingest_node',
        name='map_ingest_node',
        namespace='',
        output='screen',
        condition=IfCondition(use_map_server),
        parameters=[{
            'map_topic': '/map',
            'markers_file': markers_file,
            'output_stations_file': generated_stations_share,
            'publish_markers': True,
            'marker_topic': '/dispatcher/topology_markers',
        }],
    )

    # ---- Task 4：VDA 5050 MQTT 网关（use_vda5050:=true 时启用，默认关闭） ----
    # 薄壳网关：MQTT <-> ROS 2 话题转换；安全铁律——在线/离线只翻转
    # /fleet/{vehicle_id}/online，绝不释放路权锁（锁保留归调度核心，Spec §3）。
    vda5050_gateway_node = Node(
        package='amr_dispatcher_ros',
        executable='vda5050_gateway_node',
        name='vda5050_gateway_node',
        namespace='',
        output='screen',
        condition=IfCondition(use_vda5050),
        parameters=[{
            # 节点参数与启动门控同源，保证 use_vda5050:=true/false 语义一致
            'use_vda5050': ParameterValue(use_vda5050, value_type=bool),
            'mqtt_broker_host': mqtt_broker_host,
            'mqtt_port': ParameterValue(mqtt_port, value_type=int),
            # M5：身份/协议参数透传（默认与节点内默认值一致，保持向后兼容）
            'manufacturer': manufacturer,
            'serial_number': serial_number,
            'protocol_version': protocol_version,
            'heartbeat_ms': ParameterValue(heartbeat_ms, value_type=int),
        }],
    )

    return LaunchDescription(declared_arguments + [
        dispatcher_node,
        configure_event,
        activate_event,
        safety_gate_node,
        chassis_driver_node,
        path_tracker_node,
        navigate_sequence_bridge_node,
        rest_gateway_node,
        dispatcher_visualizer_node,
        lifecycle_coordinator_node,
        map_server_node,
        map_server_configure_event,
        map_server_activate_event,
        map_ingest_node,
        vda5050_gateway_node,
    ])
