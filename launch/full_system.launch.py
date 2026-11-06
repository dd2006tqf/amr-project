import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.actions import (DeclareLaunchArgument, RegisterEventHandler, EmitEvent,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
import lifecycle_msgs.msg

def launch_config_value(context, name):
    """读取 launch 参数的字符串值（OpaqueFunction 上下文用）。"""
    return context.perform_substitution(LaunchConfiguration(name))


def generate_launch_description():
    # 可覆盖参数，默认值与改动前保持一致，不影响本机开发
    serial_device = LaunchConfiguration('serial_device')
    health_rate_hz = LaunchConfiguration('health_rate_hz')
    # T5：底盘协议与波特率可覆盖——切台架不必再手改 config/chassis.yaml
    #（手改会把仓库默认值弄脏，演示完还得记得改回来）。
    chassis_protocol = LaunchConfiguration('chassis_protocol')
    chassis_baud = LaunchConfiguration('chassis_baud')
    # T6：地图原点 → odom 的静态外参（与 path_tracker 的 odom_origin_* 同源）
    odom_origin_x = LaunchConfiguration('odom_origin_x')
    odom_origin_y = LaunchConfiguration('odom_origin_y')
    odom_origin_yaw = LaunchConfiguration('odom_origin_yaw')
    publish_map_to_odom = LaunchConfiguration('publish_map_to_odom')
    # T4：终点判定容差（米），透传给 path_tracker（pure_pursuit 与 stanley 共用）
    goal_tolerance = LaunchConfiguration('goal_tolerance')
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
    # Task 5：受管 VDA 5050 车辆列表（逗号分隔，默认空 = 功能关闭）
    vda5050_vehicles = LaunchConfiguration('vda5050_vehicles')
    # P0.6 / A4：多车并发的 local_sim 车辆清单（逗号分隔；空 = 单车兼容模式）
    vehicle_ids = LaunchConfiguration('vehicle_ids')
    # 多车底盘串口清单（逗号分隔，与 vehicle_ids 一一对应；空 = 沿用 serial_device）
    serial_devices = LaunchConfiguration('serial_devices')
    # 相位 11：进程守护（节点崩溃自动重启）。置 false 恢复改动前的"死了就一直死"。
    respawn_enabled = LaunchConfiguration('respawn_enabled')
    # 重启延迟**不能**做成 launch 参数（`respawn_delay:=5.0` 这种写法会静默失效）。
    #
    # 原因在 jazzy 的 launch 实现里：ExecuteLocal.__init__ 把 respawn_delay 原样存
    # （execute_local.py:210，不像 respawn 那样过 normalize_typed_substitution），
    # 而重启分支里直接拿它做数值比较（:611 `self.__respawn_delay > 0.0`）。
    # 因此传 LaunchConfiguration / ParameterValue / PythonExpression 任何一种都会在
    # **第一次真正需要重启时**抛
    #     TypeError: '>' not supported between instances of 'LaunchConfiguration' and 'float'
    # 启动阶段一切正常，等节点第一次崩溃才发现守护没生效——比启动即崩更难排查。
    #
    # 实测记录（五种写法，除了裸 float 全部 TypeError）：
    #   float 1.0 ✅ / LaunchConfiguration ❌ / ParameterValue(float) ❌
    #   normalize_typed_substitution(float) ❌ / PythonExpression ❌
    #
    # 所以这里固定成真实的 float。要改延迟就改这个常量。
    respawn_delay_sec = 2.0
    # 相位 11：在线拓扑叠加层（施工区/单向/临时站点）的独立持久化文件。
    # 必须是**绝对路径**：HandleMapUpload 写的是相对 config/generated/stations.yaml，
    # 叠加层再走相对路径就会踩同一个 cwd 陷阱（容器 working_dir 变了就写丢）。
    topology_overlay_file = LaunchConfiguration('topology_overlay_file')

    def as_bool(value):
        """把 launch 的布尔参数折叠成真正的 bool。

        respawn 是 ExecuteProcess 的 **kwargs（不是 ROS 参数），所以不能包成
        ParameterValue —— 实测会抛
            TypeError: value should be either a scalar, a substitutions, ...
        也不能直接 str(value)：此时 value 是 LaunchConfiguration，还没解析，
        str() 拿到的是对象 repr。正确做法是让 launch 在**执行期**解析：
        ExecuteProcess 支持 substitution，给它一个返回 'true'/'false' 的
        PythonExpression 即可（它内部会按字符串判真）。
        """
        return PythonExpression([
            "'true' if '", value, "'.lower() in ('true','1','yes','on') else 'false'"
        ])

    def supervision():
        """全节点共用的守护参数。

        ExecuteProcess 接受 respawn / respawn_delay（LifecycleNode 经 **kwargs 透传，
        已在 jazzy 镜像内实测）。respawn_max_retries 保持默认（无限重试）：崩溃循环
        由日志与 /metrics 的 amr_ready 暴露，人工设上限只会让第三次崩溃变成永久停机。

        respawn 用执行期 substitution（可被 `respawn_enabled:=false` 关掉），
        respawn_delay 用固定 float（原因见上方 respawn_delay_sec 的注释）。
        """
        return {
            'respawn': as_bool(respawn_enabled),
            'respawn_delay': respawn_delay_sec,
        }

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
        # T5：协议/波特率从 CLI 或环境变量传入，覆盖 config/chassis.yaml 的同名值。
        # 默认值与 yaml 一致（text / 115200），不改变现网行为。
        DeclareLaunchArgument(
            'chassis_protocol',
            default_value='text',
            description='底盘链路协议：text（文本行）| mick_binary（0xAE 0xEA 帧）。'
                        '接真实 MCU 时传 mick_binary；两侧协议必须一致，否则上行 odom 全丢。',
        ),
        DeclareLaunchArgument(
            'chassis_baud',
            default_value='115200',
            description='底盘串口波特率，必须与 MCU 固件一致（台架常设 460800）。',
        ),
        # T6：静态 map→odom。chassis_driver 只发 odom→base_link，path_tracker 在内部
        # 把 odom 位姿平移到 map，但 TF 树上没有 map→odom —— RViz/Foxglove 里车会
        # 永远停在地图原点附近。这三个参数与 path_tracker 的 odom_origin_* 同源，
        # 保证"可视化看到的"和"tracker 算的"是同一套数学。
        DeclareLaunchArgument(
            'odom_origin_x',
            default_value='0.0',
            description='odom 原点在 map 帧下的 x（米）。真机台架必须标定（车摆在哪个 map 坐标上电）。',
        ),
        DeclareLaunchArgument(
            'odom_origin_y',
            default_value='0.0',
            description='odom 原点在 map 帧下的 y（米）。',
        ),
        DeclareLaunchArgument(
            'odom_origin_yaw',
            default_value='0.0',
            description='odom 原点在 map 帧下的 yaw（弧度）。',
        ),
        DeclareLaunchArgument(
            'publish_map_to_odom',
            default_value='true',
            description='是否发布静态 map→odom 变换（T6）。false = 关闭，保持引入前的 TF 树。'
                        'odom_origin_* 全为 0 时该变换是恒等变换，发布它不影响现有可视化。',
        ),
        # T4：终点容差。默认 0.10 与控制器内置值逐位一致（不改变现网行为）。
        # 台架标定偏几十厘米时调到 0.3 —— 否则车永远进不了到达窗口，
        # 任务会挂到 10s 后被死锁检测判 kIdleActive。
        DeclareLaunchArgument(
            'goal_tolerance',
            default_value='0.10',
            description='终点判定容差（米）。pure_pursuit 与 stanley 共用；'
                        '台架标定不准时调大（如 0.3），仿真/精确标定下保持 0.10。',
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
            'use_vda5050',            default_value='false',
            description='为 true 时拉起 vda5050_gateway_node（VDA 5050 <-> ROS 2 MQTT 网关，'
                        '订阅 uagv/v2/+/+/{state,connection} 并发布 /fleet/* 话题）；'
                        '默认 false 保持向后兼容。',
        ),
        DeclareLaunchArgument(
            'vda5050_vehicles',
            default_value='',
            description='受管 VDA 5050 小车清单（逗号分隔 vehicle_id，如 agv_001,agv_002）。'
                        'dispatcher_node 据此订阅 /fleet/{vid}/{state,online}，'
                        '启用 Task 5 逐段路权动态释放与断链锁保留守卫；空 = 关闭（现网行为）。',
        ),
        DeclareLaunchArgument(
            'vehicle_ids',
            default_value='',
            description='P0.6 多车并发的 local_sim 车辆清单（逗号分隔 vehicle_id，如 amr_01,amr_02）。'
                        '非空时为每台车各起一组 path_tracker/safety_gate/chassis，'
                        '话题按 /plan/{vid}、/odom/{vid}、/tracking/error/{vid} 隔离；'
                        '空 = 单车兼容模式（话题名逐字不变）。',
        ),
        DeclareLaunchArgument(
            'serial_devices',
            default_value='',
            description='多车底盘串口清单（逗号分隔，与 vehicle_ids 一一对应）。'
                        '为空时所有车沿用 serial_device 参数（单机/仿真场景）。',
        ),
        DeclareLaunchArgument(
            'mqtt_broker_host',
            default_value='127.0.0.1',
            description='VDA 5050 网关连接的 MQTT Broker 地址（容器部署可传 broker 容器名）。',
        ),        DeclareLaunchArgument(
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
        DeclareLaunchArgument(
            'respawn_enabled',
            default_value='true',
            description='节点崩溃后自动重启（进程守护）。置 false 恢复相位 11 之前的'
                        '行为（节点死了就一直是死的），仅用于对照实验与故障演练。',
        ),
        # 注意：**没有** respawn_delay 这个 launch 参数，这不是遗漏。
        # 延迟固定为 generate_launch_description 里的 respawn_delay_sec 常量——
        # jazzy 的 ExecuteLocal 不对该字段做 substitution 求值，做成参数会在
        # 第一次真正重启时抛 TypeError（详见 respawn_delay_sec 处的注释）。
        # 与其提供一个"传了也不生效"的假参数，不如不提供。
        DeclareLaunchArgument(
            'topology_overlay_file',
            default_value='',
            description='运行时拓扑叠加层（临时施工区/单向/临时站点）的持久化文件绝对路径。'
                        '空 = 功能关闭（现网行为）。生产建议放在 config/generated/ 下的'
                        '独立文件，绝不与 map_ingest 生成的 stations.yaml 同路径。',
        ),
    ]

    # B7：节点参数文件集中放 share/amr_dispatcher_ros/config/，launch 只负责注入；
    # 字典条目叠在 yaml 之后，方便运行时按 LaunchConfiguration 覆盖。
    ros_share = get_package_share_directory('amr_dispatcher_ros')
    stations_file_share = os.path.join(ros_share, 'config', 'stations.yaml')
    dispatcher_yaml = os.path.join(ros_share, 'config', 'dispatcher.yaml')
    chassis_yaml = os.path.join(ros_share, 'config', 'chassis.yaml')
    safety_yaml = os.path.join(ros_share, 'config', 'safety.yaml')
    # 相位 11：网关参数集中到文件（探针阈值 / 告警 webhook / 拓扑操作超时）。
    # 全部新参数默认关闭或取合理值，因此行为与改动前一致；
    # port 仍由下方字典条目覆盖，保持 8080 这个对外契约不变。
    rest_gateway_yaml = os.path.join(ros_share, 'config', 'rest_gateway.yaml')
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

    # 阶段 3：消费者迁移总开关（一键切换，默认关）。
    #
    # 开启后**所有**消费者（dispatcher_node / safety_gate_node /
    # link_health_publisher / dispatcher_visualizer_node /
    # lifecycle_coordinator_node / navigate_sequence_bridge_node /
    # rest_gateway / dispatcher_cli）改从 *_proto 话题读取，适配器内部
    # 反投影回 rosidl——业务逻辑与回调一字未改。
    #
    # 为什么是单个全局开关而不是逐节点：迁移期的风险是「一半节点读 rosidl、
    # 一半读 proto」，那时两侧数据流都在、谁看谁都不一致，反而最难排查。
    # 一键切换让 A/B 对比只需改一个参数，且回滚同样只改一处。
    #
    # 前置条件：发布端的 proto_dual_publish 必须全开（launch 与 config 默认已开）。
    # 订阅一条没人发布的 *_proto 话题时 DDS 不报错，只会静默收不到——
    # 这正是本节特意把「发布开关」与「订阅开关」分成两个参数的原因。
    declared_arguments.append(
        DeclareLaunchArgument(
            'proto_dual_subscribe',
            default_value='false',
            description='阶段 3 消费者迁移总开关。true 时全部消费者改读 *_proto '
                        '话题（内部反投影回 rosidl，业务逻辑不变）；'
                        '需发布端 proto_dual_publish 已开启。',
        )
    )

    # launch 参数是字符串，这里一次性规范成 Python bool 再喂给各节点，
    # 避免每个节点各写一遍字符串判定（各写法不一致是 Phase 1 踩过的坑）。
    proto_dual_subscribe = PythonExpression([
        "str('", LaunchConfiguration('proto_dual_subscribe'),
        "').lower() in ('true', '1', 'yes', 'on')",
    ])

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
            # Task 5：受管小车清单透传（空 = 逐段解锁/断链守卫关闭）
            {'vda5050_vehicles': ParameterValue(vda5050_vehicles, value_type=str)},
            # P0.6 / A4：local_sim 车辆清单（空 = 单车兼容模式，话题名逐字不变）
            {'vehicle_ids': ParameterValue(vehicle_ids, value_type=str)},
            # 相位 11：拓扑叠加层文件（空 = 在线拓扑热生效关闭，行为与改动前一致）
            {'topology_overlay_file': ParameterValue(topology_overlay_file, value_type=str)},
            # 阶段 3：/tracking/error 双订阅总开关
            {'proto_dual_subscribe': proto_dual_subscribe},
        ],
        # 相位 11：重生后节点回到 unconfigured；launch 的 CONFIGURE/ACTIVATE
        # 只发一次，二次提拉由 lifecycle_coordinator_node（auto_bringup，500ms
        # 轮询 /dispatcher_node/get_state）完成——协调器自己也在 respawn 名单里，
        # 所以"调度器自愈依赖协调器存活"这条依赖必须有守护兜底。
        **supervision(),
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

    # 单车兼容模式专用节点：仅当 vehicle_ids 为空时启用。
    # 多车并发时由 _multi_vehicle_nodes() 逐车起同名节点（namespace 隔离）。
    single_vehicle_only = IfCondition(PythonExpression([
        "'", vehicle_ids, "' == ''",
    ]))

    safety_gate_node = Node(
        package='amr_dispatcher_ros',
        executable='safety_gate_node',
        name='safety_gate_node',
        output='screen',
        condition=single_vehicle_only,
        # 阶段 3：link_health / topology_state 双订阅开关。
        # 注意必须放在 safety_yaml **之后**：ROS 参数文件的优先级低于
        # 后出现的字典，否则 yaml 里的 proto_dual_subscribe 会覆盖 launch 参数。
        parameters=[safety_yaml, {'proto_dual_subscribe': proto_dual_subscribe}],
        **supervision(),
    )

    chassis_driver_node = Node(
        package='amr_dispatcher_ros',
        executable='chassis_driver_node',
        name='chassis_driver_node',
        output='screen',
        condition=single_vehicle_only,
        parameters=[
            chassis_yaml,
            {
                # LaunchConfiguration 产出字符串，需显式声明为 double 以免参数类型不匹配；
                # serial_device 同理保留 CLI 覆盖位。
                'serial_device': serial_device,
                'health_rate_hz': ParameterValue(health_rate_hz, value_type=float),
                # T5：协议/波特率覆盖位。不传时与 chassis.yaml 的默认值一致。
                'protocol': ParameterValue(chassis_protocol, value_type=str),
                'serial_baud': ParameterValue(chassis_baud, value_type=int),
            },
        ],
        **supervision(),
    )

    path_tracker_node = Node(
        package='amr_dispatcher_ros',
        executable='path_tracker_node',
        name='path_tracker_node',
        output='screen',
        condition=single_vehicle_only,
        parameters=[{
            'controller_type': 'pure_pursuit',
            'lookahead_distance': 0.6,
            'target_speed': 0.5,
            'control_rate_hz': 20.0,
            # T4：终点容差参数化（默认与控制器内置值一致）
            'goal_tolerance': ParameterValue(goal_tolerance, value_type=float),
            # T6：与静态 map→odom 同源——两者必须一致，否则车在 RViz 里
            # 显示的位置和 tracker 用于控制的位姿会对不上。
            'odom_origin_x': ParameterValue(odom_origin_x, value_type=float),
            'odom_origin_y': ParameterValue(odom_origin_y, value_type=float),
            'odom_origin_yaw': ParameterValue(odom_origin_yaw, value_type=float),
            # 阶段 2 双发：/tracking/error_proto（20Hz，全项目最热的双发点）
            'proto_dual_publish': True,
        }],
        **supervision(),
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
            # 阶段 3：/tracking/error 双订阅（fallback 模式的到达信号）。
            # Nav2 模式下本订阅不参与逐点推进（由 navigate_to_pose 结果驱动），
            # 因此该开关只在 use_nav2:=false 时有实际作用。
            'proto_dual_subscribe': proto_dual_subscribe,
        }],
        **supervision(),
    )

    rest_gateway_parameters = []
    if os.path.exists(rest_gateway_yaml):
        rest_gateway_parameters.append(rest_gateway_yaml)
    # 阶段 3：四条业务话题（state / safety / link_health / events）双订阅。
    # 必须放在 rest_gateway_yaml 之后，理由同 safety_gate_node：后出现的
    # 字典优先级更高，否则 yaml 里的 proto_dual_subscribe 会盖掉 launch 参数。
    rest_gateway_parameters.append({'port': 8080})
    rest_gateway_parameters.append({'proto_dual_subscribe': proto_dual_subscribe})

    rest_gateway_node = Node(
        package='amr_dispatcher_tools',
        executable='rest_gateway',
        name='rest_gateway_node',
        output='screen',
        parameters=rest_gateway_parameters,
        **supervision(),
    )

    # ---- T6：静态 map→odom ----
    # chassis_driver 只发 odom→base_link；path_tracker 在内部把 odom 位姿按
    # odom_origin_* 平移进 map 帧，但 TF 树上从来没有 map→odom。
    # 后果是 RViz/Foxglove 里车永远停在地图原点附近，和站点坐标对不上。
    # 这里补上这条静态边，参数与 path_tracker 完全同源——可视化和控制用同一套数学。
    # 注意：这是**静态**外参，适用于"车每次都在同一个固定位置上线"的台架；
    # 真机若需要靠定位系统动态修正 map→odom，应改由定位节点发布并由
    # publish_map_to_odom:=false 关掉这一路，否则两条边会打架。
    # 注意 2：**只在单车模式下发布**。多车时每台车的 `odom` 帧是 `{vid}/odom`，
    # 且每台车的落地位置各不相同——一套 odom_origin_* 描述不了 N 台车的摆放，
    # 硬发一条 map→odom 只会让所有车在 RViz 里叠在同一个点上。多车要可视化，
    # 应由定位/地图服务提供 per-vehicle 的 map→{vid}/odom。
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        output='screen',
        condition=IfCondition(PythonExpression([
            "str('", publish_map_to_odom, "').lower() in ('true','1','yes','on')",
            " and str('", vehicle_ids, "') == ''",
        ])),
        arguments=[
            '--x', odom_origin_x,
            '--y', odom_origin_y,
            '--z', '0.0',
            '--yaw', odom_origin_yaw,
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
        **supervision(),
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
            # 阶段 3：/dispatcher/topology_state 双订阅
            'proto_dual_subscribe': proto_dual_subscribe,
        }],
        **supervision(),
    )

    lifecycle_coordinator_node = Node(
        package='amr_dispatcher_ros',
        executable='lifecycle_coordinator_node',
        name='lifecycle_coordinator_node',
        output='screen',
        parameters=[{
            'auto_bringup': True,
            'check_period_ms': 500,
            # 阶段 3：/safety/state 双订阅。安全状态陈旧判据（safety_stale_sec）
            # 不受影响——它看的是「最近一次收到消息」的时间，与传输层无关。
            'proto_dual_subscribe': proto_dual_subscribe,
        }],
        **supervision(),
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
        # 重生后的 map_server 同样回到 unconfigured；下方的 CONFIGURE/ACTIVATE
        # 只在首启触发一次。地图链路属于可选功能，重生后不自动提拉（保持现状），
        # 人工 `ros2 lifecycle set /map_server configure/activate` 即可，写在此处备查。
        **supervision(),
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
        **supervision(),
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
        **supervision(),
    )


    # ---- P0.6 / A4：多车并发（local_sim 后端）逐车节点 ----
    # 每台车一组 path_tracker / safety_gate / chassis_driver：
    #   * namespace = vehicle_id  -> 节点名唯一（/amr_01/safety_gate_node）
    #   * vehicle_id 参数         -> 话题按后缀派生（/plan/amr_01、/odom/amr_01、
    #                                /cmd_vel_safe/amr_01 …），绝对名不受 namespace 影响
    #   * TF frame 唯一化        -> {vid}/odom、{vid}/base_link
    # 兼容性红线：vehicle_ids 为空时，本函数不产生任何节点，单车路径逐字不变。
    def _multi_vehicle_nodes(context):
        raw_ids = launch_config_value(context, 'vehicle_ids')
        ids = [t.strip() for t in raw_ids.split(',') if t.strip()]
        if not ids:
            return []
        raw_serials = launch_config_value(context, 'serial_devices')
        serials = [t.strip() for t in raw_serials.split(',') if t.strip()]
        default_serial = launch_config_value(context, 'serial_device')
        health = launch_config_value(context, 'health_rate_hz')
        protocol = launch_config_value(context, 'chassis_protocol')
        baud = launch_config_value(context, 'chassis_baud')
        # T4：多车与单车用同一个容差来源，否则同一份配置在两种模式下行为不同。
        tolerance = launch_config_value(context, 'goal_tolerance')
        nodes = []
        for idx, vid in enumerate(ids):
            serial = serials[idx] if idx < len(serials) else default_serial
            ns = vid
            nodes.append(Node(
                package='amr_dispatcher_ros',
                executable='path_tracker_node',
                name='path_tracker_node',
                namespace=ns,
                output='screen',
                parameters=[{
                    'vehicle_id': vid,
                    'controller_type': 'pure_pursuit',
                    'lookahead_distance': 0.6,
                    'target_speed': 0.5,
                    'control_rate_hz': 20.0,
                    # T4：与单车模式同一来源
                    'goal_tolerance': float(tolerance),
                    'proto_dual_publish': True,
                }],
                **supervision(),
            ))
            nodes.append(Node(
                package='amr_dispatcher_ros',
                executable='safety_gate_node',
                name='safety_gate_node',
                namespace=ns,
                output='screen',
                parameters=[
                    safety_yaml,
                    {'vehicle_id': vid,
                     'proto_dual_subscribe': proto_dual_subscribe},
                ],
                **supervision(),
            ))
            nodes.append(Node(
                package='amr_dispatcher_ros',
                executable='chassis_driver_node',
                name='chassis_driver_node',
                namespace=ns,
                output='screen',
                parameters=[
                    chassis_yaml,
                    {
                        'vehicle_id': vid,
                        'serial_device': serial,
                        # OpaqueFunction 里 launch_config_value 已经求值成字符串，
                        # 不能再包 ParameterValue(float)——那要求传入真正的 float，
                        # 会抛 "value='5.0' is not an instance of <class 'float'>"，
                        # 整个 launch 直接退出（多车模式此前从未成功启动过）。
                        'health_rate_hz': float(health),
                        # T5：协议/波特率与单车模式同源
                        'protocol': protocol,
                        'serial_baud': int(baud),
                    },
                ],
                **supervision(),
            ))
        return nodes

    multi_vehicle_nodes = OpaqueFunction(function=_multi_vehicle_nodes)

    return LaunchDescription(declared_arguments + [
        dispatcher_node,
        configure_event,
        activate_event,
        safety_gate_node,
        chassis_driver_node,
        path_tracker_node,
        map_to_odom_tf,
        multi_vehicle_nodes,
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
