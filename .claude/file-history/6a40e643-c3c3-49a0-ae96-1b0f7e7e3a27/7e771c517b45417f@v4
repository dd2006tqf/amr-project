# AMR Dispatcher 演进路线图 (Extension Roadmap)

## Phase 1: 当前已落地 (Core Baseline & ROS 2 Adaptation - Complete)
- [x] 纯 C++17/20 核心库解耦，脱离 ROS 独立编译构建与 120 个单测验证
- [x] Mick 二进制与文本双流协议编解码、粘包/半包流式解析
- [x] Pure Pursuit 与 Stanley 循迹控制器纯类化
- [x] 可插拔调度比较器策略模式（PriorityFIFO / EDF / SJF / WeightedScore）
- [x] 交通管制拓扑原子锁（边与节点互斥表）
- [x] 任务死锁检测（长持资源/无进展）与自愈恢复动作决策
- [x] 硬件通信多级降级控制器（Serial -> UDP -> Mock）与指数退避重连
- [x] 多源安全仲裁门控（CmdVelGate）与节点心跳软看门狗
- [x] 强类型 YAML 配置加载器、Schema 校验与文件变动热重载
- [x] 异步低阻塞日志器与同步双实现
- [x] ROS 2 Lifecycle 调度节点、底盘驱动节点、安全门节点、循迹节点
- [x] 基于 hardware_interface::SystemInterface 的 ros2_control 硬件插件
- [x] 交互式 CLI 诊断客户端与 RESTful HTTP 提单网关（带幂等性校验）

## Phase 2: 工业级与高可靠增强 (Industrial Reliability - 2026 Q4)
- [x] **全局地图拓扑规划器整合 (Complete)**：
  - 支持官方 ROS 2 `nav2_map_server` 格式地图包 (yaml/pgm) 导入与解析
  - 纯 C++ 核心安全空间圆形核腐蚀、未知区封锁、Zhang-Suen 细化骨架提取与受保护毛刺剪枝
  - 自动提取带真实 `polyline` 几何折线的路网，站点正交投影拆边接入拓扑
  - `SkeletonPlanner` 真实折线 0.1m 等距插值规划器，彻底解决走廊拐弯直线穿墙隐患
  - 全路线原子排他锁与 `map_hash` 强一致性版本门禁
- [x] **VDA 5050 协议原生对接 (Complete)**：
  - 支持 VDA 5050 v2.0+ 标准 JSON MQTT 订单与状态报文
  - 将 VDA 5050 `order` 映射为 `MissionQueue` 中的任务节点与时空轨迹
  - dispatcher_node 直发 `/fleet/{vid}/send_order`（orderId=mission_id），
    gateway 网关双向桥接 MQTT；Task 5 逐段路权动态释放 (Spec 5.3) +
    断链锁保留与调度挂起守卫 (Spec 5.2)；
    操作员强清 `/dispatcher/force_clear_vda5050_link` 闭合 Spec 5.2 人工强解半边
    （释锁 + 收口在途任务 + **置隔离**而非摘车；在线车拒绝误杀；需 operator_id
    + reason + confirm_vehicle_id 审计三件套）；恢复走
    `/dispatcher/restore_vda5050_link`（需操作员声明已核实车辆物理位置）；
    端到端验收 `./scripts/check_vda5050_e2e.sh`（9 阶段全 PASS）
- [x] **多车车队化并发 (Multi-Vehicle Fleet Concurrency - Complete)**：
  - **执行后端分离**：`local_sim`（本地 `/plan/{vid}` 运动链）与 `vda5050`
    （外部标准协议车，只收 MQTT order、完成判定为 `lastNodeId` 推进）两套后端
    **不共用运动控制链**，也**不互相冒充**（评审否决项）
  - **per-vehicle 话题与 TF 隔离**：`/plan/{vid}`、`/odom/{vid}`、
    `/tracking/error/{vid}`、`/cmd_vel_raw/{vid}`、`/cmd_vel_safe/{vid}` +
    `{vid}/odom` / `{vid}/base_link`；急停/碰撞仍为全局安全信号
    （兼容红线：无受管车清单时话题名逐字不变）
  - **选车三态语义**：`kAssigned` / `kDeferred`（暂时不可用 -> 留队）/
    `kIndeterminate`（状态不确定 -> 暂停自动分配）/ `kRejected`（永久不满足 -> 判负）；
    显式指定车辆（`assigned_vehicle_id`）**不豁免**任何硬约束
  - **队首阻塞规避**：一次 tick 扫描前 N 个排队任务，派第一个可派发的
  - **人工强清改隔离语义**：强清不摘车，隔离车不再参与自动分配但仍可见可审计
  - 端到端验收：双 VDA 5050 车并发（Stage 7，独立域、无 tracker 冒充）
    与双本地模拟车并发（Stage 8，各一套 tracker/gate/chassis、两条互不相交的通道）
  - **Web 控制台浏览器端验收**：`scripts/check_web_console_e2e.sh` 用 headless Chrome
    + CDP 真实点击按钮，断言在线车强清被**服务端**拒绝、缺审计字段/未核实位置
    时**根本不发请求**、离线车强清后进入隔离态、显式恢复后解除隔离
- [x] **时空窗交通协调 (Spatio-Temporal Reservation - 一期完成)**：
  - 逐资源区间（边=通道 / 节点=路口）进出时刻，占用时长含加减速、转弯附加时间
    与车辆尺寸安全余量；最小安全间隔 `min_headway_sec`
  - **全有或全无**提交：整条路径预约全部成功才落表，任一失败整体回滚
  - **窗口到期 ≠ 资源已释放**：物理释放由车辆确认驶离（`lastNodeId` 推进）驱动；
    执行偏离靠延期扩窗（`ExtendTimeSpace`）
  - `traffic_mode: static | time_space`，**默认 static**（现网行为不变）

## Phase 2.5: 企业级加固 (Enterprise Hardening - 2026 Q4, 进行中)
- [x] **REST 网关健壮化 (Complete)**：
  - 已复现的致命缺陷全部修复：畸形 `Content-Length` / 非数字 `prio` 曾让
    网关 `std::terminate`；静默连接曾把单线程 accept 卡死到全线不可用
  - 异常兜底（任何处理异常 -> `400`，不再 terminate）、5s 收发超时、
    请求体 1 MiB / 头部 8 KiB 上限、分块传输拒绝、精确路由
  - 有界工作线程池（4 worker / 32 待处理），过载快速回 `503`
  - 静态 Bearer Token（`auth_token`，**默认关闭**，常量时间比较）与
    可配 CORS 来源（默认不下发 CORS 头）
  - 幂等键改为"后端确认受理后才记账"，`503/504` 回滚允许重试
  - 验收：`scripts/check_rest_gateway_hardening.sh`（独立域 86，21 条断言）
  - 单测基建：tools 包 57 例（http_limits / bearer_auth / idempotency_store /
    json_escape），容器内 `ctest --test-dir build/amr_dispatcher_tools`
- [x] **状态持久化与审计落盘 (Complete)**：
  - `state_snapshot_file`：排队任务（含优先级/序号/截止期/绑车）、暂停订单、
    在途任务、隔离车辆、`next_sequence` 游标 -> 原子 YAML 快照
    （`tmp` -> `flush` 校验 -> 回读校验 -> `rename`）；变更按防抖窗口合并写盘
  - 恢复语义：文件损坏/版本不认识 -> 拒绝恢复并保留现场；`map_hash` 与当前
    拓扑不一致 -> **拒绝恢复**（不拿旧拓扑派发）；在途任务**重新入队**
    （重启后没有车在跑它们）；暂停订单先不入队、标记生效后补回；游标不回退
  - `audit_log_file`：强清 / 解隔离 / 恢复决策写成 JSONL（operator_id、
    reason、released_resources），O_APPEND 单行追加
  - 幂等键可落盘（`idempotency_store_file`），跨重启去重
  - **默认全部关闭**（参数留空 = 现网行为逐字不变）
  - 验收：`scripts/check_state_persistence_e2e.sh`（独立域 87）
- [x] **dispatcher_node 分层重构 (Complete)**：
  - 派发主链从"单个 426 行 `ScheduleTick` 内联决策 + 副作用"改为
    "采集 -> core 纯函数出有序 step -> 按序落地"：
    - `workflow/dispatch_gate_planning.hpp` `EvaluateDispatchGates`
      （队列空 / 容量 / 空闲槽位 / 地图版本；**判据优先顺序**与
      "前序门禁提前返回不覆盖 `map_version_valid_`"由单测锁定）
    - `workflow/dispatch_endpoint_resolution.hpp` `ResolveDispatchEndpoints`
      （目录站点 / 车辆当前位姿 / 失败文案 `unknown dropoff station: <id>`）
    - `workflow/dispatch_tick_planning.hpp` `PlanDispatchTick`
      （三态处置、队首阻塞规避、一 tick 一派、两类拒绝的记账/告警先后；
      预留经 `try_reserve` 回调注入，纯函数只出**有序** step 列表）
    - `fleet/assignment_dispatch_action.hpp`、`fleet/vehicle_assignment_request_builder.hpp`、
      `fleet/vda5050_vehicle_binding.hpp`（选车动作映射、选车请求构建、
      订单绑车与 round-robin 游标）
    - `dispatcher/deadlock_snapshot_assembly.hpp`、`workflow/fleet_link_guard.hpp`、
      `fleet/vda5050_link_state.hpp`（快照组包、链路守卫、链路纯数据）
  - ROS 侧 `ScheduleTick` 拆为 `ScheduleGatesPass` / `SelectReadyCandidate` /
    `ExecuteDispatch` 三段；`Vda5050Link` 拆成订阅句柄表与纯数据表
  - 兼容红线逐条保持：legacy 单车不经选车、启动请求消费点不变、
    `candidates.empty()` 分支仍不 rearm、日志文案与 throttle 窗口逐字不变、
    守卫永不释放锁
  - 验收：core 830 例（新增 6 个 suite 约 60 例）+ ROS 25 例、三套 e2e 与
    `check_all.sh` 全 PASS
  - 说明：原计划用 `dispatch_trace_file` 新旧路径 diff 做机械防线；实际改为
    "纯函数单测锁定 step 顺序 + 同一套 e2e 全量回归"，未引入该调试参数

- [x] **拓扑级静态绕行与在途交通协调 (Phase 7 Complete)**：
  - [x] `catalog::FindRoutePathAvoiding`：与 `FindRoutePath` 共用同一个 Dijkstra，
    禁用键 = 路权资源键（`route_edge:<lo>__<hi>`），保证"规划时绕开的边"与
    "预约时锁住的边"是同一套资源；`FindRoutePathAvoidingTest` 10 例。
  - [x] **运行时交通协调状态模型 (P0)**：定义 `TrafficCoordinationState`（RUNNING / BLOCKED /
    REPLANNING / WAITING_FOR_RESOURCE / RECOVERY_FAILED），并在快照组包中引入
    `in_flight_pending_wait`，使在途等待无缝注入 WFG 图论环检测；单测 3 例。
  - [x] **在途安全重规划与两阶段资源置换 (P1)**：实现 `PlanInFlightReroute` 纯函数；
    从当前安全停靠锚点重新规划绕行替代路线，严格执行“先申请新路权，成功后再释放旧未来锁，
    当前节点锁绝不提前释放”的安全事务铁律；单测 3 例。
  - [x] **死锁分级恢复协调器 (P2)**：实现 `PlanDeadlockRecovery` 纯函数；
    优先选取死锁环中具有替代路线的较低优先级车辆执行拓扑绕行破环，无解时安全驻车等待；
    接入 `DeadlockTick`，实现 local_sim 与 VDA 5050 航线/订单更新发布；单测 2 例。
  - [x] **全流程矩阵化验证 (P3)**：新增 `scripts/check_traffic_coordination_e2e.sh`
    端到端集成测试，测试矩阵扩充至 1012 例，全部通过。
  - [x] **高层规划器插件 (CBS / 冲突搜索与 PBS 降级，Phase 9 & 10 Complete)**：
    低层带原地等待（`Wait In Place`）时空 $A^*$ + 高层冲突树（CBS）搜索，
    消除顶点重叠与通道对向冲突；引入优先级规划（PBS）、终点驻留保护（Goal-Hold）
    与自适应门面（`SolveMAPF`，小规模求最优，超限或大规模自动降级至 PBS）；
    `CBSTrafficCoordinator` 统一注入 `MotionTimingConfig` 膨胀量（起停/转弯/安全余量/min_headway），
    使时空窗口与 `time_space` 口径严格对齐；`check_cbs_coordination_e2e.sh` 5 阶段验收通过。

- [x] **CI 门禁 + L2 契约真断言 + `/metrics` (Complete)**：
  - CI 新增 `e2e-fleet` job：`check_map_ingest_e2e.sh` 与 `check_vda5050_e2e.sh`
    在 `ros:jazzy-ros-base` 容器内以 `AMR_E2E_INNER=1` 直接跑（脚本自带内层守卫，
    不需要 DinD）；失败时上传 `/tmp/amr_*` 现场日志。web 控制台 e2e 需要宿主机
    Chrome + 独立容器，**如实标注**为手动/nightly，不进常规门禁
  - `build-ros2-jazzy` 的单测从"只看 core + bt"改为遍历 core / bt / tools / ros
    四个包——此前网关的四组纯逻辑与 L2 契约在 CI 里从未被执行
  - L2 契约：`test/l2_launch_contracts/test_lifecycle_contracts.py` 从
    `assertTrue(True)` 改为 launch_testing 真断言（真起 coordinator + dispatcher，
    断言 `/system/ready` / `/system/healthy` 可调用、dispatcher 未激活时
    `ready=false` 且 `state=INITIALIZING`、`/system/ready_flag` 的
    transient_local 当前值为 false），已注册进 `add_launch_test`
  - `/metrics`：`GET /metrics` 输出 Prometheus 文本格式（`metrics_token` 独立口令，
    空则回退 `auth_token`）。渲染逻辑在 `tools/metrics_format.hpp`（12 例单测锁定
    转义/数值格式/空族省略），`scripts/check_metrics_e2e.sh` 对真实进程断言格式
    合法性与口令隔离

- [x] **批次 6：功能与接口补齐 (Batch 6 Complete)**：
  - [x] **选车策略可插拔**：定义 `VehicleSelectionStrategy`（`nearest` / `battery_preferred`）；
    `battery_preferred` 基于 50% 门限平滑软打折（最大 25% 惩罚），严格以 `vehicle_id`
    字典序破同分；`RankVehiclesForSelection` 单测 12 例。
  - [x] **低电量自动充电编排**：`PlanAutoCharging` 纯函数；涵盖机会充电与临界电量优先级分级、
    车级去重（`has_pending_charge_mission`）、离线/忙碌/隔离保护及充电桩有效性检查；
    接入 `ScheduleGatesPass` tick 驱动，单测 11 例。
  - [x] **订单批处理 / 拼单**：`PlanOrderBatchBundling` 纯函数；支持同起止站批量订单的
    Exact Match 归集，以及首尾相连站点的多 leg 路线串联（Route Chaining）；
    防回环检测；单测 6 例。
  - [x] **上游契约字段透传与整单取消**：REST 网关补齐透传 `order_id`、`deadline_unix_ms`、
    `expected_seconds`、`type`；`/api/workflows/cancel` 支持 `order_id` 整单取消。
  - [x] **物理设施适配器抽象**：抽象 `IFacilityAdapter` 硬件控制接口与 `MockFacilityAdapter`
    内存实现，统一门/电梯等现场设备的动作指令与互斥锁语义；单测 4 例。
  - [x] **订单载荷 Schema 校验**：`PayloadSchema` 规则校验引擎，覆盖字段类型约束、必填约束、
    枚举范围与数值极值限制；单测 5 例。

## Phase 3: 多机集群与智能化协同 (Fleet Coordination & AI - 2027 Q1)
- [ ] **分布式对等协商 (Peer-to-Peer Conflict Resolution)**：
  - 当中心调度离线或网络分区时，机器人之间通过局域网广播进行右侧让行或优先级协商
  - 现状：中心调度不可用时**保守派发**（不自动放行需协商的冲突），
    分布式协商落地前不引入自动让行
- [ ] **自适应速度平滑曲线**：
  - 循迹控制器集成 S 型加减速（Jerks/Acc 约束）及根据路面曲率的动态前瞻降速
- [ ] **能耗感知调度策略 (Battery-Aware Fleet Dispatching)**：
  - 引入 `EnergyOptimalComparator`，结合站点几何距离与电池剩余电量自动插入充电机动任务
  - 现状：选车已支持 `min_battery_percent` 硬阈值过滤（电量低于阈值不合格），
    但**尚不**自动安排充电机动任务
