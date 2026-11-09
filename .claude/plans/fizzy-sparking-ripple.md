# Phase 11 — 生产可观测性补洞 + 在线拓扑热生效

## Context

现网 `amr_dispatcher` 容器显示 `Up 8 days`，但 `dispatcher_node` 已死 8 天、安全门锁存 `CRITICAL` 急停，
`rest_gateway` 对任何请求仍回 200。根因不是某个 bug，而是三个结构性缺口：

1. **没有进程守护**：`launch/full_system.launch.py` 里 11 个节点 + 逐车 3 类节点全部没有 `respawn`。
   容器 PID 1 是 `ros2 launch`，子进程死了 PID 1 不退，所以 `restart: unless-stopped` 永远看不见。
2. **没有 HTTP 层探针**：`/api/system/ready` 只是把问题转述给协调器，没有 liveness/readiness 之分，
   且**没人定期去问**。另外 `lifecycle_coordinator_node` 的 `safety_healthy_` 只记值不记时间戳
   （`lifecycle_coordinator_node.cpp:43-46`）——安全门发布一次后死掉，`ready` 会永远为 `true`。这是同一个病根上的
   第二个真实缺陷，本阶段一并修掉。
3. **没有主动告警**：`/metrics` 只有被动抓取，长时间 `CRITICAL` 没有任何推送通道。

第二个缺口来自现场实施：地图/站点只能靠改文件 + mtime 热重载（`StationCatalogReloader`），
现场要设"临时施工禁行区 / 单向通行 / 新增临时站点"就得进容器 `vim stations.yaml`，
既没有连通性校验，也没有对在途车辆的安全仲裁；而 `map_ingest` / `/api/v1/map/upload`
会整份覆盖该文件，手改的施工区会被静默抹掉。

**交付**：A 探针与守护 · B 主动告警与新指标 · C 核心库拓扑叠加层纯函数 · D dispatcher 两阶段生效 ·
E 网关拓扑端点 · F 控制台面板 · G 测试与文档。

**红线**：测试只用域 92/93/94、端口 18100/18101/18111；生产 42/8080/1883 绝不触碰；
legacy 单车话题与既有日志文案逐字不变；所有新参数默认关闭（空/`false`）时现网行为逐字节不变。

---

## A. 进程守护 + 探针

### A1. 全节点 respawn（`launch/full_system.launch.py`）

- 新 launch 参数 `respawn_enabled`（默认 `true`）、`respawn_delay`（默认 `2.0`）。
  **已在容器内实测** `ExecuteProcess` 接受 `respawn` / `respawn_delay` / `respawn_max_retries`
  三个 kwarg（`LifecycleNode` 经 `**kwargs` 透传）；`respawn_max_retries` 留默认无限重试，
  崩溃循环靠日志 + `amr_ready` 指标暴露，不设人工上限。
- 作用于全部节点：`dispatcher_node`(LifecycleNode)、`safety_gate_node`、`chassis_driver_node`、
  `path_tracker_node`、`navigate_sequence_bridge_node`、`rest_gateway_node`、
  `dispatcher_visualizer_node`、`lifecycle_coordinator_node`、`map_server_node`、`map_ingest_node`、
  `vda5050_gateway_node`，以及 `_multi_vehicle_nodes()` 内逐车生成的
  tracker/gate/chassis（`launch/full_system.launch.py:403-442`）。
- **重生后如何被重新提拉**（关键，已核实）：launch 的 `EmitEvent(CONFIGURE)` 与
  `OnStateTransition(inactive→ACTIVATE)` 只在启动时触发一次。重生的 `dispatcher_node`
  回到 `unconfigured`，由 `lifecycle_coordinator_node` 的 500ms `CoordinatorTick`
  （`lifecycle_coordinator_node.cpp:99-108`，`auto_bringup=true`）重新
  `unconfigured→inactive→active`。**无需新代码**，但部署文档必须写明
  "dispatcher 自愈依赖协调器存活；协调器自己也有 respawn"。
- 服务注册在 `on_configure`（`dispatcher_node.cpp:362-425`）而非构造函数，所以重生后
  重新 configure 就会重建全部服务与定时器——这条已成立，但要作为 e2e 断言锁定。
- 副作用如实说明：respawn 是**冷重启**，`on_configure` 会从 `state_snapshot_file` 恢复队列并把
  在途任务重新入队（`:180-190`），车辆可能收到第二份 `/plan`/order。这是既有的重启语义，
  只是现在无需人工触发。

### A2. `/healthz` 与 `/readyz`（`rest_gateway`）

**探针专用快车道**（这是 A2 最重要的结构改动）：现在所有请求都进 `pending_` 队列由 4 个
worker 处理，每个 handler 都可能 `wait_for_service(1s)` / `fut.wait_for(2s)`。一旦 DDS 卡死，
4 个 worker 全被占住，探针永远得不到应答——正是我们要检测的失效模式本身。因此：

- 把 `ReadRequest` 拆成 `ReadRequestHead`（读到 `\r\n\r\n`，受 `kMaxHeaderBytes` 约束）+
  `ReadRequestBody`（其余）。`accept` 线程读头后**同步**判断路径：
  `/healthz`、`/readyz` 直接在主线程内 `HandleProbeInline()` 应答并关闭；其余请求带
  `prebuffered` 前缀入队（`std::deque<QueuedConn{fd, prebuffered}>` 取代 `std::deque<int>`）。
- 探针处理**不调用任何 ROS 服务**，只读本地缓存的消息与到达时刻（`data_mutex_` 只用于
  拷贝 4 个小结构 + 4 个时间戳，微秒级，不持锁做格式化）。
- `GET /healthz`：恒定 `200 {"status":"ok","uptime_sec":N}`，无参数、无依赖。
  语义只有"网关进程还能处理 HTTP"——DDS 卡死时它仍应答，这是 liveness 的定义。
- `GET /readyz`：逐组件聚合，`200` 或 `503`：

| 组件 | 数据源 | ok 判据 | 阈值参数（默认） |
|---|---|---|---|
| `dispatcher` | `/dispatcher/state`（2Hz，仅 active 时发布） | 新鲜 | `readyz_dispatcher_state_stale_sec` 3.0 |
| `safety` | `/safety/state`（空闲 10Hz / 有指令时 50Hz） | 新鲜 且 `motion_allowed` 且 severity≠CRITICAL | `readyz_safety_stale_sec` 2.0 |
| `chassis` | `/chassis/link_health` | 新鲜 且 `is_healthy` | `readyz_chassis_stale_sec` 5.0 |
| `coordinator` | `/system/ready_flag`（transient_local, 2Hz） | 新鲜 且 `true` | `readyz_coordinator_stale_sec` 3.0 |

  `chassis` 用 `readyz_require_chassis`（默认 `true`）控制；多车模式下 `/chassis/link_health`
  是 per-vehicle 话题、网关订阅不到 → 该组件标 `"monitored": false` 且**不**计入失败
  （诚实标注，不假装在监控）。
- 新增订阅 `/system/ready_flag`（QoS transient_local），与现有 4 个订阅同一模式
  （`rest_gateway.cpp:249-279`），同时补齐 `state_seen_` / `link_seen_` / 各 `last_*_arrival_`。
- 鉴权豁免：只暴露布尔量与秒数、不含业务数据，把 `/healthz`、`/readyz` 加进
  `include/amr_dispatcher_tools/bearer_auth.hpp::IsAuthExemptPath` 并扩充 `test_bearer_auth.cpp`。
- 纯逻辑抽到新头文件 `include/amr_dispatcher_tools/health_probe.hpp`
  （`ComponentProbe` / `ReadinessThresholds` / `ReadinessVerdict` / `EvaluateReadiness()` /
  `RenderReadinessJson()`），不读时钟、不碰 ROS，由 `test_health_probe.cpp` 锁死契约。
  规则：`ok = seen && age <= stale && semantic_ok`；`!seen` **判为不就绪**（fail closed）；
  `state` = 全新鲜且 ok → `READY`；任一 `!seen` → `INITIALIZING`（沿用协调器词汇）；否则 `DEGRADED`。
- 修协调器的时间戳缺陷（M1.3 评审项）：`lifecycle_coordinator_node` 记 `safety_last_seen_`，
  `EvaluateReadiness` 增加"安全状态新鲜度"判据（新参数 `safety_stale_sec`，默认 2.0）。
  这是 bug fix（只在安全状态确实陈旧时才判不就绪），PR 描述需显式声明。

### A3. 容器健康检查

- 新增 `docker/healthcheck.py`（仅 stdlib；`ros:jazzy-ros-base` 不保证有 `curl`）：
  `GET http://127.0.0.1:${AMR_REST_PORT:-8080}/readyz`，200 → 0，其余 → 1，任何异常 → 1。
- `docker/docker-compose.server.yml` 增 `healthcheck`（`interval 10s / timeout 5s / retries 3 /
  start_period 180s`，首启要 colcon 构建）。
- **必须写明政策**：HEALTHCHECK 只提供"可见性"与 `up --wait` 判定，**不触发自动重启**；
  自愈靠 A1 的节点 respawn。不要加 autoheal —— 因 `503` 重启会拆掉"健康但安全锁存"的栈并丢调度状态。

---

## B. 主动告警 + 新指标

### B1. 告警状态机（`include/amr_dispatcher_tools/alerting.hpp`，header-only，可单测）

`AlertEvent{type, severity, state(firing|resolved), dedup_key, summary, fired_at, resolved_at,
consecutive_observations, details_json}`、`AlertPolicy`、`AlertStateMachine::Observe(obs)`、
`RenderAlertPayload()`。语义（每条一个用例）：

- `readiness_not_ready`：连续 `N` 次不就绪才发（`alert_not_ready_threshold`，默认 3）；`min_interval_sec`
  内不重复；首次恢复发**恰好一条** `readiness_recovered`（不受节流），然后复位。
- `safety_critical`：`CRITICAL` 连续超过 `alert_safety_critical_sec`（默认 30）才发；恢复发 `safety_recovered`。
- `component_missing:<name>`：单组件连续 3 次陈旧。
- `topology_overlay_invalidated`：一次性 `info`，由 dispatcher 事件流触发（D 节），无去重状态。
- payload schema `amr.alert.v1`：
  `{schema,type,state,severity,dedup_key,summary,fired_at,resolved_at,consecutive_observations,`
  `instance:{host,pid,ros_domain_id,gateway_port},details:{...}}`。

### B2. 发送通道（`include/amr_dispatcher_tools/alert_webhook_client.hpp` + `rest_gateway.cpp`）

- 客户端只用裸 socket（与网关现有风格一致、零新依赖）：`ParseHttpUrl`（仅 `http://`，
  `https://` 启动时 WARN 并禁用，**诚实标注**不做 TLS）、`BuildWebhookPost`（纯字符串可断言）、
  `PostWebhook`（超时毫秒、响应体上限 4KiB 丢弃）。
- **不阻塞 worker**：`EnqueueAlert()` 在 worker/定时器里只持 `alert_mutex_` 入队 + `notify_one`，
  O(1) 无系统调用；**独立单线程** `AlertSenderLoop()` 是唯一做阻塞 POST 的地方；
  有界队列（`alert_queue_capacity` 默认 64）满时丢**最旧**并计数（旧的 "not ready" 不如新的有用）；
  析构时置 `stopping_` + notify + join（与 `workers_` 同一模式）。
- 生产者是 **ROS executor 上的 wall timer**（`alert_probe_period_ms` 默认 1000）：
  锁内快照探针状态 → `Observe()` → 入队。executor 永不阻塞在网络。
- `alert_webhook_url` 为空 → **不建 timer、不起线程**，进程/线程/话题与今天完全一致。
- 发送失败只计数不重试（`amr_alert_webhook_failures_total` 让告警通道本身可被抓取侧告警）。

### B3. 新 Prometheus 指标（`RenderMetrics()`）

标签基数必须有界（**绝不为边名打标签**）：

```
amr_ready                                       0/1（与 /readyz 同判据）
amr_ready_component{component="dispatcher|safety|chassis|coordinator"}   0/1   （最多 4 条）
amr_dispatcher_state_age_seconds                gauge（从未收到 = -1）
amr_safety_severity_level                       0=INFO 1=WARN 2=CRITICAL，-1=never seen
amr_safety_critical                             0/1
amr_topology_overlay_active                     0/1
amr_topology_blocked_edges / amr_topology_one_way_edges / amr_topology_overlay_revision
amr_alert_sent_total / amr_alert_webhook_failures_total / amr_alert_dropped_total   counter
```

`check_metrics_e2e.sh` 断言"无空指标族"，因此新族**必须无条件产生样本**（用 0 而不是省略）。
拓扑三项由告警 timer 每秒调一次 `GetTopologyOverlay`（500ms 超时）缓存，`/metrics` 与
`/readyz` 读缓存 —— 不每次抓取都打服务，也保证控制台与指标不会互相矛盾。

---

## C. 核心库：拓扑叠加层（零 ROS，宿主机 ctest）

新头/源（追加进 `src/amr_dispatcher_core/CMakeLists.txt` 的库源列表）：

- `catalog/topology_overlay.hpp/.cpp`：
  `TopologyOverlay{overlay_version, base_map_hash, revision, updated_at, blocked_edges[], one_way_edges[], added_markers[]}`、
  `ValidateTopologyOverlay`、`ApplyTopologyOverlay(base, overlay) -> StationCatalog`、
  `SerializeTopologyOverlay`（确定性：固定字段序 + 固定浮点格式，byte-stable 便于 diff/e2e）、
  `LoadTopologyOverlay`（沿用 `station_catalog.cpp` 的 `Trim`/`ValueAfterColon` 手写行式风格，
  未知章节严格失败）、`PersistTopologyOverlay`（tmp → fsync → 回读校验 → rename，旧文件留 `.bak`）、
  `InvalidateOverlayFile`（rename 到 `<path>.stale.<epoch>`）。
- `catalog/topology_connectivity.hpp/.cpp`：`AnalyzeTopologyConnectivity()` →
  `ReachabilityReport{all_stations_reachable, components, unreachable_stations, dead_end_road_nodes}`，
  Tarjan SCC（尊重 `bidirectional`）。规则：**所有站点必须互相可达**；路网节点可为死胡同（仅告警）。
- `workflow/topology_change_impact.hpp/.cpp`：`AnalyzeTopologyChangeImpact(request)` →
  逐车 `VehicleImpact{kind: kUnaffected|kRerouteReady|kOnBlockedEdge|kNoAlternative|kAlreadyAtGoal,
  new_required_locks, obsolete_locks, full_updated_path, detail}` + 汇总
  `{safe_to_apply, reroute_count, stranded_count, occupied_count, error_code}`；
  辅助 `RemainingPathTraversable(catalog, full_path, from_node)`。

**三条不可让渡的语义决策**：

1. **封路 = `enabled=false`；单向 = `bidirectional=false`**（就地改目录副本），
   **不是**维护一份"禁用键"运行态。理由：`FindRoutePathImpl` 已经同时尊重这两个字段
   （`station_catalog.cpp:511` 跳过 `!enabled`，`:530-533` 仅在 `bidirectional` 时反向松弛），
   而 `FindRoutePathAvoiding` 的键是**无序**的（`:453-455`），用它做单向会连合法方向一起封死。
   这样 `SkeletonPlanner` 折线展开、`EstimateRouteDistance`、CBS/时空窗全部自动一致，不用改 4 个调用点。
2. **单向反转必须交换 `from`/`to`**：若基线里该边存的是 `from=B,to=A`，要表达"只许 A→B"
   就必须交换端点，否则方向被静默反转。这是最容易写错的一处，必须有专门用例。
3. **被禁的边保留在 vector 里**（不 erase），只置 `enabled=false`，这样
   `ListStations`/`BuildStationRouteListProjection` 与控制台仍能看到"这条边存在但被封"。

`ApplyTopologyOverlay` 在空叠加层时**逐字节等于基线**（用例锁定），`map_hash`/`graph_version`
原样拷贝（叠加层不是新地图版本，这也是它能复用同一套状态恢复门禁的前提）。

---

## D. dispatcher：有效目录 + 两阶段生效

### D1. 有效目录抽象（唯一合并点）

```cpp
// 无叠加层时 return-by-reference 直接给 reloader 的目录（零拷贝，行为逐字不变）
const catalog::StationCatalog& EffectiveCatalog() const;
bool CatalogLoaded() const;   // = reloader_->loaded()
```

新增成员：`effective_catalog_`、`overlay_`、`overlay_active_`、`overlay_loaded_`、`overlay_deferred_`、
`topology_overlay_file_`（**参数默认 `""` = 功能关闭**；生产由 launch 注入**绝对路径**——
`HandleMapUpload` 写的是相对 `config/generated/stations.yaml`，沿用相对路径会踩同一个 cwd 陷阱）、
`topology_allow_edge_under_vehicle_`、`last_overlay_error_`。

**调用点迁移（必须穷尽，否则"目录说封了、派发仍按老路存档"）**：把
`catalog_reloader_->catalog()` 换成 `EffectiveCatalog()` 于所有**决定可达性**的读取处
（2 处路径规划、路由锁、端点解析、EstimateMissionCost、ListStations、死锁重规划、CBS、
自动充电、VDA 订单构建等，现计 23 处 `catalog()` / 13 处 `loaded()`）。
**不换**的（版本/身份比较必须用基线）：
- 状态快照恢复的 `map_hash` 比较（`:151-157`）—— 并且 `BuildStateSnapshot()` 的
  `snapshot.map_hash = base.map_hash`（`:4039`），否则每次改叠加层都会让下次重启丢队列；
- `RebuildPlanner()` 的初始目录（`:591`）改为 `EffectiveCatalog()`（规划器要看到叠加层），
  但 mtime 重载日志仍用基线；
- `map_version_valid_` 门禁保持比对**基线** hash（`expected_map_hash` 是地图版本，不是叠加层版本）。

`planner_->UpdateCatalog(EffectiveCatalog())` 必须在**三处**同步：`RebuildPlanner`、`reload_timer_`、
叠加层提交之后。`SkeletonPlanner` 持有目录副本，漏一处就是"目录封了但车照开"。

### D2. 启动加载与基线变更（`on_configure` / `reload_timer_`）

- `on_configure`：建 `catalog_reloader_` 后加载叠加层（**再**走状态恢复，使路径重算基于有效目录）。
  - 文件不存在 → INFO "no topology overlay"（不是错误）；
  - **文件损坏/校验失败 → 拒绝**：保留基线、`overlay_active_=false`、`RCLCPP_ERROR` + 审计 +
    发 `overlay_rejected` 事件，**不改名、不动原文件**（与既有"坏快照不清理现场"同一姿态）；
  - `base_map_hash != base.map_hash` → **失效**：rename 到 `.stale.<epoch>`、清内存叠加层、审计
    `topology_overlay_invalidated`、发事件（供告警）；
  - 成功 → `effective_catalog_ = ApplyTopologyOverlay(base, overlay_)` + `ReconcileResourceBlocksWithOverlay()`
    （否则重启后封锁不出现在 `list_traffic_reservations` 里，死锁检测丢一路输入）。
- `reload_timer_`（`:562-581`）在 `Tick()` 为 true 时：若 `overlay_deferred_` 则补加载（map_ingest 晚写出）；
  若 `overlay_active_ && base.map_hash` 变了则按上面失效；否则把叠加层**重新应用**到新基线并同步规划器。
  **绝不允许基线热重载静默丢掉叠加层。**

### D3. 提交管线（原子性）

`BuildTopologyCommitPlan()`（**零副作用**）→ `CommitTopologyPlan()`（唯一变更函数）。

`Build`：
1. `catalog_unavailable`（目录未加载）；
2. `map_hash_mismatch`（`expected_map_hash` 非空且不等基线 —— 乐观并发，防陈旧客户端把施工区套到新地图上）；
3. 盖 `base_map_hash` / `revision = overlay_.revision + 1` / `updated_at`，条目补 `operator_id/reason/created_at`；
4. **幂等**：规范化后（忽略 revision/时间戳）与当前一致 → `no_change`（200，**零写盘零计数**）；
5. `ValidateTopologyOverlay` → `overlay_invalid`；
6. `ApplyTopologyOverlay` + `AnalyzeTopologyConnectivity` → `would_disconnect_topology` + `unreachable_nodes`；
7. 从 `active_missions_` / `mission_route_paths_` / `vehicle_safe_nodes_`（缺则回落 `pickup_station`，
   与 `:2747-2752` 同构）/ `reservation_table_->Snapshot()` 过滤 owner 组 `InFlightMissionView`；
8. `AnalyzeTopologyChangeImpact` → `edge_occupied_in_flight` 或 `would_strand_vehicles`，**立即返回**；
9. 锁预检：所有 `new_required_locks` 若被非本任务、非"将被替换的 traffic_block"持有 → `lock_conflict`。
   单线程 `rclcpp::spin`（`dispatcher_node.hpp:396-399`）保证预检到提交之间状态不变 ——
   这一点要写成注释，因为未来切 `MultiThreadedExecutor` 会破坏它。

`Commit`（顺序即安全铁律）：

```
1) PersistTopologyOverlay(file, candidate)      // 写前提交点：失败 → overlay_persist_failed，内存零改动
2) Reserve(mid, new_required_locks)             // 预检保证成功
3) ReleaseResources(mid, obsolete_locks)        // 必须早于第 4 步
4) Block(route_edge key, "topology_overlay:"+reason)   // Block 在资源被持有时会失败，故必须在 3 之后
5) Unblock(已从叠加层移除的边)                  // 只解 traffic_block: 前缀
6) overlay_=candidate; effective_catalog_=new; overlay_active_=!empty
7) planner_->UpdateCatalog(EffectiveCatalog())
8) mission_route_paths_[mid]=full_updated_path; in_flight_pending_wait_.erase(mid);
   ApplyRerouteToVehicle(mid, impact)           // 从 DeadlockTick:2793-2849 抽出的发布段
9) AppendAudit("topology_hot_update"|"topology_block_edge", operator, reason, released_resources)
10) MissionEvent{event:"rerouted"} 逐车 + 一条 "topology_changed" 汇总
```

**写前持久化（第 1 步在任何内存变更之前）**：叠加层文件是恢复的唯一真相源。进程在第 1 步之后
任意时刻死掉，重启从冷启动回放即为目标态；死在第 1 步之前，哪里都没变。这样彻底免掉
"磁盘失败要回滚内存锁"这一最难写对的分支。残留窗口（文件新、内存旧，存活微秒级）在单线程
executor 下不可观测。

`ApplyRerouteToVehicle(mid, impact)` 从 `DeadlockTick` 抽出，**死锁路径的日志文案留在原位**
（`"Phase 7 Deadlock recovery: rerouting mission %s..."` 不移动），`MissionEvent` 由各自调用方发，
以保住两条路径各自的 `reason` 文本（兼容红线）。

**时空窗补充**：`ReleaseResources` 只处理静态锁（`traffic_reservation.hpp:59-60`），
`time_space`/`cbs` 模式下重规划还必须对每台受影响车调 `ReleaseTimeSpaceResources`，
否则旧窗口仍占着新路线的资源。

### D4. 在途车"正压在边上"的语义（安全重点）

`vehicle_safe_nodes_` 来自 VDA 5050 `lastNodeId`（`:3851`），只说明"最后确认到达的节点"，
车辆可能正驶在下一段上。因此：

- 若某车**剩余航段**里有边被整体禁用、且该边正是离开它当前安全节点的那条 → `kOnBlockedEdge`；
  默认 **409 `edge_occupied_in_flight`**（拒绝是最诚实的答复）；
- `allow_edge_under_vehicle=true`（参数 `topology_allow_edge_under_vehicle`，默认 false）时允许提交：
  语义为"对**新**规划立刻生效，物理通行等该车自己驶离"，此时 `Block` 该边会失败 →
  跳过并计入 `retained_edge_locks`（车持有的锁不能被抢），车释放后该边自然回到无锁，
  下一次同边提交才会真正落表；
- **单向反转不提供 `force`**：让在途车反向穿越没有安全解，一律 409。

**已知语义缺口（必须在 `docs/API.md` 端点旁写明）**：`route_edge` 锁是无序互斥，
**单向在预约层没有表达形式**——单向只由规划层（`bidirectional=false`）保证。
锁的职责是"两车不得同时占用该段"，与方向无关，因此可接受；但操作员会期望
`list_traffic_reservations` 能看到它，必须提前说清。

### D5. 新接口（`amr_dispatcher_interfaces`，加进 `CMakeLists.txt: srv_files`）

- `BlockEdge.srv`：`from_station, to_station, action(block|unblock), direction(both|forward|reverse),
  reason, operator_id, expected_map_hash, confirm` →
  `success, error_code, message, resource_id, impacted_vehicles[], impacted_missions[], rerouted, overlay_revision`
- `TopologyHotUpdate.srv`：`overlay_yaml, expected_map_hash, operator_id, reason, dry_run, confirm` →
  `success, error_code, message, unreachable_nodes[], impacted_vehicles[], impacted_missions[],
  reroute_failures[], catalog_map_hash, overlay_revision, file_written`
- `GetTopologyOverlay.srv`：`include_yaml` → `success, overlay_yaml, revision,
  blocked_edge_count, one_way_edge_count, marker_count, base_map_hash, effective_map_hash, message`

`error_code` 词表（网关状态码映射共用）：`ok, no_change, invalid_request, not_confirmed,
map_hash_mismatch, unknown_edge, would_disconnect_topology, edge_occupied_in_flight,
would_strand_vehicles, lock_conflict, catalog_unavailable, overlay_invalid,
overlay_persist_failed, internal_error`。

注册在 `on_configure`（紧邻 `block_route_srv_`，`:407`）：`/v2/block_edge`、
`/v2/topology_hot_update`、`/v2/get_topology_overlay`，并在 `on_cleanup`（`:643`）全部 reset。
旧的 `/v2/block_station_route` 处理器（`:1279-1292`）**逐字节保持不变** —— 新的是"目录感知封路"，
旧的是"仅预约表封路"，两者并存。

`DispatcherState.msg` **追加**（rosidl 兼容）`bool overlay_active`、`uint32 overlay_revision`、
`uint32 blocked_edge_count`，供控制台免服务调用展示。

---

## E. 网关拓扑端点（`rest_gateway.cpp`）

| 方法 | 路径 | 动作 |
|---|---|---|
| GET | `/api/v1/topology/overlay` | `GetTopologyOverlay`（`?yaml=1` 才回 YAML）；顺手刷新指标缓存 |
| POST | `/api/v1/topology/block_edge` | `BlockEdge{action=block}` |
| POST | `/api/v1/topology/unblock_edge` | `BlockEdge{action=unblock}` |
| POST | `/api/v1/topology/hot_update` | `TopologyHotUpdate` |
| POST | `/api/v1/topology/preview` | 同上强制 `dry_run=true` |

处理器沿用 `HandleForceClear`（`:652-695`）骨架：必填字段（`operator_id`/`reason`/`confirm`）→ 400；
`wait_for_service(1s)` → 503；`fut.wait_for(topology_op_timeout_ms` 默认 10000，因为 N 车重规划慢`)` → 504。
状态码映射：`ok|no_change`→200（`"status":"ok"|"noop"`）；`invalid_request|not_confirmed`→400；
`map_hash_mismatch|would_disconnect_topology|edge_occupied_in_flight|would_strand_vehicles|lock_conflict|overlay_invalid`→409；
`unknown_edge`→404；`catalog_unavailable`→503；`overlay_persist_failed|internal_error`→500。
响应体总带 `error_code`、`message`、`impacted_*`、`overlay_revision`，409 连通性问题额外带 `unreachable_nodes`。

新增 `config/rest_gateway.yaml`（网关参数集中化，`full_system.launch.py` 注入 + `port` 覆盖位），
全部新参数默认关闭/合理值。

---

## F. 控制台拓扑运维面板（`tools/operator_console.html`）

在地图导入区之后新增 `span-12` 区块：

- **就绪红灯表**：`GET /readyz` 四组件的 pill + 时延 + CRITICAL 持续秒数；独立 2s 轮询
  （`res.ok` 只决定颜色，503 的 body 仍要解析渲染）。
- **叠加层状态**：`revision`、blocked/one-way/marker 计数、基线 hash、`overlay_active` 徽标。
- **封路/解封表单**：from/to、方向（both/forward/reverse）、operator、reason、
  `expected_map_hash`（从状态预填）、confirm 勾选 + `window.confirm` 二次确认（与强清同款
  `:583-626`）。
- **全量预览**：`<textarea>` 预填当前叠加层 YAML，`预演` → `/preview`；
  **`应用` 按钮在"同一份文本预演成功"之前保持 disabled**（JS 对文本做 hash，内容变化即失效）——
  客户端防线，服务端仍独立复验。
- **影响面面板**：原样渲染 409 返回的 `impacted_vehicles`/`impacted_missions`/
  `unreachable_nodes`/`reroute_failures`（不做归并，操作员要看到被拒的确切原因）。
- 新增行用 `textContent` 赋值（现有 `fetchVehicleLinks` 用 `innerHTML` 插值服务端字符串，不要照抄扩大风险面）。

`scripts/web_console_check.mjs` 新增 phase（沿用现有 `check(name, ok, detail)` 契约）：
`topology_panel`（4 行组件 + READY/DEGRADED + revision）、
`topology_preview_reject`（断连叠加层 → `unreachable_nodes` 非空、应用按钮仍 disabled）、
`topology_block`（成功 → 计数 +1 → textarea 刷新）、
`readiness_degraded`（shell 侧 kill dispatcher → 该行转红且页面继续轮询，证明 503 body 被渲染）。

---

## G. 测试与验收

### G1. Core 单测（`ctest --test-dir build/core`，零 ROS）

注册 4 个新套件进 `UNIT_TESTS`：
- `test_topology_overlay.cpp`：`Apply_BlocksEdgeInBothDirections`、`Apply_OneWayKeepsForwardDirection`、
  **`Apply_OneWaySwapsReversedBaseOrientation`**、`Apply_AddsMarkerStation`、
  `Apply_EmptyOverlayIsByteIdenticalToBase`、`Apply_KeepsDisabledEdgeVisible`、
  `Validate_RejectsUnknownEndpoint|SelfLoop|DuplicateBlockedEdge|BlockAndOneWayConflict|NonFiniteMarker|DuplicateMarkerId`、
  `Validate_WarnsOnMapHashMismatch`、`Serialize_RoundTripsAllFields`、`Serialize_IsDeterministic`、
  `Load_RejectsCorruptOverlay|UnknownSection|MissingBaseHash`。
- `test_topology_connectivity.cpp`：`Analyze_ConnectedGraphIsReachable`、
  `Analyze_BlockedBridgeSplitsComponents`、`Analyze_ReportsUnreachableStationNames`、
  `Analyze_OneWayRingStillReachable`、`Analyze_DeadEndRoadNodeIsWarningOnly`。
- `test_topology_change_impact.cpp`：`Impact_NoInFlightVehiclesIsSafe`、
  `Impact_UnaffectedVehicleIsNoOp`（不产生锁置换）、`Impact_ReroutableVehicleProducesTwoPhaseLocks`、
  `Impact_VehicleOnBlockedEdgeIsRejected`、`Impact_NoAlternativeIsRejected`、
  `Impact_OneWayReversalForcesReroute`、`Impact_PreservesCurrentSafeNodeLock`、
  `Impact_AllowEdgeUnderVehicleOverrides`、`Impact_MixedFleetAnyStrandBlocksAll`。
- `test_topology_overlay_file.cpp`：`Persist_WritesBakAndReplacesAtomically`、
  `Persist_FailureLeavesOriginalIntact`、`Invalidate_RelocatesWithSuffix`、`Persist_RoundTripsThroughLoad`。

### G2. Tools 单测（CI 容器内 `ctest --test-dir build/amr_dispatcher_tools`）

`test_health_probe.cpp`（阈值边界 `age == stale` / `stale+1`、`never_seen` 判不就绪、
`monitored:false` 不判负、CRITICAL 语义、JSON 转义）；
`test_alerting.cpp`（连续 N 次才发、节流、恢复只发一条、队列满丢最旧、payload 合法、`https://` 被拒、
`ParseHttpUrl` 边界）；扩充 `test_bearer_auth.cpp`（`/healthz`、`/readyz` 豁免，`/api/v1/topology/*` 不豁免）。

### G3. e2e（域 92/93 与 18100/18101/18111 已核对未被占用）

- **`scripts/check_health_supervision_e2e.sh`**（域 92 / REST 18100 / webhook 收单 18111）：
  1. `/healthz` 200 且**不带** `Authorization`；`/readyz` 200 且含 dispatcher/safety/coordinator；
  2. `kill -9 dispatcher_node` → 3s 内 `/readyz` 503 且 `components[dispatcher].reason == "state_stale"`（不是 `never_seen`）；
  3. 5s 内 webhook 收到 `readiness_not_ready`；
  4. 25s 内新 PID 出现、`ros2 lifecycle get` 为 `active`（证明协调器重新提拉）、`/readyz` 回 200；
  5. 恰好一条 `readiness_recovered`；
  6. 发一次 `/safety/estop true` → 1s 内 `/readyz` 503、`/metrics` 里 `amr_safety_critical 1` 且
     `amr_safety_severity_level 2`；`alert_safety_critical_sec:=3` 后收到 `safety_critical`；解除后收到 `safety_recovered`；
  7. `/metrics` 新族全部有样本、无标签基数爆炸（断言不含边名）；`amr_alert_*` 计数有界。
- **`scripts/check_topology_hot_update_e2e.sh`**（域 93 / REST 18101）：
  1. 初始 `revision 0`；
  2. 断连叠加层 `preview` → **409 `would_disconnect_topology`** 且**叠加层文件不存在**（"拒绝时不写盘"红线）；
  3. 缺 `reason` → 400 且仍无文件；
  4. 合法封非桥边 → 200，文件存在，`revision 1`，`/metrics` `amr_topology_blocked_edges 1`；
  5. 重放同一请求 → 200 `"status":"noop"` 且 `revision` 不变（幂等）；
  6. 错误 `expected_map_hash` → 409 `map_hash_mismatch`，`revision` 不变；
  7. 提单走原最短路径 → 成功且存档路径已避开被封边；
  8. 在途车压在即将被封的边上 → 409 `edge_occupied_in_flight` + 车辆 id，且该车**没有**收到新 `/plan`；
  9. kill + respawn 后 `revision 1` 仍在（文件回放）；`list_traffic_reservations` 仍见 `traffic_block:` owner；
  10. 改 `stations.yaml` 的 `map_hash` 并等一拍 → 叠加层失效为 `revision 0`、出现 `.stale.*` 文件、
      `audit_log_file` 有 `topology_overlay_invalidated`、对应事件被记录；
  11. `unblock_edge` → 计数归零、`amr_topology_blocked_edges 0`、预约表不再有该 `traffic_block:` owner；
  12. legacy 红线：全程无叠加层时 `/v2/list_stations` 与改动前逐字一致。
- **`test/l2_launch_contracts/test_supervision_contracts.py`**（域 `AMR_L2_DOMAIN_ID` 默认 **94**，
  与 91 的既有 L2 契约测试分开）：CI 内无需 Docker 即可证明 respawn + 协调器重新提拉可组合——
  起 `LifecycleNode(respawn=True, respawn_delay=1.0)` + `auto_bringup:=true` 协调器 → 等 `ready` →
  SIGKILL dispatcher → 30s 内新进程出现、`get_state` 为 `active`、`/system/ready` 再次 `true`。
- **`scripts/check_web_console_e2e.sh`** 增加上述 4 个 topology/readiness phase。
- `scripts/check_all.sh` 的测试矩阵新增"探针与守护"、"拓扑热更新"两行（状态如实填）。
- CI `e2e-fleet` job 增列两套新脚本，沿用 `AMR_E2E_INNER=1` 内层守卫。

### G4. 回归

core（现有 1012 例 + 新增约 37 例）、tools（现有 57 例 + 新增约 20 例）全绿；
既有 7 套 e2e 全部 PASS（重点 `check_state_persistence_e2e.sh`、`check_metrics_e2e.sh`、
`check_rest_gateway_hardening.sh` —— 分别直接碰持久化、指标契约、网关请求路径）。

---

## H. 文档

- `docs/API.md`：新增 `/healthz` + `/readyz`（组件表 / 阈值 / 免鉴权理由 / 状态码）、
  §5.x 拓扑 REST（请求响应示例、`error_code` 表、单向与预约层的语义缺口）、3 个新 srv、
  新旧两套网关参数、dispatcher 新参数。
- `docs/DEPLOYMENT.md`：HEALTHCHECK 的真实边界（不触发自动重启，禁止 autoheal）、
  respawn 与协调器的依赖、冷重启会重派在途任务、webhook 配置与 Alertmanager 规则引用、
  施工区 runbook（预演 → 提交 → 收工清空；`.bak`/`.stale`/`.corrupt` 文件生命周期与手工回滚）。
- `deploy/alerts/amr_dispatcher.rules.yml` + `README.md`：`AmrNotReady`、`AmrDispatcherStateStale`、
  `AmrSafetyCriticalLatched`、`AmrAlertChannelBroken`、`AmrTopologyOverlayActive`（临时改道变永久了）。
- `docs/EXTENSION_ROADMAP.md` 新增 Phase 11；`README.md`/`USAGE.md` 增加两条脚本入口。

---

## 实施顺序

| 步骤 | 范围 | 依赖 | 理由 |
|---|---|---|---|
| 1 | A1 respawn + A3 HEALTHCHECK | — | 约 30 行即止血 8 天无人发现的问题 |
| 2 | A2 探针（含请求快车道拆分） | 1 | HEALTHCHECK 的目标；也是告警的判据来源 |
| 3 | B 告警 + 新指标 | 2 | 触发条件定义在探针结论上 |
| 4 | C 核心叠加层库 | — | 纯函数，可与 1-3 并行 |
| 5 | D dispatcher 两阶段生效 | 4 | 最大最险的一块 |
| 6 | E 网关拓扑端点 | 5 | 薄转发层 |
| 7 | F 控制台面板 + mjs phases | 6 | |
| 8 | G/H 测试收口与文档 | 1-7 | |

## 提交批次

1. `feat(launch): 全节点进程守护与容器健康检查`
2. `feat(tools): /healthz 与 /readyz 探针、请求快车道与组件聚合`
3. `fix(ros): 协调器就绪判定补齐安全状态新鲜度`
4. `feat(tools): Webhook 告警状态机、发送线程与 Prometheus 指标扩充`
5. `feat(core): 拓扑叠加层、连通性分析与在途影响分析纯函数`
6. `feat(ros): dispatcher 拓扑热更新服务与两阶段在途重规划仲裁`
7. `feat(tools): 拓扑运维 REST 端点`
8. `feat(web): 拓扑运维面板与就绪红灯表`
9. `test(e2e): 健康守护与拓扑热更新端到端验收 + L2 守护契约`
10. `docs: API/部署/告警规则/路线图同步`

## 验收口径

| 检验项 | 命令 | 预期 |
|---|---|---|
| Core 单测 | `ctest --test-dir build/core` | 全绿（含 4 个新套件） |
| Tools 单测 | 容器内 `ctest --test-dir build/amr_dispatcher_tools` | 全绿 |
| 既有 e2e | 7 套依次运行 | 全部 PASS，零回归 |
| 探针与守护 | `./scripts/check_health_supervision_e2e.sh` | 退出码 0（含 kill→503→重生→200） |
| 拓扑热更新 | `./scripts/check_topology_hot_update_e2e.sh` | 退出码 0（含 409 原子拒绝、重启存活、基线变更失效） |
| 控制台 | `./scripts/check_web_console_e2e.sh` | 全 PASS（含 4 个新 phase） |
| 生产隔离 | 全程域 ∈ {92,93,94}、REST ∈ {18100,18101}、webhook ∈ {18111} | 42/8080/1883 从未出现 |
