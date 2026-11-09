# 地图导入与自动路网驱动调度引擎实施计划

> REQUIRED SUB-SKILL: Use subagent-driven-development or executing-plans

**Goal:** 实现标准 ROS 2 占据栅格地图包自动导入、受保护骨架路网提取、业务站点吸附、混合图 Dijkstra 寻路、原子路径锁与 SkeletonPlanner 真实折线循迹的完整闭环。
**Architecture:** 纯 C++ 核心库负责安全空间腐蚀、骨架细化、受保护剪枝与拓扑构建；ROS 薄壳 `map_ingest_node` 订阅 `/map` 并原子写出 `stations.yaml`；调度主循环通过 `StationCatalogReloader` 热重载，使用扩展后的 `FindRoutePath` 与 `SkeletonPlanner` 驱动底盘执行。
**Tech Stack:** C++17, ROS 2 Jazzy, nav2_map_server, GoogleTest, YAML-CPP, Docker
**Spec:** `docs/superpowers/specs/map_ingest_and_topology_dispatch.md`

## Global Constraints
1. 核心库 `amr_dispatcher_core` 保持**零 ROS 依赖**，所有几何与图算法必须能在脱机单测（`ctest`）中毫秒级执行。
2. 调度节点 `dispatcher_node` 不使用任何共享内存与 map_ingest 耦合，通过 `tmp+rename` 原子写文件与 `mtime` 热重载通讯。
3. 遵循 TDD 规范：先写失败测试并确认红灯，再写最小实现使其变绿。

---

### Task 1: 扩展 StationCatalog 数据结构与混合图寻路 (P0/P1)

**Files:**
- Modify: `src/amr_dispatcher_core/include/amr_dispatcher_core/catalog/station_catalog.hpp`
- Modify: `src/amr_dispatcher_core/src/catalog/station_catalog.cpp`
- Test: `test/unit/test_station_catalog.cpp`

**Interfaces:**
- Produces: `RoadNode` 结构体、`CatalogStationEdge::polyline`、`StationCatalog::map_hash`、`StationCatalog::road_nodes`。
- Updates: `ValidateStationCatalog` 校验支持端点属于 `stations` 或 `road_nodes`。
- Updates: `FindRoutePath` 支持在 `stations + road_nodes` 混合图上执行 Dijkstra。

- [ ] Step 1: 在 `test/unit/test_station_catalog.cpp` 中编写混合图寻路与带 polyline 边的失败单测。
- [ ] Step 2: 运行测试确认编译失败或断言失败（红灯）。
- [ ] Step 3: 在 `station_catalog.hpp/cpp` 中实现 `RoadNode`、`polyline` 解析，以及 `FindRoutePath` 对混合节点的支持与 `ValidateStationCatalog` 逻辑。
- [ ] Step 4: 重新编译并执行测试，确认全部变绿。
- [ ] Step 5: Git commit 提交改动。

---

### Task 2: 纯 C++ 栅格几何与坐标变换模块 `MapGeometryUtils` (P1)

**Files:**
- Create: `src/amr_dispatcher_core/include/amr_dispatcher_core/planning/map_geometry_utils.hpp`
- Create: `src/amr_dispatcher_core/src/planning/map_geometry_utils.cpp`
- Test: `test/unit/test_map_geometry_utils.cpp`

**Interfaces:**
- Produces: `MapGrid` 结构体（width, height, resolution, origin_x, origin_y, origin_yaw, data: std::vector<int8_t>）。
- Produces: `WorldToPixel(x, y)` 和 `PixelToWorld(col, row)`，完整支持旋转角和中心半像素对齐。

- [ ] Step 1: 编写 `test_map_geometry_utils.cpp`，覆盖原点平移、旋转 15° 与半像素对齐用例。
- [ ] Step 2: 运行测试确认红灯。
- [ ] Step 3: 实现 `map_geometry_utils.hpp/cpp` 矩阵旋转和平移换算。
- [ ] Step 4: 运行测试确认绿灯。
- [ ] Step 5: Git commit 提交改动。

---

### Task 3: 机器人尺寸腐蚀与自由空间过滤 `SafetySpaceFilter` (P1)

**Files:**
- Create: `src/amr_dispatcher_core/include/amr_dispatcher_core/planning/safety_space_filter.hpp`
- Create: `src/amr_dispatcher_core/src/planning/safety_space_filter.cpp`
- Test: `test/unit/test_safety_space_filter.cpp`

**Interfaces:**
- Produces: `BinaryGrid FilterSafetySpace(const MapGrid& grid, double robot_radius, double safety_margin)`。
- Rule: 障碍物（>65）与未知区域（-1）统一按不可通行处理，以圆形核执行腐蚀操作。

- [ ] Step 1: 编写 `test_safety_space_filter.cpp`，测试狭窄通道被腐蚀切断与未知区域被阻断行为。
- [ ] Step 2: 运行测试确认红灯。
- [ ] Step 3: 实现 `safety_space_filter.hpp/cpp`（基于欧式距离变换或形态学腐蚀）。
- [ ] Step 4: 运行测试确认绿灯。
- [ ] Step 5: Git commit 提交改动。

---

### Task 4: 细化骨架提取与受保护剪枝引擎 `SkeletonExtractor` (P1)

**Files:**
- Create: `src/amr_dispatcher_core/include/amr_dispatcher_core/planning/skeleton_extractor.hpp`
- Create: `src/amr_dispatcher_core/src/planning/skeleton_extractor.cpp`
- Test: `test/unit/test_skeleton_extractor.cpp`

**Interfaces:**
- Produces: `ZhangSuenThinning(const BinaryGrid& grid)` 输出单像素骨架。
- Produces: `PruneSkeletonProtected(BinaryGrid& skeleton, const std::vector<PixelCoord>& protected_points, double min_branch_length_m, double resolution)`。
- Rule: 剪除小于阈值的毛刺分支，但若分支包含保护点（站点）则强制豁免。

- [ ] Step 1: 编写 `test_skeleton_extractor.cpp`，包含 L 弯、十字路口及带站点保护的毛刺单测。
- [ ] Step 2: 运行测试确认红灯。
- [ ] Step 3: 实现经典 Zhang-Suen 细化与受保护剪枝遍历算法。
- [ ] Step 4: 运行测试确认绿灯。
- [ ] Step 5: Git commit 提交改动。

---

### Task 5: 拓扑路网构建与站点吸附边拆分 `TopologyGraphBuilder` (P1)

**Files:**
- Create: `src/amr_dispatcher_core/include/amr_dispatcher_core/planning/topology_graph_builder.hpp`
- Create: `src/amr_dispatcher_core/src/planning/topology_graph_builder.cpp`
- Test: `test/unit/test_topology_graph_builder.cpp`

**Interfaces:**
- Produces: `BuildTopologyCatalog(const BinaryGrid& skeleton, const MapGrid& map_info, const std::vector<AnchorStation>& anchors, double max_snap_dist_m)`。
- Rule: 自动提取 road_nodes、沿骨架提取真实 polyline 折线；将站点正交投影拆边并检验全图连通性。

- [ ] Step 1: 编写 `test_topology_graph_builder.cpp`，覆盖端点追踪、折线坐标生成与断网报错单测。
- [ ] Step 2: 运行测试确认红灯。
- [ ] Step 3: 实现跟踪像素追踪图构建与投影插点逻辑。
- [ ] Step 4: 运行测试确认绿灯。
- [ ] Step 5: Git commit 提交改动。

---

### Task 6: 真实折线规划器 `SkeletonPlanner` (P1/P3)

**Files:**
- Create: `src/amr_dispatcher_core/include/amr_dispatcher_core/planning/skeleton_planner.hpp`
- Create: `src/amr_dispatcher_core/src/planning/skeleton_planner.cpp`
- Test: `test/unit/test_skeleton_planner.cpp`

**Interfaces:**
- Consumes: `IPathPlanner` 接口、`StationCatalog`。
- Produces: `std::vector<Pose2D> Plan(const Pose2D& start, const Pose2D& goal, const std::string& edge_id)`。
- Rule: 沿 Dijkstra 结果拼接 polyline，以 0.1m 步长等距插值，计算航向角。

- [ ] Step 1: 编写 `test_skeleton_planner.cpp`，验证 L 弯直角走廊不发生穿墙、按走廊折线前进。
- [ ] Step 2: 运行测试确认红灯。
- [ ] Step 3: 实现 `SkeletonPlanner` 拼接与插值逻辑。
- [ ] Step 4: 运行测试确认绿灯。
- [ ] Step 5: Git commit 提交改动。

---

### Task 7: 调度节点全路线原子锁与版本门禁强化 (P3)

**Files:**
- Modify: `src/amr_dispatcher_core/src/workflow/traffic_planning.cpp`
- Modify: `src/amr_dispatcher_ros/src/dispatcher_node.cpp`
- Test: `test/unit/test_traffic_planning.cpp`

**Interfaces:**
- Updates: `PlanMissionRouteLocks` 生成包含中间 `road_node` 与边的连续锁序列。
- Updates: `DispatcherNode::ScheduleTick` 校验 `map_hash` 版本一致性。

- [ ] Step 1: 编写 `test_traffic_planning.cpp` 验证多边全路线锁列表生成。
- [ ] Step 2: 运行单测确认红灯。
- [ ] Step 3: 更新 `PlanMissionRouteLocks` 与 `DispatcherNode::ScheduleTick` 中的版本门禁守卫。
- [ ] Step 4: 运行单测确认绿灯。
- [ ] Step 5: Git commit 提交改动。

---

### Task 8: 地图导入节点 `MapIngestNode` 与 RViz 拓扑可视化 (P2)

**Files:**
- Create: `src/amr_dispatcher_ros/src/map_ingest_node.cpp`
- Create: `src/amr_dispatcher_ros/include/amr_dispatcher_ros/map_ingest_node.hpp`
- Modify: `src/amr_dispatcher_ros/CMakeLists.txt`
- Modify: `launch/full_system.launch.py`

**Interfaces:**
- Consumes: `/map` (nav_msgs::msg::OccupancyGrid)。
- Produces: `config/generated/stations.yaml` (原子 tmp+rename)。
- Produces: `/dispatcher/topology_markers` (visualization_msgs::msg::MarkerArray)。

- [ ] Step 1: 在 CMakeLists.txt 中注册 `map_ingest_node`。
- [ ] Step 2: 编写 `map_ingest_node.cpp`，接入核心库算法流水线与 Marker 发布。
- [ ] Step 3: 在 `full_system.launch.py` 中增加 map_server 和 map_ingest_node 编排选项。
- [ ] Step 4: 容器内构建并验证节点启动与参数解析正常。
- [ ] Step 5: Git commit 提交改动。

---

### Task 9: 端到端集成测试与环境交付 (P3)

**Files:**
- Create: `test/maps/demo_warehouse/map.yaml`
- Create: `test/maps/demo_warehouse/map.pgm`
- Create: `test/maps/demo_warehouse/markers.yaml`
- Create: `scripts/check_map_ingest_e2e.sh`
- Modify: `Dockerfile`

**Interfaces:**
- 完整验收场景：容器启动加载 `demo_warehouse` 地图包，自动提取路网并发布 Marker，通过 HTTP 提交搬运订单，小车沿走廊折线安全到站。

- [ ] Step 1: 更新 `Dockerfile`，添加 `ros-jazzy-nav2-map-server` 依赖。
- [ ] Step 2: 构造包含 L 弯走廊的 `demo_warehouse` 标准测试地图与语义文件。
- [ ] Step 3: 编写并执行 `check_map_ingest_e2e.sh` 全流程自动化验收脚本。
- [ ] Step 4: 运行现有 `scripts/check_all.sh`，确保 457+ 单元测试与三大混沌演练零回归。
- [ ] Step 5: Git commit 提交全部改动。
