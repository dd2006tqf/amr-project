# 地图导入与自动路网驱动调度引擎设计规范 (v2.1)

## 1. 系统定位与设计边界

> **项目定位**：本项目构建一个面向 AMR 调度系统的地图转换引擎，将标准占据栅格地图与站点语义标注转换为可验证的拓扑路网，并接入现有路径规划和多车资源预约机制，实现地图驱动的任务调度与路径执行。

- **一期交付目标**：静态地图输入 → 自动路网与站点生成 → Dijkstra 选路 → 路线原子锁定 → 车辆沿真实折线（Polyline）循迹到站。
- **一期明确暂缓**：在线热切图、栅格 A* 接驳、Nav2 全家桶接管、真机实时相机视觉定位。

---

## 2. 核心架构与数据流向

```
用户地图包目录/
├── map.yaml               # 官方格式（resolution/origin/image）
├── map.pgm                # 纯障碍物地图（白=通行，黑=障碍，无任何涂抹）
└── markers.yaml           # 站点语义（anchors 坐标与类型映射）

        │
        ▼ 官方 nav2_map_server 节点解析发布 /map (OccupancyGrid)
┌─────────────────────────────────────────────────────────────┐
│ map_ingest_node (ROS 薄壳，唯一新增进程)                   │
│   ① 订阅 /map (transient_local QoS)                         │
│   ② 转换为核心库 MapGrid 纯数据结构                         │
│   ③ 调用纯 C++ 核心库处理流水线 (core/planning)            │
│   ④ 输出 config/generated/stations.yaml (tmp+rename 原子写) │
│   ⑤ 发布 MarkerArray 供 RViz 预览路网与站点                 │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│ 现有 AMR 调度主链路 (薄壳 + 核心库 + 行为树)                │
│   ① StationCatalogReloader 1Hz mtime 监测并热重载           │
│   ② ScheduleTick 校验 map_hash 版本门禁                     │
│   ③ PlanMissionRouteLocks 在混合图上规划整条路线锁          │
│   ④ ResourceTable 原子全有或全无预留                        │
│   ⑤ SkeletonPlanner 沿边的 polyline 折线等距展开路点        │
│   ⑥ path_tracker_node 循迹 -> safety_gate -> 模拟器到站    │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. 数据契约定义 (Schema)

### 3.1 站点语义配置 `markers.yaml`
```yaml
anchors:
  - id: "station_pickup_A"
    x: 1.5
    y: 2.0
    yaw: 0.0
    type: "pickup"
  - id: "station_dropoff_1"
    x: 8.0
    y: 5.0
    yaw: 1.5707
    type: "dropoff"
  - id: "station_charge_1"
    x: 0.0
    y: 0.0
    yaw: 3.1415
    type: "charge"
```

### 3.2 自动生成产物 `config/generated/stations.yaml` (v2.1)
```yaml
version:
  map_hash: "sha256_of_map_yaml_and_pgm"
  graph_version: "v1_algo_hash"

stations:
  - id: "station_pickup_A"
    x: 1.5
    y: 2.0
    yaw: 0.0
    type: "pickup"

road_nodes:
  - id: "node_junc_1"
    x: 1.5
    y: 5.0

edges:
  - from: "station_pickup_A"
    to: "node_junc_1"
    distance_m: 3.0
    bidirectional: true
    enabled: true
    polyline:
      - [1.50, 2.00]
      - [1.50, 3.50]
      - [1.50, 5.00]
```

---

## 4. 纯 C++ 算法流水线设计 (Zero-ROS, `core/planning/`)

算法链严格按顺序执行：
1. **坐标系变换与对齐 (`MapGeometryUtils`)**：
   - 支持非零原点、地图旋转 (`yaw`)、半像素中心偏移 (`+0.5 * resolution`)。
2. **机器人尺寸约束与腐蚀 (`SafetySpaceFilter`)**：
   - 膨胀障碍物/腐蚀通行区，腐蚀半径 $R = R_{robot} + R_{margin}$。
   - 栅格中未知区域（值为 -1）严格按障碍物处理，禁止通行。
3. **细化骨架提取 (`ZhangSuenThinning`)**：
   - 迭代消除边界点，提炼 1 像素宽的通行中心线。
4. **受保护的毛刺剪枝 (`ProtectedPruner`)**：
   - 提取候选站点位置（来自 anchors）。
   - 剪除长度小于阈值（默认 0.5m）的短分支；**但若分支承载站点，强制豁免，不可误删**。
5. **拓扑构建与折线提取 (`TopologyGraphBuilder`)**：
   - 识别度数 $\neq 2$ 的点为 `road_node`；
   - 沿骨架追踪像素点，记录真实几何折线 `polyline`（世界坐标米制）并计算累积弧长为 `distance_m`。
6. **站点投影与边拆分**：
   - 将站点正交投影到最近的安全边，在生成期将边拆分成两条子边，并将站点作为连通节点插入图中。
7. **连通性校验**：
   - 验证所有业务站点处于同一个最大连通分量，否则拒绝生成并报错。

---

## 5. 调度与运行时规划闭环

1. **混合图寻路与锁规划**：
   - 扩展 `StationCatalog` 支持 `road_nodes`；
   - 扩展 `FindRoutePath` 在 `stations` 与 `road_nodes` 混合节点集合上执行 Dijkstra；
   - `PlanMissionRouteLocks` 将整条路径拆分为连续的节点锁与边锁。
2. **原子路线锁**：
   - 派发时调用 `ResourceTable::Reserve(candidate_id, locks)`。任一锁冲突即全量回滚，队首保留等待。
3. **版本门禁**：
   - `ScheduleTick` 比对 `map_hash`。若检测到地图不匹配或突变，挂起新任务派发，将状态探针置为 `DEGRADED`。
4. **真实折线规划器 `SkeletonPlanner`**：
   - 实现 `IPathPlanner` 接口，根据 Dijkstra 选出的边，顺序拼接 `polyline` 并按 0.1m 等距插值输出 `std::vector<Pose2D>`。
