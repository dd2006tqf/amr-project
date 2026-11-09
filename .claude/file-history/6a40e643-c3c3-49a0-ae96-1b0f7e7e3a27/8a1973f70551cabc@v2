# VDA 5050 车队协议适配层与异构机器人接入实施计划

> REQUIRED SUB-SKILL: Use subagent-driven-development or executing-plans

**Goal:** 构建符合 VDA 5050 工业标准的车队通信适配层（Fleet Protocol Adapter），向内通过统一抽象接口 `IVehicleAdapter` 衔接调度核心，向外通过标准 MQTT 协议接入异构移动机器人，实现标准化订单下发、状态归一化与逐段路权释放。
**Architecture:** 纯 C++ 核心库负责统一抽象接口定义、VDA 5050 JSON 报文编解码与 Order 构建（保持零 ROS、零 MQTT 依赖）；ROS 薄壳 `vda5050_gateway_node` 桥接 ROS 2 调度系统与外部 MQTT Broker；配套提供轻量 Python 虚拟小车进行端到端全链路自动化验收。
**Tech Stack:** C++17, ROS 2 Jazzy, MQTT (mosquitto/libpaho), JSON, GoogleTest, Python 3
**Spec:** `docs/superpowers/specs/vda5050_fleet_adapter.md`

## Global Constraints
1. 核心库 `amr_dispatcher_core` 保持**零 ROS 依赖、零 MQTT 依赖**，协议编解码与订单映射算法必须能在脱机单测（`ctest`）中毫秒级执行。
2. 遵循 TDD 规范：先写失败测试并确认红灯，再写最小实现使其变绿。
3. 严格遵循安全准则：MQTT 通信断开时，保留小车已持有的所有路网资源锁，杜绝后车追尾事故。

---

### Task 1: 核心库抽象车队控制接口与数据模型 (P0)

**Files:**
- Create: `src/amr_dispatcher_core/include/amr_dispatcher_core/fleet/vehicle_adapter_interface.hpp`
- Create: `src/amr_dispatcher_core/src/fleet/vehicle_adapter_interface.cpp`
- Modify: `src/amr_dispatcher_core/CMakeLists.txt`
- Create: `test/unit/test_vehicle_adapter_interface.cpp`

**Interfaces:**
- Produces: `IVehicleAdapter` 纯虚类、`VehicleOrder`、`VehicleNode`、`VehicleEdge`、`VehicleAction`、`VehicleStateReport` 结构体。
- Produces: `MockVehicleAdapter` 测试桩（实现内存单车状态维护与订单收发）。

- [ ] Step 1: 编写 `test_vehicle_adapter_interface.cpp`，测试基于 `MockVehicleAdapter` 的订单分发、状态更新与取消操作。
- [ ] Step 2: 运行测试确认红灯（头文件缺失）。
- [ ] Step 3: 实现 `vehicle_adapter_interface.hpp` 和 `vehicle_adapter_interface.cpp`。
- [ ] Step 4: 编译并运行单测，确认全部变绿（`ctest --test-dir build/core -R VehicleAdapter`）。
- [ ] Step 5: Git commit 提交改动。

---

### Task 2: 纯 C++ VDA 5050 报文序列化与反序列化引擎 (P1)

**Files:**
- Create: `src/amr_dispatcher_core/include/amr_dispatcher_core/fleet/vda5050_serializer.hpp`
- Create: `src/amr_dispatcher_core/src/fleet/vda5050_serializer.cpp`
- Modify: `src/amr_dispatcher_core/CMakeLists.txt`
- Create: `test/unit/test_vda5050_serializer.cpp`

**Interfaces:**
- Produces:
  - `std::string SerializeVda5050Order(const VehicleOrder& order, const std::string& version = "v2")`
  - `std::string SerializeVda5050InstantActions(const std::string& vehicle_id, const std::vector<VehicleAction>& actions)`
  - `std::optional<VehicleStateReport> DeserializeVda5050State(const std::string& json_str)`
  - `std::optional<std::string> DeserializeVda5050Connection(const std::string& json_str)` // "ONLINE", "CONNECTIONBROKEN"

- [ ] Step 1: 编写 `test_vda5050_serializer.cpp`，覆盖合规 JSON 报文解析、非法字段防护、NaN 坐标校验、以及 v2/v3 Topic 规范。
- [ ] Step 2: 运行测试确认红灯。
- [ ] Step 3: 在 `vda5050_serializer.hpp/cpp` 中实现轻量 JSON 编解码（纯标准库或轻量解析，零外部重依赖）。
- [ ] Step 4: 运行测试确认绿灯通过。
- [ ] Step 5: Git commit 提交改动。

---

### Task 3: 拓扑折线转 VDA 5050 订单构建引擎 `Vda5050OrderBuilder` (P1)

**Files:**
- Create: `src/amr_dispatcher_core/include/amr_dispatcher_core/fleet/vda5050_order_builder.hpp`
- Create: `src/amr_dispatcher_core/src/fleet/vda5050_order_builder.cpp`
- Modify: `src/amr_dispatcher_core/CMakeLists.txt`
- Create: `test/unit/test_vda5050_order_builder.cpp`

**Interfaces:**
- Consumes: `StationCatalog`（含 road_nodes 与 polyline 边）、Dijkstra 路径节点序列、资源锁列表。
- Produces: `VehicleOrder BuildOrderFromPath(const std::string& vehicle_id, const std::string& mission_id, const std::vector<std::string>& path_node_ids, const catalog::StationCatalog& catalog, const std::unordered_set<std::string>& reserved_locks)`。
- Rule: 只有在 `reserved_locks` 存在的边和节点，设置 `released = true`（Base 区间）；未取得锁的设置为 `released = false`（Horizon 规划区间）。

- [ ] Step 1: 编写 `test_vda5050_order_builder.cpp`，验证 L 弯路径转换、Base/Horizon 分段放行标记及动作挂载。
- [ ] Step 2: 运行测试确认红灯。
- [ ] Step 3: 实现 `vda5050_order_builder.hpp/cpp`。
- [ ] Step 4: 运行单测确认全部绿灯（`ctest --test-dir build/core` 保持 100% 通过）。
- [ ] Step 5: Git commit 提交改动。

---

### Task 4: ROS 2 VDA 5050 MQTT 通信适配网关 `vda5050_gateway_node` (P2)

**Files:**
- Create: `src/amr_dispatcher_ros/include/amr_dispatcher_ros/vda5050_gateway_node.hpp`
- Create: `src/amr_dispatcher_ros/src/vda5050_gateway_node.cpp`
- Modify: `src/amr_dispatcher_ros/CMakeLists.txt`
- Modify: `launch/full_system.launch.py`

**Interfaces:**
- 负责与外部 MQTT Broker 通信：
  - 订阅 `uagv/v2/+/+/state`，解析后转换为 ROS 2 话题 `/fleet/{vehicle_id}/state` 与位姿 `/odom`。
  - 订阅 `uagv/v2/+/+/connection`，检测 LWT 遗嘱断链，通知调度安全门。
  - 监听内部调度下发，生成并发布 `uagv/v2/{manufacturer}/{serialNumber}/order`。
- 提供启动参数：`use_vda5050`（默认 false），`mqtt_broker_host`（默认 "127.0.0.1"），`mqtt_port`（默认 1883）。

- [ ] Step 1: 在 `amr_dispatcher_ros/CMakeLists.txt` 中配置编译目标与依赖。
- [ ] Step 2: 实现 `vda5050_gateway_node` 核心收发逻辑与 MQTT 客户端封装。
- [ ] Step 3: 更新 `full_system.launch.py`，支持一键拉起 VDA 5050 网关。
- [ ] Step 4: 容器内编译验证通过。
- [ ] Step 5: Git commit 提交改动。

---

### Task 5: 逐段路权动态释放与断链安全守卫 (P3)

**Files:**
- Modify: `src/amr_dispatcher_core/src/workflow/traffic_planning.cpp`
- Modify: `src/amr_dispatcher_ros/src/dispatcher_node.cpp`
- Create: `test/unit/test_dynamic_unlocking.cpp`

**Interfaces:**
- Rule 1 (动态解锁): 当收到小车上报 `lastNodeId` 已经跨越 node_A 进入 edge_AB 时，自动调用 `ResourceTable` 释放已离开的路网节点与边锁，唤醒等待后车。
- Rule 2 (断链安全): 当小车 connection 离线时，保留其持有锁，挂起派发并触发软看门狗告警。

- [ ] Step 1: 编写单测验证车辆沿折线推进时逐段释放旧锁逻辑。
- [ ] Step 2: 运行单测确认红灯。
- [ ] Step 3: 在 `dispatcher_node` 中接入 VDA 5050 状态流，打通动态解锁与断链保护。
- [ ] Step 4: 运行单测确认全部变绿。
- [ ] Step 5: Git commit 提交改动。

---

### Task 6: Python 虚拟小车模拟器与端到端联合验收 (P2/P3)

**Files:**
- Create: `scripts/mock_vda5050_vehicle.py`
- Create: `scripts/check_vda5050_e2e.sh`
- Modify: `docker/Dockerfile.jazzy` (确保包含 mosquitto 依赖)

**Interfaces:**
- `mock_vda5050_vehicle.py`：模拟真实的 Geek+ 或海康 AMR，连接 MQTT Broker，接收 order，按折线步进模拟行驶，按 2Hz 上报 state 报文，并支持模拟网络故障。
- `check_vda5050_e2e.sh`：自动化执行端到端闭环验证：
  1. 启动轻量 Mosquitto MQTT Broker；
  2. 启动系统（启用 `use_vda5050:=true`）；
  3. 启动 `mock_vda5050_vehicle.py`；
  4. 提单下发，验证 MQTT 收到合法 VDA 5050 Order；
  5. 验证小车沿折线运动、中心端逐段解锁、最终到达完成；
  6. 验证模拟断链场景下，锁不被非法释放。

- [ ] Step 1: 编写 `mock_vda5050_vehicle.py` 仿真脚本。
- [ ] Step 2: 编写并执行 `check_vda5050_e2e.sh` 全链路验收脚本。
- [ ] Step 3: 运行全量 `scripts/check_all.sh` 确保 566+ 单测无回归。
- [ ] Step 4: 更新 `README.md` 与 `docs/EXTENSION_ROADMAP.md`。
- [ ] Step 5: Git commit 提交全部改动。
