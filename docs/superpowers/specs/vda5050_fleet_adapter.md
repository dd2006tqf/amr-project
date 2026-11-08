# VDA 5050 车队协议适配层与异构机器人接入设计规范 (v1.0)

## 1. 架构定位与设计哲学

> **系统定位**：构建一个符合工业通用标准（VDA 5050）的车队通信适配层（Fleet Protocol Adapter），向内通过统一抽象接口 `IVehicleAdapter` 衔接调度核心（任务队列、拓扑路网、时空原子锁与行为树），向外通过标准 MQTT 协议接入不同厂商的自主移动机器人（AMR/AGV），消除异构车队硬件私有协议孤岛。

### 核心分工铁律：
1. **调度大脑不碰协议**：核心库 `amr_dispatcher_core` 保持**零 ROS 依赖、零 MQTT 依赖、零网络 I/O 依赖**。调度系统只通过统一的数据结构 `VehicleOrder` 与虚接口 `IVehicleAdapter` 操作车辆。
2. **通信标准不碰调度逻辑**：VDA 5050 纯粹是车辆与中心控制台之间的“消息契约”，不包含任何路网寻路、时空锁互斥、多车死锁防范逻辑。所有的路权分配与安全仲裁由本系统的 `ResourceTable` 和 `SafetyGate` 权威裁决。
3. **断链绝对不等于释放路权**：网络断开或 MQTT LWT 遗嘱触发时，仅标记通信离线并阻止新任务下发，**绝对不释放该车持有的路段锁**，防止后车追尾；只有当收到明确停止或人工介入时才回收路权。

---

## 2. 系统整体分层架构

```text
┌─────────────────────────────────────────────────────────────────┐
│                 amr_dispatcher_core (调度大脑)                  │
│   • MissionQueue (任务优先级队列)  • ResourceTable (时空原子锁) │
│   • SkeletonPlanner (折线路径)     • DeadlockDetector (死锁自愈)│
└────────────────────────────────┬────────────────────────────────┘
                                 │
                 C++ 统一纯虚接口: │ IVehicleAdapter (core/fleet/)
                                 │
         ┌───────────────────────┴───────────────────────┐
         ▼                                               ▼
┌────────────────────────────────┐              ┌────────────────────────────────┐
│ MockVehicleAdapter             │              │ Vda5050VehicleAdapter          │
│ (现有内部模拟底盘/PTY串口通信)  │              │ (vda5050_gateway_node ROS 节点)│
└────────────────────────────────┘              └───────────────┬────────────────┘
                                                                │ MQTT (JSON)
                                                                ▼
                                                        MQTT Broker (Mosquitto)
                                                                │
                                                ┌───────────────┴───────────────┐
                                                ▼                               ▼
                                            Geek+ 潜伏车                  海康 叉车 AMR
```

---

## 3. 核心数据契约 (Domain Schemas, `core/fleet/`)

### 3.1 统一车辆控制抽象 `vehicle_adapter_interface.hpp`
```cpp
namespace amr_dispatcher_core::fleet {

struct VehicleAction {
  std::string action_type; // e.g., "lift", "drop", "charge", "beep"
  std::string action_id;
  std::unordered_map<std::string, std::string> parameters;
};

struct VehicleTrajectoryPoint {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct VehicleEdge {
  std::string edge_id;
  std::string start_node_id;
  std::string end_node_id;
  bool released = true; // 是否获得路权允许小车行驶
  double max_speed = 1.0;
  std::vector<VehicleTrajectoryPoint> polyline;
  std::vector<VehicleAction> actions;
};

struct VehicleNode {
  std::string node_id;
  uint32_t sequence_id = 0;
  bool released = true; // 是否已放行
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  std::vector<VehicleAction> actions;
};

struct VehicleOrder {
  std::string order_id;
  uint32_t order_update_id = 0;
  std::string vehicle_id;
  std::vector<VehicleNode> nodes;
  std::vector<VehicleEdge> edges;
};

struct VehicleStateReport {
  std::string vehicle_id;
  std::string order_id;
  uint32_t last_node_sequence_id = 0;
  std::string last_node_id;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double battery_percent = 100.0;
  bool is_charging = false;
  bool estop_active = false;
  bool online = true;
  std::string operating_mode = "AUTOMATIC"; // "AUTOMATIC", "MANUAL", "SEMIAUTOMATIC"
  std::vector<std::string> active_errors;
  std::chrono::steady_clock::time_point last_report_time;
};

class IVehicleAdapter {
 public:
  virtual ~IVehicleAdapter() = default;

  virtual bool DispatchOrder(const std::string& vehicle_id, const VehicleOrder& order) = 0;
  virtual bool CancelOrder(const std::string& vehicle_id, const std::string& order_id) = 0;
  virtual bool PauseVehicle(const std::string& vehicle_id) = 0;
  virtual bool ResumeVehicle(const std::string& vehicle_id) = 0;
  virtual std::optional<VehicleStateReport> GetVehicleState(const std::string& vehicle_id) const = 0;
};

} // namespace amr_dispatcher_core::fleet
```

---

## 4. VDA 5050 报文映射规范与 Builder 模块

### 4.1 Topic 结构设计
支持根据配置动态适配规范：
- v2.x 默认格式：`uagv/v2/{manufacturer}/{serialNumber}/{topic}`
- v3.x 默认格式：`vda5050/v3/{manufacturer}/{serialNumber}/{topic}`
其中核心 topic 包括：
- `order`：主控下发路径与动作
- `instantActions`：急停、暂停、复位等即时动作
- `state`：小车高频上报状态 (1~5Hz)
- `connection`：LWT 遗嘱与网络状态

### 4.2 订单生成器 `Vda5050OrderBuilder` (`core/fleet/`)
- 输入：`SkeletonPlanner` 规划出的路网节点序列、真实折线 `polyline`、以及 `ResourceTable` 分配的锁状态。
- 输出：符合 VDA 5050 标准 JSON 报文。
- **路权放行（Horizon 与 Base）机制**：
  - 只有成功取得 `ResourceTable::Reserve` 原子锁的路段，其对应的 `Node.released` 和 `Edge.released` 赋值为 `true`（Base 区间）；
  - 尚未取得锁的前方路段，赋值为 `released = false`（Horizon 规划区间），小车在接近未放行节点时自动平稳制动等待。

---

## 5. 通信容灾与多车安全防撞安全准则

1. **原子锁全有或全无**：
   - 下发订单前必须先通过 `ResourceTable` 锁定全部预期经过的路段和节点；任一资源冲突，整单挂起等待，严禁向小车下发未获路权许可的 `released=true` 边。
2. **断链锁定保护**：
   - 收到 MQTT 遗嘱（`connectionState == "CONNECTIONBROKEN"`）或心跳超时，适配器立即向调度系统标记该车 `online = false`；
   - 调度系统**保留该车占用的所有资源锁**，直至车辆重新上线核对坐标，或操作员在控制台手动强行解除占用。
3. **逐段释放（Dynamic Unlocking）**：
   - 当小车通过 `state` 报文确认 `lastNodeId` 已经推进并越过某个路口时，适配器触发内部回调，释放小车身后已经脱离的路网边锁，使等待的后车可以立即通行。
