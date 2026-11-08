# Protobuf 接入 ROS 2 话题：完整替换方案

> 状态：阶段 0（包骨架 + 技术验证）已完成并**实测通过**。
> 本文档只记录已验证的事实、真实的迁移清单、分阶段步骤，以及不可回避的边界。
> 日期：2026-11-05

---

## 0. 结论先行

用**零 fork 的 typesupport 注入**，让 protobuf 消息直接作为 ROS 2 话题类型使用，
在 ROS 2 Jazzy 上已实测可行：

- C++ 侧 `create_publisher<ProtoT>` / `create_subscription<ProtoT>` 正常工作
- `ros2 amr echo` 能解析出完整字段（含嵌套 message、enum、uint64 大数值）
- 端到端 249 帧零丢帧，字段逐项校验通过

代价是三条硬约束，必须先接受：

1. **只覆盖话题（topic）**。服务与动作明确不支持，见 §5.1。
2. **`ros2 topic echo` / RViz / rosbag 回放不认**。已提供替代工具 `ros2 amr echo`，见 §5.2。
3. **DDS 类型名变了**，与既有 rosidl 话题**不能直接互通**，只能双发过渡或全量切换，见 §4。

### 完成度（2026-11-08）

**话题层的迁移已完成；本计划没有被"完整执行"，剩余项是刻意保留的边界，不是欠账。**

已落地：

- 阶段 0–2：骨架 / 垂直切片 / 7 条遥测话题的 proto 双发
- 阶段 3：全部消费者改读 proto（反投影，业务逻辑零改动）
- 阶段 4c（保守方案）：发布端只发 proto，rosidl 发布器与 `proto_rosidl_publish`
  参数已删除；`.msg` 与反投影层**刻意保留**以便回滚

仍然存在的、需要如实知道的事：

| 项 | 状态 | 性质 |
|---|---|---|
| `amr_dispatcher_interfaces/msg/` 的 7 个 `.msg` | 保留 | **刻意**：删除会让"切回 rosidl"无法回滚。前置条件是确认无第三方订阅旧话题，该确认未取得 |
| 反投影层与 `ProtoDualSubscriber::use_proto=false` | 保留 | 同上，回滚路径的实现 |
| 24 个服务 + 2 个动作 | 永久 rosidl | **架构性**，本方案只覆盖话题，见 §5.1 |
| `ros2 topic echo` / RViz / rosbag | 不可用于 proto 话题 | **架构性**，替代工具是 `ros2 amr echo`，见 §5.2 |
| `scripts/check_proto_dual_consistency.py` | 休眠 | 比对基准已随 4c 消失，保留作回滚路径的校验工具 |
| 多车 per-vehicle 话题的 proto 链路 | **已实测通过**（2026-11-08） | 补测结论见下方注记 |
| 外部第三方订阅者是否受影响 | **未确认** | 保守 4c 并未解除此风险，它只降低了回滚成本 |

若要把"完整 4c"也做掉（删 `.msg` 与反投影层），前置条件与连带影响见
「阶段 4 / 完整 4c 仍未做」。

> **多车 per-vehicle 补测记录（2026-11-08）**：分别在 `ROS_DOMAIN_ID=76/77`
> 起了 `vehicle_id:=amr01` 的 `chassis_driver_node` 与 `safety_gate_node`。
> `ros2 topic info /chassis/link_health/amr01_proto --no-daemon` 得到
> `Publisher count: 1` / `Subscription count: 1`，订阅端 `node info` 显示
> `/chassis/link_health/amr01_proto: amr_dispatcher/proto/ChassisLinkHealth`——
> 发布端与订阅端**派生出同一个名字**，链路通。
>
> 注意话题名的形状：是 `/chassis/link_health/amr01_proto`，不是
> `/chassis/link_health_proto/amr01`。因为 `ResolveVehicleTopic` 先拼出
> `/chassis/link_health/amr01`，`ProtoTopicName` 再在末尾追加 `_proto`。
> 两侧派生顺序一致所以能通，但名字不好看；改它会同时动到已有话题契约，
> 故保持现状。
>
> **另一个排查陷阱**：`ros2 topic info` / `ros2 node info` 在这个容器里
> **不加 `--no-daemon` 会返回空输出**（daemon 缓存过期）。看到空输出时先加
> `--no-daemon` 重试，不要据此判断"节点没有订阅"。

---

## 1. 动机

现有 7 个自定义消息（`amr_dispatcher_interfaces/msg/`）由 rosidl 生成，采用**固定宽度**编码：
`uint64` 恒占 8 字节，`bool` 恒占 1 字节。而 `frames_total`、`commands_sent` 这类计数器
数值单调递增、高位长期为 0，固定宽度编码浪费明显。

但省字节不是唯一动机，甚至不是主要动机：

| 维度 | rosidl | protobuf |
|---|---|---|
| 跨语言 | Python/C++ 有生成器；Rust/Go/Java 需另找 | 官方支持十几种语言 |
| 向前兼容 | 加字段要重新生成 + 重新编译 + 重启 | 字段编号 + `reserved`，老端忽略新字段 |
| 枚举 | 只能靠字符串约定（现状 `tier` 就是 string） | 编译期检查 |
| 工具生态 | 绑定 ROS | `protoc`、gRPC、buf、schema 管理 |

需要说明的是：**本项目当前没有跨语言需求，也没有热升级需求**。
引入 protobuf 的收益主要是「未来可能性」与「编码效率」，属于工程投资而非痛点解决。
如果团队更看重「所有 ROS 工具开箱即用」，那么留在 rosidl 是完全合理的选择（见 §7）。

---

## 2. 技术方案

### 2.1 原理：两个特化点

ROS 2 在 C++ 侧用函数指针表 `rosidl_message_type_support_t` 描述一个消息类型。
`rclcpp::Publisher<T>` 通过 **ADL 找到的模板特化点** 获取它：

```cpp
rosidl_typesupport_cpp::get_message_type_support_handle<T>()
```

**我们为 protobuf 类型提供自己的特化**，从而绕过整个 rosidl 代码生成链路。
无需修改 rclcpp / rcl / rmw / rosidl 的任何一行源码。

还必须同时提供另一个特化点，否则 rclcpp 内部会走 rosidl 分支：

```cpp
rosidl_generator_traits::is_message<T>   // 必须返回 true
```

### 2.2 Jazzy 的四个强制回调——本方案与 Humble 的分水岭

Jazzy 的 `rcl` 会在以下位置**无条件解引用**函数指针。源码位置已逐条核对：

| 回调 | 调用点 | 缺失后果 |
|---|---|---|
| `get_type_hash_func` | `rcl/publisher.c:133` | SEGFAULT（跳转 0x0） |
| `get_type_description_func` | 同上 | SEGFAULT |
| `get_type_description_sources_func` | 同上 | SEGFAULT |
| introspection（经 `ts.func` 分发） | `TypeSupport_impl.cpp:612` | `failed to register type object with incompatible type` |

**这四个在 Humble 上都不存在。** 这是为什么把上游 ROS2-Protobuf 补丁
「改名搬过来」行不通——不是重命名，而是多了一层必须实现的协议。

各回调的实现方式：

- **type hash**：对 `FileDescriptorProto` 序列化后取 SHA-256，版本字节 `0x01`（RIHS01）。
- **type description**：返回合成的 `TypeDescription`，只填名称，字段表为空。
- **type description sources**：`encoding = "dynamic"`（官方定义值），`raw_file_contents` 填 `.proto` 源文本。
- **introspection**：返回合成的 `MessageMembers`，只填名称与大小，**不填字段偏移**。

关于 introspection 的安全性，已核对 `register_type_object` 的实现：它只读名称做类型名反查，
从不读 `offset_` 或数组访问器指针。因此合成结构不会崩。但也正因为反射信息不完整，
`ros2 topic echo` 无法工作（§5.2）——**这是同一个原因的两种后果**。

### 2.3 线格式

```
[0,2)    encapsulation kind    00 01 = CDR_LE / 00 00 = CDR_BE   ← Fast DDS 加的
[2,4)    encapsulation options
[4,8)    uint32 长度前缀（字节序随 encapsulation）                 ← 本方案 cdr_serialize 写的
[8,8+n)  protobuf 标准 wire format
[8+n,..) 4 字节对齐 padding
```

前 4 字节由 Fast DDS 添加，**不属于本方案**。实现 `_strip_length_prefix` 时曾漏掉这一层，
把 `00 01 00 00`（小端 256）当成长度，导致每帧误报「载荷过短」。已记录在 `echo.py` 注释中。

`cdr_serialize` / `cdr_deserialize` 只做「长度前缀 + 裸字节搬运」，不逐字段编解码。
因此 **protobuf 的线格式就是 DDS 载荷本身**，天然跨语言。

---

## 3. 需要替换什么

### 3.1 消息定义：7 个 `.msg` → 9 个 proto message

| 源 `.msg` | 字段数 | 对应 proto | 状态 |
|---|---|---|---|
| `ChassisLinkHealth.msg` | 14 | `chassis.proto` `ChassisLinkHealth` | 已实现 |
| `TrackingError.msg` | 11 | `chassis.proto` `TrackingError` | 已实现 |
| `DispatcherState.msg` | 19 | `dispatcher.proto` `DispatcherState` | 已实现 |
| `VehicleLinkState.msg` | 10 | `dispatcher.proto` `VehicleLinkState` | 已实现 |
| `SafetyState.msg` | 7 | `safety.proto` `SafetyState` | 已实现 |
| `TopologyState.msg` | 6 | `safety.proto` `TopologyState` | 已实现 |
| `MissionEvent.msg` | 7 | `safety.proto` `MissionEvent` | 已实现 |

另有 2 个共享结构：`common.proto` 的 `Time` 与 `Header`（替代 `std_msgs/Header`）。

**语义变更点（不是简单照搬）**：

- `ChassisLinkHealth.tier`：`.msg` 里是 `string`，proto 改为 `enum Tier`，
  另加 `tier_text` 保留无法映射的原始字符串。好处是拼写错误编译期就暴露。
- `ChassisLinkHealth.command_delivery_rate`：`.msg` 用 `-1` 表示「不适用」。
  proto3 无符号类型不能表达负数，故拆为 `command_delivery_rate_valid` + `command_delivery_rate`。
  **桥接代码负责翻译，下游消费者也必须改判断逻辑。**
- `SafetyState.severity`：同 `tier`，`enum` + `severity_text`。
- `Header` 是**重新定义**而非复用 `std_msgs/Header`。刻意为之：让 proto 类型不依赖任何
  ROS 消息包，从而在纯 protobuf 环境（如嵌入式端、工具链）中也能用。

### 3.2 C++ 源码：真实引用点

以下为 grep 实测结果（已排除 `install/`、`build/`）：

| 消息 | 引用处数 | 主要文件 |
|---|---|---|
| `MissionEvent` | 73 | `ros/dispatcher_node.*`、`core/workflow/*`（见下方警告） |
| `ChassisLinkHealth` | 34 | `ros/chassis_driver_node.*`、`link_health_publisher.cpp`、`safety_gate_node.*` |
| `SafetyState` | 31 | `ros/safety_gate_node.*`、`core/workflow/mission_safety_workflow.*` |
| `TrackingError` | 26 | `ros/path_tracker_node.*`、`navigate_sequence_bridge_node.*` |
| `DispatcherState` | 14 | `ros/dispatcher_node.*`、`tools/dispatcher_cli.cpp`、`rest_gateway.cpp` |
| `TopologyState` | 10 | `ros/dispatcher_node.*`、`dispatcher_visualizer_node.cpp`、`safety_gate_node.*` |
| `VehicleLinkState` | 2 | `ros/dispatcher_node.cpp` |

> **重要警告：`amr_dispatcher_core` 里也出现这些名字，但那是同名 struct，不是 ROS 消息。**
> 已核实：`amr_dispatcher_core/package.xml` **不依赖** `amr_dispatcher_interfaces`，
> 符合 ADR-001「核心算法与 ROS 完全解耦」。
>
> 因此迁移时**绝对不要动 `amr_dispatcher_core`**。真实要改的是
> `amr_dispatcher_ros`（节点层）与 `amr_dispatcher_tools`（CLI / REST）。
> 把「同名」误判为「同类型」是这次迁移最容易犯的错误，且编译期不一定报错
> ——两者字段恰好相似时会出现隐式转换成功但语义错误的静默 bug。

### 3.3 构建与依赖

| 项 | 变更 |
|---|---|
| `amr_dispatcher_ros/package.xml` | 加 `<depend>amr_dispatcher_proto</depend>` |
| `amr_dispatcher_ros/CMakeLists.txt` | `find_package(amr_dispatcher_proto REQUIRED)` |
| `amr_dispatcher_tools` | 同上（REST / CLI 输出需要 proto 类型） |
| 部署镜像 | 加 `libprotobuf-dev`（构建）与 `libprotobuf`（运行）+ `pybind11-dev` |

`amr_dispatcher_interfaces` **保留不删**。服务/动作仍完全依赖它，且过渡期要双发。

### 3.4 不改的东西（明确划界）

- `amr_dispatcher_core`：零改动（ADR-001）
- 24 个 `.srv`、2 个 `.action`：零改动（见 §5.1）
- 第三方消息（`std_msgs` / `nav_msgs` / `geometry_msgs` / `sensor_msgs` /
  `visualization_msgs` / `nav_msgs/OccupancyGrid`）：零改动。
  实测代码中 `std_msgs::msg::Bool` 用了 8 次订阅 + 4 次发布，
  这些标准化程度高、跨语言不是问题，不值得迁移。

---

## 4. 迁移策略：为什么不能「一次性切换」

### 4.1 DDS 类型名与兼容性

protobuf 类型的 DDS type name 由 `message_namespace_` + `message_name_` 决定
（在 `callbacks` 里设置），与 rosidl 生成的名字**不同**。
XTypes 类型匹配基于 type name + type hash，因此：

- 新旧两端**无法互通**：一个 protobuf 发布者和一个 rosidl 订阅者不会匹配
- 不存在「兼容解码」：类型不同就无法通信，与编码格式无关

### 4.2 双发过渡（推荐）

每个迁移的消息，过渡期内**同时发布两个话题**：

```
/chassis/link_health          ← 既有 rosidl 话题，保持不动
/chassis/link_health_proto    ← 新增 proto 话题
```

订阅方按自己的节奏迁移。全部迁完后删掉旧话题与旧 `.msg` 字段。

优点：任何时刻系统都是可回滚的；不影响现有调试流程与既有测试。
缺点：过渡期带宽翻倍（但 proto 话题本身更省，净增量有限）；发布端多一份代码。

### 4.3 全量切换（不推荐，但在小系统可行）

一次性改完所有生产者和消费者。省掉双份代码，但**没有并行验证期**，
一旦某个字段语义没对齐，只能整体回滚。本项目有 7 消息 + 24 srv + 2 action
+ 多个节点，风险偏高。

---

## 5. 边界与限制（不可回避）

### 5.1 服务与动作：明确不支持

服务（`.srv`）和动作（`.action`）的 typesupport 不是单个 handle，而是
request / response / event 的三元组结构。本方案的注入点只覆盖单个消息类型。

**结论：24 个服务、2 个动作永久保留 rosidl，不迁移。**
这部分占接口总数（33 个）的 79%。所以在可预见的未来，`amr_dispatcher_interfaces`
必须继续存在，本方案是「话题层可选替换」而非「整体替代 rosidl」。

### 5.2 工具生态：`ros2 topic echo` 不可用

根因链条（已逐层验证）：

```
echo.py
  → ros2topic.api.get_msg_class()
    → rosidl_runtime_py.utilities.get_message(type_name)
      → import_message_from_namespaced_type(...)      # 按名字 import Python 模块
        → is_message(interface) 判断 hasattr(interface, 'SLOT_TYPES')
```

protobuf 类型没有 rosidl 的 Python 生成包，`import` 必然失败。
这是**架构性**的：`ros2 topic echo` 依赖 Python 侧用同一份生成代码还原消息结构，
而 protobuf 走的是描述符池动态解析。写一个 echo 插件也绕不过去，
因为它不提供「换一个解析器」的钩子。

**已给出的解法：`ros2 amr echo`**

不试图修复 `ros2 topic echo`，而是新增一个 ROS CLI 命令组：

```bash
ros2 amr echo /chassis/link_health_proto amr_dispatcher.proto.ChassisLinkHealth --count 2
ros2 amr echo --list-types          # 列出全部可解析类型
```

实现要点：
1. 用 rclpy `raw=True` 订阅，直接拿 DDS 载荷 bytes，不构造 Python 消息对象
2. 从 `share/amr_dispatcher_proto/proto/descriptors.pb`（构建期生成）建描述符池，
   运行期**不需要 protoc**
3. `ParseFromString` 后用 protobuf 文本格式输出，字段名与 `.proto` 完全一致

其它工具的影响：

| 工具 | 状态 | 说明 |
|---|---|---|
| `ros2 topic echo` | ✗ | 架构性不可用（见上） |
| `ros2 topic hz` / `bw` | ✓ | 只看时间戳与字节数，不看内容 |
| `ros2 bag record` | △ | 能录，回放出来是裸 `uint8[]`，需插件才能还原 |
| RViz 可视化 | ✗ | 需要专门的 RViz 插件 |
| `rqt` 系列 | ✗ | 依赖 introspection 反射信息 |

### 5.3 反射信息不完整

合成的 introspection 只有名称、没有字段表。任何依赖反射的工具
（rqt、通用可视化、动态重组）都无法工作。这是 §2.2 权衡的直接后果：
为了让 rcl 不崩，必须提供 introspection；为了简单，返回的是最小实现。

若将来需要支持 rqt，可以生成**完整的** `MessageMembers`（字段名、类型、偏移都能从
`FileDescriptor` 推出）。工作量中等，但纯属可选增强。

### 5.4 其它已知限制

- **不支持嵌套超过 100 层的消息**：`max_serialized_size` 返回
  `ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE`，即无界类型。DDS 侧没有大小上限保护。
- **单帧上限 4 GiB**：长度前缀是 uint32。对遥测话题远远够用。
- **`max_serialized_size` 返回 0 且标记为 unbounded**：某些 DDS 实现可能对无界类型
  有额外限制（如 Fast DDS 的碎片化阈值）。当前 249 帧实测无问题，但大规模部署前
  建议做一次压力测试。
- **type hash 的稳定性**：当前 hash 只覆盖 `FileDescriptorProto`，不含 ROS 侧的类型名。
  若将来两个包定义了同名同结构的 proto，理论上 hash 会碰撞。建议后续把
  `message_namespace_` + `message_name_` 一并纳入 hash 输入。

---

## 6. 分阶段实施计划

### 阶段 0：骨架与验证 ✅ 已完成

- [x] `amr_dispatcher_proto` 包：proto 定义、typesupport 头、构建链路
- [x] `amr_dispatcher_proto_cli` 包：`ros2 amr echo`
- [x] 冒烟测试：C++ pub/sub 往返，字段逐项校验通过
- [x] pybind11 模块：把 typesupport handle 以 PyCapsule 暴露给 rclpy
- [x] 端到端：长驻发布者 + `ros2 amr echo`，解析出完整字段

验证记录：

```
$ ros2 amr echo /proto_smoke amr_dispatcher.proto.ChassisLinkHealth --count 2
--- [1] amr_dispatcher.proto.ChassisLinkHealth ---
header {
  stamp { sec: 123 nanosec: 456 }
  frame_id: "base_link"
}
backend_name: "serial"
tier: TIER_HEALTHY
frames_total: 1000000
frames_success: 999997
frames_crc_error: 3
is_healthy: true
```

### 阶段 1：垂直切片（1 个消息，单话题）✅ 已完成

只迁 `ChassisLinkHealth`，双发 `/chassis/link_health` 与 `/chassis/link_health_proto`。

- [x] `chassis_driver_node` / `link_health_publisher` 增加 proto 发布
- [x] 保持既有 rosidl 发布不动
- [x] 用 `ros2 amr echo` 与 `ros2 topic echo` 对照验证两者字段一致
- [x] 跑现有测试套件，确认零回归

退出标准：proto 与 rosidl 两条链路数据逐字段一致；现有测试全绿。

**实施记录**：`/system/aggregated_link_health` 与 `_proto` 实测 10 帧逐字段一致
（`scripts/check_proto_dual_e2e.sh`）。注意 `/chassis/link_health` 由真实底盘
驱动发布，本阶段验证用的是 `link_health_publisher` 的聚合话题——它在同一个
`ChassisLinkHealth` 类型上走完全相同的双发路径，且不依赖串口硬件。

### 阶段 2：核心遥测全量双发 ✅ 已完成

`TrackingError`、`DispatcherState`、`SafetyState`、`TopologyState`、`MissionEvent`、`VehicleLinkState`。

- [x] 逐个消息加 proto 发布
- [x] 写一个「双发一致性检查器」：订阅两路，逐字段比对，差异即报错
- [x] 该检查器接入 CI

退出标准：所有话题双发且一致性检查在 CI 中通过。

**实施记录（与原计划的偏离，如实记录）**

1. **`VehicleLinkState` 不是独立话题。** 它只作为 `DispatcherState.vehicle_links`
   的嵌套字段存在，没有自己的发布者。因此它的双发是通过 `ProjectDispatcherState`
   的嵌套投影完成的，`.proto` 里对应 `repeated VehicleLinkState vehicle_links`。

2. **新增了一个计划里没有的层：`proto_projection.{hpp,cpp}`。**
   原计划的表述是「在每个节点里写两个 publish」。实际实施时判断：若每个发布点
   各写一遍字段映射，两路会随代码演进静默漂移，而漂移不会让任何测试变红。
   于是把 7 个消息的 rosidl→proto 映射集中到**一份**实现里，两侧从同一个上游
   结构体派生，一致性**由构造保证**；检查器退化为独立验证网（第二意见）。

3. **双发器拆成两个类**（`ProtoDualPublisher` / `ProtoLifecycleDualPublisher`）。
   `dispatcher_node` 是 lifecycle 节点，proto 侧必须跟随 `on_activate`/
   `on_deactivate`，否则会出现「节点 inactive、rosidl 静默、proto 照发」的
   假数据流。若用同一个模板参数二选一，`rclcpp_lifecycle` 头文件会被迫进入
   chassis/path_tracker/safety_gate/link_health 四个**非 lifecycle** 节点的
   编译单元。拆开后投影分派函数共用，映射仍只有一份。

4. **成本被低估。** 计划把「逐个消息加 proto 发布」当成机械工作。实际每一处
   都要处理三件事：QoS 深度对齐、lifecycle 激活态同步、per-vehicle 话题名只
   派生一次。`dispatcher_node` 单独有 18 处 `MissionEvent` 发布点，最终收敛到
   一个 `PublishMissionEvent()` 出口，而不是在 18 处各加一行。

5. **性能影响（已实测，2026-11-07 补齐）。** 原记录是「未实测」——现在有了基线与
   数字。`TrackingError` 是 20Hz、`SafetyState` 是 10Hz，双发意味着这两条链路的
   序列化次数翻倍；实测下来这个翻倍的代价**可以忽略不计**：
   `proto_dual_benchmark` 在容器内实测（iterations=50000，取中位数）：

   | 消息 | 频率 | 投影 | 编码 | 合计 p95 | proto 字节 | 每秒 CPU |
   |---|---|---|---|---|---|---|
   | ChassisLinkHealth | 5 Hz | 163 ns | 127 ns | 616 ns | 98 B | 1.4 µs/s |
   | TrackingError | 20 Hz | 157 ns | 109 ns | 494 ns | 100 B | 5.3 µs/s |
   | SafetyState | 10 Hz | 335 ns | 141 ns | 707 ns | 54 B | 4.8 µs/s |
   | TopologyState | 2 Hz | 1226 ns | 581 ns | 2998 ns | 181 B | 3.6 µs/s |
   | MissionEvent | 1 Hz | 236 ns | 153 ns | 517 ns | 78 B | 0.4 µs/s |
   | DispatcherState | 2 Hz | 1108 ns | 450 ns | 2128 ns | 281 B | 3.1 µs/s |

   **合计约 18.7 微秒 CPU/秒 ≈ 单核的 0.0019%。** 即便是负载最重的
   `TopologyState`（1.2 µs 投影 + 0.6 µs 编码），在 2Hz 下也只有 3.6 µs/s。

   边界必须一并记住：这个数字**只含投影与 protobuf 编码**，不含 rosidl 侧的 DDS
   编码（它在 `publish()` 内部、本进程外，测不到）；也**不含带宽**。所以它是
   「双发相对单发的新增 CPU 成本」，不是双发的总成本——单发本身的开销在两种模式
   下都存在，被抵消了。

   各频率取值经 `ros2 topic hz` 在运行中的系统上核对：`/safety/state` 9.999 Hz、
   `/dispatcher/state` 2.000 Hz、`/chassis/link_health` 5.000 Hz、
   `/dispatcher/topology_state` 2.000 Hz，与基准假设一致。

   由此可以给出明确结论：**CPU 不构成保留或关掉 double-publish 的理由**。
   `proto_dual_publish` 的默认值该不该改，取决于「有没有 proto 消费者」这个
   架构问题，而不是性能问题。

**已交付代码清单**

| 文件 | 作用 |
|---|---|
| `src/amr_dispatcher_proto/include/amr_dispatcher_proto/register_all_types.hpp` | 聚合 7 个类型注册，保证每个发布 TU 都能看到全部特化 |
| `src/amr_dispatcher_ros/include/amr_dispatcher_ros/proto_projection.hpp` | 投影层声明 |
| `src/amr_dispatcher_ros/src/proto_projection.cpp` | 投影层实现（7 个消息，含 -1 哨兵/枚举映射） |
| `src/amr_dispatcher_ros/include/amr_dispatcher_ros/proto_dual_publisher.hpp` | 普通节点双发器 + `ProtoTopicName` + 投影分派 |
| `src/amr_dispatcher_ros/include/amr_dispatcher_ros/proto_lifecycle_dual_publisher.hpp` | lifecycle 节点双发器 |
| `src/amr_dispatcher_ros/test/test_proto_projection.cpp` | 17 个单测：字段完整性 + 哨兵/枚举/重复字段语义 |
| `scripts/check_proto_dual_consistency.py` | 独立解码器 + 逐字段比对（不复用 echo 的解码路径）。**4c 之后休眠**，见阶段 4 |
| `scripts/check_proto_dual_e2e.sh` | 真进程 e2e。4c 前守双发一致性，4c 后守「rosidl 已停发 + proto 有数据 + 消费者默认在 proto」 |

**双发的话题（8 条）**

`/chassis/link_health`、`/system/aggregated_link_health`、`/tracking/error`、
`/safety/state`、`/dispatcher/state`、`/dispatcher/events`、
`/dispatcher/topology_state`，以及多车模式下由 `ResolveVehicleTopic` 派生的
同名 per-vehicle 话题。每条都在原话题名后追加 `_proto`。

### 阶段 3：消费者迁移 ✅ 已完成

**机制：反向投影 + 双订阅，业务代码零改动。**

新增包 `amr_dispatcher_proto_bridge`，提供：

| 文件 | 内容 |
|---|---|
| `include/.../proto_reverse_projection.hpp` | proto -> rosidl 反投影声明（7 个消息） |
| `src/proto_reverse_projection.cpp` | 反投影实现，逐字段还原三处有损点 |
| `include/.../proto_dual_subscriber.hpp` | `ProtoDualSubscriber<RosidlT, ProtoT>` + `ProtoTopicName` |
| `test/test_proto_reverse_projection.cpp` | 25 个单测 |

为什么不在消费者里直接读 protobuf：`rest_gateway` / `safety_gate` / `dispatcher_node`
合计 60+ 处字段访问，全改写的语义漂移风险高；且「还没收到过消息」这类判据
（`severity.empty()`）在 proto 侧没有对应表达。反向投影把 proto 还原成 rosidl
再交给既有回调，**业务逻辑一行不改**，且可以逐话题切回。

三处有损点必须逐条反向还原（否则往返测试会失败）：

1. `command_delivery_rate`：`valid=false` -> `-1.0`（**不是** `0.0`；
   0.0 是「命令全丢」，与「不适用」结论相反）
2. `tier`：`TIER_UNKNOWN` -> `tier_text` 非空则用原文，否则 `"UNKNOWN"`
3. `severity`：与 `tier` 同构

一个**不可逆**边界：rosidl 默认构造的 `severity` 是 `""`，而 proto enum 默认值
反投影出来是 `"UNKNOWN"`。依赖 `severity.empty()` 表达「没收到过」的判断，
必须改看各自的 `*_seen_` 布尔量（各消费者本来就有）。

**已迁移的订阅点（13 处 / 9 个文件）**

| 文件 | 话题 | QoS | 开关参数 |
|---|---|---|---|
| `dispatcher_visualizer_node.cpp` | `/dispatcher/topology_state` | 10 | `proto_dual_subscribe` |
| `link_health_publisher.cpp` | `watched_topics` 每项 | 10 | `proto_dual_subscribe` |
| `safety_gate_node.cpp` | `link_health_topic_`（per-vehicle） | 10 | `proto_dual_subscribe` |
| `safety_gate_node.cpp` | `/dispatcher/topology_state` | 10 | `proto_dual_subscribe` |
| `lifecycle_coordinator_node.cpp` | `/safety/state` | 10 | `proto_dual_subscribe` |
| `navigate_sequence_bridge_node.cpp` | `/tracking/error` | 10 | `proto_dual_subscribe` |
| `dispatcher_node.cpp` | `/tracking/error` | 10 | `proto_dual_subscribe` |
| `dispatcher_node.cpp` | `/tracking/error/{vid}` | 10 | `proto_dual_subscribe` |
| `rest_gateway.cpp` | `/dispatcher/state` | 10 | `proto_dual_subscribe` |
| `rest_gateway.cpp` | `/safety/state` | 10 | `proto_dual_subscribe` |
| `rest_gateway.cpp` | `/chassis/link_health` | 10 | `proto_dual_subscribe` |
| `rest_gateway.cpp` | `/dispatcher/events` | **20** | `proto_dual_subscribe` |
| `dispatcher_cli.cpp` | `/dispatcher/state` | 10 | `proto_dual_subscribe` |

**开关默认值一律 `false`**：发布端必须先开着 `proto_dual_publish` 才有配对话题，
否则订阅端会「订阅成功但永远收不到」——DDS 不报错，只是没有匹配的发布者。
这也是把「发布开关」与「订阅开关」分成两个参数的原因。

launch 侧提供**单个全局开关** `proto_dual_subscribe`（`full_system.launch.py`），
一键切换全部消费者。逐节点切换在迁移期是反模式：一半节点读 rosidl、一半读 proto
时两侧数据流都在，谁看谁都不一致，反而最难排查。

- [x] `amr_dispatcher_tools`（CLI / REST）改读 proto 话题
- [x] 内部订阅方（`safety_gate_node`、`dispatcher_node` 等）迁移
- [x] 每个消费者迁移后观察一个版本周期（2026-11-07 完成，见下方观察期记录。
      唯一未覆盖项：**多车模式的 per-vehicle 订阅名**，本次跑的是单车）

**阶段 3 验收记录（2026-11-06，在真实运行的系统上完成 A/B）**

代码写完不等于能用。以下是在运行中的容器里（`full_system.launch.py` 起的
7 节点系统）做的验证，不是单测：

| 验证项 | 做法 | 结果 |
|---|---|---|
| 单测 | `test_proto_reverse_projection` 25 项 + `test_proto_round_trip` 11 项 | 全过 |
| 全量回归 | `amr_dispatcher_ros` 的 ctest（含 2 个 L2 launch 契约测试） | 60/60 通过 |
| 双发 e2e | `check_proto_dual_e2e.sh`（阶段 2 段 + 阶段 3 订阅侧段） | 两段均通过 |
| **A/B 对照** | 8080 = rosidl 网关（运行中），另起 18081 = `proto_dual_subscribe:=true` | 见下 |

A/B 的做法：`docker ps` 里的生产网关 `rest_gateway_node` 订阅四条 rosidl 话题；
再起一个同名包的第二实例（`__node:=rest_gateway_proto_test`, `port:=18081`,
`proto_dual_subscribe:=true`）。`ros2 topic info -v` 确认两条订阅分别落在
rosidl 与 `_proto` 话题上：

```
/dispatcher/state        <- rest_gateway_node        (rosidl)
/dispatcher/state_proto  <- rest_gateway_proto_test  (proto)
/safety/state            <- rest_gateway_node
/safety/state_proto      <- rest_gateway_proto_test
/chassis/link_health     <- safety_gate_node
/chassis/link_health_proto <- rest_gateway_proto_test
/dispatcher/events       <- rest_gateway_node
/dispatcher/events_proto <- rest_gateway_proto_test
```

然后比对两边 `/api/v1/state` 与 `/readyz`：

- `/readyz` 四个组件（dispatcher / safety / chassis / coordinator）在 proto 侧
  全部 `ok`，`age_seconds` 持续在 0.004–0.37 秒——证明它**真的在持续收数据**，
  不是缓存了启动瞬间的一次值。这是分辨「订阅成功」与「订阅成功但收不到」的
  唯一可靠判据：DDS 不会因为没人发布而报错。
- 触发真实业务事件（`submit_order` -> `queued` -> `plan_failed`），两个网关的
  `recent_events` 内容逐条相同。**反投影把枚举、字符串、嵌套字段都还原对了。**

一个容易被误读的观察：proto 网关的 `recent_events` 一开始为空，是因为我手工
往 rosidl 的 `/dispatcher/events` 注入了消息——那条消息不经过 `dispatcher_node`
的双发路径，自然不会出现在 `_proto` 话题上。这不是缺陷，而是双发的定义：
proto 话题的内容只来自发布端的投影，第三方往 rosidl 侧直接注入不会镜像过去。
现场排查时要知道这一点，否则会误判成「proto 订阅漏消息」。

**尚未验证的（据实说明）**：`proto_dual_subscribe:=true` 下跑一整个版本周期
的行为（长时间运行下的内存、QoS 突发丢帧、per-vehicle 多车模式的实际订阅名）。
这几项都需要现场运行，不是代码变更能覆盖的；阶段 4a 正是为此留的观察窗口。

**阶段 4a 观察期记录（2026-11-07，消费者全量切到 proto 链路）**

把 `proto_dual_subscribe:=true` 做成入口脚本的 `AMR_PROTO_DUAL_SUBSCRIBE` 环境变量
（`docker/entrypoint.server.sh`）并写进 compose，重启后全部 6 个消费者切换完成。
实测结果：

| 验证项 | 判据 | 结果 |
|---|---|---|
| 消费者真的切过去了 | `ros2 topic info -v` 看订阅方落在哪个话题 | rosidl 侧 `Subscription count: 0`，`_proto` 侧 2 个订阅者 |
| 数据真的在流动 | `/readyz` 的 `age_seconds` 连续采样 | 四个组件持续 0.01–0.85 s，非缓存值 |
| 业务功能正常 | `POST /api/v1/orders` 连发 8 单 | 8×200，16 条事件（8 queued + 8 plan_failed）全部到达 |
| **QoS 突发不丢帧** | 突发前后 `recent_events` 长度 | 0 → 16，**一帧不丢**（events QoS depth 20 够用） |
| 内存无泄漏迹象 | 每节点 RSS，39 分钟 | 全系统 292.9 → 295.3 MB（+0.8%，非单调增长） |
| 字段语义正确 | `/dispatcher/state` 与 `_proto` 逐字段对照 | 一致；`chassis_command_delivery_rate: null` 证明 `-1` 哨兵反投影正确 |

一个必须记住的排查陷阱：`ros2 topic hz` **读不到 proto 话题**（它靠 rosidl
introspection，§5.2 已说明不可用），会报「does not appear to be published yet」。
这不是「没数据」，只是工具不可用。判断 proto 话题有没有数据要用
`ros2 amr echo` 或看 `/readyz` 的 `age_seconds`。

**per-vehicle 多车模式的实际订阅名仍未验证**——本次是单车模式（`vehicle_id` 为空）。
多车场景下 `ResolveVehicleTopic` 会派生 `/tracking/error/{vid}` 这类带车名的话题，
其 `_proto` 配对名没有在真实多车环境下核过。这是个明确的待办，不能算已验证。

**观察期发现的真实缺陷（已修，两处）**

1. **`ros2 amr echo` 在部署环境完全不可用。** 报
   `AttributeError: module 'google.protobuf.message_factory' has no attribute
   'GetMessageClass'`。根因：`GetMessageClass` 是 protobuf 4.22+ 的 API，而
   Ubuntu 24.04（Jazzy 基线）自带的是 4.21.x，只有旧的
   `MessageFactory().GetPrototype()`。修法是加版本兼容层
   （`descriptor_pool.py:_message_class_for`），两条路径都覆盖。
   **这个缺陷只在部署环境显形**（开发机 protobuf 更新），而它恰好击穿了
   §5.2 的承诺：`ros2 amr echo` 是唯一能读懂 proto 话题的工具，坏了等于
   整套 proto 链路在运维侧变成黑盒。

2. **镜像缺 `libprotobuf.so.32`，所有节点 exit code 127。** 详见
   `docs/DEPLOYMENT.md` 新增的排障条目。这不是本次改动引入的——Dockerfile 一直
   列着这些包，但镜像里一个都没有。它被瞒住是因为旧容器从 `docker restart`
   一路活下来（保留手工装过的库），`docker compose up -d` 重建容器时才显形。
   已在 Dockerfile 里加构建期自检（`dpkg -s ... && ldconfig -p | grep -q`），
   包没装上就让构建失败，而不是产出一个跑不起来的镜像。

### 阶段 4：清理 ⏳ 部分完成（保守 4c 已落地，完整 4c 未执行）

**当前状态：4a / 4b 已完成（2026-11-07）；4c 的保守方案已落地（2026-11-08）。**

阶段 3 的双订阅结构使清理可以分三步走，每步都可独立回滚：

| 步骤 | 动作 | 回滚方式 |
|---|---|---|
| 4a | 消费者侧 `proto_dual_subscribe:=true`，观察 | 改回 `false`，无需重新构建 |
| 4b | 发布端 `proto_rosidl_publish:=false`，只留 proto 侧 | 改回 `true`，无需重新构建 |
| 4c | 删除 rosidl **发布端**代码（`.msg` 与反投影层保留） | 需要回滚代码（不可一键） |

4a / 4b 是**参数级**切换，不涉及代码；4c 才是真正的破坏性变更。

- [x] 4a：消费者侧切 proto 并观察（2026-11-07，见上方观察期记录）
- [x] 4b：发布端支持关闭 rosidl 侧（2026-11-07，见下节）
- [x] 更新文档与运维手册（2026-11-07，`docs/DEPLOYMENT.md` §6.3 切流 runbook）
- [x] 4c：删除旧 rosidl 话题**发布代码**（2026-11-08，保守方案，见下方"4c 已落地"）
- [ ] 删除 `amr_dispatcher_interfaces/msg/` 下已迁的 `.msg`（**未执行**，见"4c 的边界"）

> **注意 4c 的生效方式与 4a/4b 不同**：4a/4b 是参数级开关，4c 之后
> `proto_rosidl_publish` 这个参数**已从代码里删除**——它不再有任何意义
> （没有 rosidl 发布器可供关闭）。运维手册里的相关操作已同步失效，见
> `docs/DEPLOYMENT.md` §6.3 改写说明。

#### 4b 已落地：`proto_rosidl_publish`

4a 期间确认了一个设计缺口：`proto_dual_publish` 只控制 **proto 侧**（这是它
本来的正确语义），而 rosidl 侧的 `create_publisher` 与 `publish` 是**无条件**
执行的。因此「只留 proto、关掉 rosidl」这个方向当时没有任何开关可表达：

```cpp
// 4b 之前：rosidl 侧无条件创建，proto 侧才受参数控制
state_pub_ = create_publisher<...>("/dispatcher/state", 10);
topology_proto_pub_ = std::make_unique<ProtoLifecycleDualPublisher<...>>(
    this, "/dispatcher/topology_state", 10, proto_dual_publish_);
```

补上来的做法**不是**在七个节点里各加一个 `if`，而是把 rosidl 侧的
`create_publisher` / `publish` 一并**收进双发器**——每个话题从「两个独立句柄」
变成「单一 owner + 单一出口」：

```cpp
// 4b 之后：两侧都归双发器，PublishAll() 是唯一出口
state_proto_pub_ = std::make_unique<ProtoLifecycleDualPublisher<...>>(
    this, "/dispatcher/state", 10, proto_dual_publish_, proto_rosidl_publish_);
...
state_proto_pub_->PublishAll(msg);   // 两侧同时投递，开关在构造参数里
```

这一点是 4b 能成为**参数级**操作的前提：只要节点还自己留着一个 rosidl
发布器，删掉双发器里的 rosidl 行就没用——节点自己那份照样在发。

| 项 | 值 |
|---|---|
| 新参数 | `proto_rosidl_publish`，默认 `true` |
| 与 `proto_dual_publish` 关系 | 相互独立。都 `false` = 一条都不发（仅测试用） |
| 覆盖节点 | chassis_driver / path_tracker / safety_gate / dispatcher / link_health_publisher |
| 不受影响 | `dispatcher_node` 的 `/plan`（`nav_msgs::Path`，无 proto 对应物） |
| 四个组合的日志 | 由双发器的 `LogStatus()` 统一播报，避免各节点文案漂移 |

**验证方式**（2026-11-07，容器内真进程 + 真 DDS）：

- `proto_rosidl_publish:=false` 起 `link_health_publisher`：`ros2 topic list`
  里只有 `/system/aggregated_link_health_proto`，rosidl 侧话题**根本不存在**。
- 同一手法验证 lifecycle 节点：`dispatcher_node` configure+activate 后，
  `/dispatcher/{state,events,topology_state}` 三条 rosidl 话题上该实例的
  writer 数均为 **0**，而三条 `_proto` 话题各有 1 个 writer。
- 反向组合 `proto_dual_publish:=false`：只有 rosidl、没有 `_proto`，证明两个
  开关互不牵连。
- 两个都关：两侧 writer 均为 0，日志报 `ERROR` 而不是谎称「仅发布 proto」。

**4b 验证过程中踩到的三个坑**（写下来是因为 4c 的验收会以更严的方式重复遇到）：

1. **`ros2 topic hz` 读不到 `*_proto` 话题**，报 `does not appear to be published yet`。
   它靠 rosidl introspection，对 proto 类型架构性不可用。判断 proto 话题有没有数据
   必须用 `ros2 amr echo <topic> <proto.full.Type>`（需要同时给 topic 和 type_name）
   或看 `/readyz` 的 `age_seconds`。**据此判「没发布」会得到完全错误的结论。**
2. **「能收到帧」不等于「被测实例在发」**。同一话题上随时可能有别的进程在发——
   `deactivate` 之后仍收到 `DispatcherState`，一度被误判成 `Deactivate()` 没生效，
   实际是容器里常驻的系统 `dispatcher_node` 在发。验证前必须用独立的
   `ROS_DOMAIN_ID` 隔离，或先看 `ros2 topic info -v` 的 writer 清单。
3. **新参数在节点构造期 `declare_parameter`，已运行的进程不认识它**。
   改完配置不重启就去 `ros2 param get`，会得到 `Parameter not set`——
   那是进程还是旧的，不是参数没配上。诊断顺序应先看启动日志的状态行。

#### 4c 已落地：保守方案（2026-11-08）

**执行的是保守 4c，不是完整 4c。** 两者的差别是整个阶段 4 里最关键的一句话：

| | 保守 4c（已执行） | 完整 4c（未执行） |
|---|---|---|
| rosidl **发布端**代码 | 删除 | 删除 |
| `proto_rosidl_publish` 参数 | 删除（已无意义） | 删除 |
| `.msg` 定义 | **保留** | 删除 |
| 反投影层（`proto_bridge`） | **保留** | 删除 |
| `ProtoDualSubscriber::use_proto=false` | **保留** | 删除 |
| 回滚方式 | 回滚代码即可，`.msg` 还在 | 需重写 `.msg` 与反投影 |

保留 `.msg` 与反投影层的**唯一理由是可回滚**：反投影函数签名依赖 rosidl 类型，
删掉 `.msg` 等于把「切回 rosidl」这条路彻底焊死。代价是仓库里留着一批
「只被测试和反投影引用、不再上链路」的消息定义——这是刻意接受的成本。

**四项改动**（缺任何一项都会交付一个静默坏掉的默认配置）：

1. **删除双发器的 rosidl 发布器**。`ProtoDualPublisher` 与
   `ProtoLifecycleDualPublisher` 现在只持有 proto 侧的一个 `Publisher<ProtoT>`。
   构造函数从 5 参降为 4 参（去掉 `rosidl_publish`）：

   ```cpp
   // 4c 之后：唯一出口，rosidl 侧没有句柄可发
   state_proto_pub_ = std::make_unique<ProtoLifecycleDualPublisher<...>>(
       this, "/dispatcher/state", 10, proto_dual_publish_);
   state_proto_pub_->PublishAll(msg);
   ```

   `PublishAll()` 这个**名字**保留（它现在等价于 `Publish()`），因为 5 个节点
   的调用点不必改动；真正的语义变化在头部注释里写明了。
2. **删除 `proto_rosidl_publish` 参数**。5 个节点里的 `declare_parameter` /
   `get_parameter` / 成员变量全部移除。这个参数在 4c 之后**没有任何合法取值**：
   没有 rosidl 发布器可供它关闭。
3. **`proto_dual_subscribe` 默认值从 `false` 翻转为 `true`（8 个消费者全部）**。
   这是保守 4c 的**必要配套**，不是顺手优化：发布端不再产生 rosidl 话题，
   若订阅端仍默认 `false`，两边各自的日志都正常，链路却永远静默——
   DDS 不会因为「话题没有发布者」而报错，订阅端只会一直等下去。
4. **改写 e2e 与运维文档**。`scripts/check_proto_dual_e2e.sh` 原先以「rosidl 与
   proto 逐字段比对」为核心断言，4c 之后**比对基准在物理上不存在了**，
   该脚本已按新语义重写（见下）。

**类名为什么还叫 `ProtoDualPublisher`**：改名要动 5 个节点的类型别名、注释与
测试，语义上零收益。头文件里已注明「Dual」是 4b 的遗留命名，现在它只发一路。

**`proto_rosidl_publish` 时代的四个组合日志表已作废**。`LogStatus()` 现在是两态：

- 正常：`发布 proto 话题 X（旧话题 Y 已于阶段 4c 停止发布）` —— `INFO`
- `proto_dual_publish:=false`：`proto 侧已关闭：本节点不发布 X…` —— `ERROR`

**验证**（2026-11-08，容器内真进程 + 独立 `ROS_DOMAIN_ID=73`）：

- 默认参数起 `link_health_publisher`：域内只有
  `/system/aggregated_link_health_proto`，7 条受管 rosidl 话题**一条都不存在**。
- `safety_gate_node` 默认参数：发布 `/safety/state_proto`，看门狗时序不受影响。
- 消费端 `ros2 node info` 的 `Subscribers` 显示
  `/chassis/link_health_proto: amr_dispatcher/proto/ChassisLinkHealth`——
  这是默认值翻转生效的直接证据。
- `colcon test --base-paths src`：1252 个测试 0 失败。
- **反证也做了**：把某个消费者的默认值临时改回 `false` 后重跑 e2e，
  脚本在阶段 3 失败（退出码 1）并打印出该消费者仍订阅 `/chassis/link_health`。
  这说明 e2e 不是恒绿的摆设。

**e2e 的新断言结构**（`scripts/check_proto_dual_e2e.sh`，仍是 CI 的一环）：

| 阶段 | 断言 | 守的是什么失效模式 |
|---|---|---|
| 1 | 7 条 rosidl 话题在空域里不存在 | 4c 没做完（发布端还留着 rosidl） |
| 2 | `link_health_publisher` 日志出现「发布 proto 话题」 | 发布器没建起来 |
| 3 | 消费者 `node info` 订阅 `*_proto` | **默认值没翻 = 全链路静默断流** |
| 4 | `ros2 topic info` 的 `Publisher count > 0` | 有话题但没 writer |

阶段 4 只能用 `ros2 topic info`：`ros2 topic hz` / `echo` 对 proto 话题不可用
（见上方三个坑的第 1 条）。

#### `check_proto_dual_consistency.py` 的处置

该脚本（455 行）的前提是「同一份数据同时存在于 rosidl 与 proto 两条路上，
逐字段比对」。4c 删掉发布端之后，**这个前提在物理上不再成立**，脚本已无法
产出任何有意义结论，也已被新 e2e 从调用链里摘除（CI 只引用
`check_proto_dual_e2e.sh`）。

**保留而不删除**，理由与保留 `.msg` 相同：它是回滚路径上的校验工具。
若将来回滚到 4b 状态（恢复 rosidl 发布端），这个脚本立刻重新可用，
且它是唯一一个不走 C++ 投影代码的**独立第二意见**（自己从 DDS 取原始字节解码）。
文件头已加注「4c 之后处于休眠状态」，避免被误当成在跑的门禁。

#### 完整 4c 仍未做：删除 `.msg` 的前置条件没有变化

以下判断与 4c 落地前**完全一致**，保守方案刻意绕开了它们：

> **阶段 4 的前置条件**：确认没有任何外部系统（上位机、第三方集成）还在订阅旧话题。
> 这是整个计划里最容易被低估的风险——rosidl 话题名对第三方是公开契约。

保守 4c **并没有解除**这个风险，它只是把回滚代价从「重写 `.msg`」降成
「回滚代码」：对外部订阅者而言，rosidl 话题**照样会停止发布**，行为上同样是
静默失联。因此本次落地的前提假设是「当前没有第三方订阅旧话题」——
若该假设不成立，需要先回滚本次提交（`.msg` 与反投影层都还在，成本低），
而不是继续往下删。

**若将来确认可以删除 `.msg`，连带影响**：

- `amr_dispatcher_interfaces` 的 7 个 `.msg` 被 `registry/generated` 的消息
  类型清单引用；删 `.msg` 要同步删对应 proto 反向依赖（`proto_bridge` 的反投影
  函数签名会失去 rosidl 类型）。
- `ProtoDualSubscriber` 的 `use_proto=false` 分支会失去编译依据——该分支
  正是「可一键切回」的实现，删除 rosidl 后双订阅器本身就没有存在意义了，
  应当整体退化为 `create_subscription<ProtoT>` + 反投影。
- `scripts/check_proto_dual_consistency.py` 届时可以直接删除（它服务的回滚
  路径同时消失），当前只是休眠。

---

## 7. 决策建议

本方案技术上可行且已验证，但**并不适合无脑推进**。建议按以下顺序决策：

**适合推进的信号：**
- 确实出现了跨语言消费需求（如 Rust 边缘节点、Go 的 Web 后端直接订阅）
- 遥测带宽成为瓶颈（先量化，别猜）
- 需要 proto 生态的工具链（schema 版本管理、代码生成）

**不适合推进的信号：**
- 团队重度依赖 rqt / RViz 排查问题——迁移后会失去这些工具
- 有第三方系统按当前话题名订阅——双发过渡期会拉长，且清理阶段有破坏风险
- 团队规模小、维护预算紧——多出的两个包、一套 CLI、一条绕开 ROS 工具链的调试路径，
  都是长期维护成本

**中肯结论：**
当前系统没有必须引入 protobuf 的硬性理由。这项工作的价值在于「打通了一条路」，
而不是「解决了当前痛点」。若决定推进，阶段 1 的双发切片风险最低，可以作为一个
低成本的可行性验证；若阶段 1 之后发现收益不明显，随时可以停在双发状态，
代价只是多维护一个 proto 包的发布逻辑。

---

## 8. 已交付的代码

### `src/amr_dispatcher_proto/`（C++ 包）

| 文件 | 作用 |
|---|---|
| `proto/*.proto` | 9 个消息定义（对应 7 个 `.msg` + 2 个共享结构） |
| `include/.../protobuf_typesupport.hpp` | 核心：typesupport 构造、四个 Jazzy 回调、CDR 编解码 |
| `include/.../register_protobuf_type.hpp` | `AMR_PROTO_REGISTER_TYPE(T)` 宏，一行完成注册 |
| `include/.../py_bindings.hpp` | `AMR_PROTO_BIND_TYPESUPPORT(...)` 宏 |
| `src/py_typesupport_module.cpp` | pybind11 模块，输出 PyCapsule |
| `scripts/gen_typesupport_bindings.py` | 从 `FileDescriptorSet` 生成绑定清单（正确处理嵌套消息） |
| `src/smoke_test.cpp` | 冒烟自检，支持 `--duration` 长驻模式 |

### `src/amr_dispatcher_proto_cli/`（Python 包）

| 文件 | 作用 |
|---|---|
| `amr_dispatcher_proto_cli/command.py` | `ros2 amr` 命令组 |
| `amr_dispatcher_proto_cli/echo_verb.py` | `echo` 子命令参数解析 |
| `amr_dispatcher_proto_cli/echo.py` | raw 订阅 + 布局解析 + protobuf 反序列化 |
| `amr_dispatcher_proto_cli/descriptor_pool.py` | 描述符池构建与缓存 |

（该包已随 `3d01e24` 提交到 git，早期版本本节写的「未提交到 git」已过期。）

---

## 9. 复现步骤

```bash
# 构建
colcon build --packages-select amr_dispatcher_proto amr_dispatcher_proto_cli
source install/setup.bash

# 冒烟自检（2 秒后自退）
ros2 run amr_dispatcher_proto proto_smoke_test

# 长驻演示 + 解析
ros2 run amr_dispatcher_proto proto_smoke_test --duration 60 &
ros2 amr echo /proto_smoke amr_dispatcher.proto.ChassisLinkHealth --count 2
```

依赖：`libprotobuf-dev`、`protobuf-compiler`、`pybind11-dev`、`python3-protobuf`。
