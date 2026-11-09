---
name: verification-traps-multiwriter
description: 验证 ROS 话题行为时的四个反直觉陷阱：多 writer 混淆、proto 话题工具不可用、新参数需重启才可见、图查询会漏报且 --no-daemon 不是万能药
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3a5d0363-e464-456c-ba58-94da17d7ef41
  modified: 2026-11-08T08:36:23.028Z
---

验证「某节点有没有发布/停止发布」时，有四类情况会让结论**完全反过来**。前三个是阶段 4b 实测踩到的，第四个是 2026-11-08 补测多车 per-vehicle 链路时踩到的。

**1. 「能收到帧」≠「被测实例在发」。**
`deactivate` 一个 lifecycle 探针后，`ros2 amr echo` 仍收到 `DispatcherState`，差点判成 `Deactivate()` 失效。真相是话题上有两个 writer——容器里常驻的系统 `dispatcher_node` 一直在发。
**Why:** DDS 话题是多方合并的，`echo` 收到的是所有 writer 的并集，无法区分来源。
**How to apply:** 验证前先用 `ROS_DOMAIN_ID=<空闲号>` 隔离，或先看 `ros2 topic info -v` 里的 `Publisher count` 与 `Node name` 清单。隔离 domain 是更可靠的做法——探针与常驻系统并存时，`topic info` 自己也会被污染。

**2. `ros2 topic hz` / `ros2 topic echo` 对 `*_proto` 话题不可用。**
`hz` 报 `does not appear to be published yet`，那是工具靠 rosidl introspection、对 proto 类型架构性不支持，不是没数据。
**How to apply:** 只能用 `ros2 amr echo <topic> <proto.full.TypeName>`——注意**必须同时给 topic 和 type_name**，只给 topic 会报「需要同时提供 topic 和 type_name」。或看 `/readyz` 的 `age_seconds` 是否持续刷新。

**3. 改配置后不重启就 `ros2 param get`，会得到 `Parameter not set`。**
新参数是在节点构造期 `declare_parameter` 的，**已运行的进程不认识它**。这不是参数没配上，是进程还是旧的。
**How to apply:** 诊断顺序永远是「先看启动日志里的状态行，再看 `param get`」。日志里我们让双发器统一播报四类开关组合（双发已启用 / proto 侧已关闭 / rosidl 侧已关闭 / 两侧都已关闭），比 `param get` 更早、更准。

**4. `ros2 topic info` / `ros2 node info` 的图查询会漏报，而 `--no-daemon` 不是万能药。**
2026-11-08 一天内踩到两个方向的漏报，先记哪个都不完整：

- **不带 `--no-daemon`**：daemon 的图缓存陈旧或与节点不同步。多车补测时 `/mv_sg` 在 `node list` 里、日志也正常，但 `node info` 的 `Subscribers:` 段和 `topic info` 全是空的，一度以为没建订阅。加 `--no-daemon` 后立刻拿到 `Publisher count: 1 / Subscription count: 1`。
- **带 `--no-daemon`**：它每次调用新建一个临时 participant，**只等一个很短的发现窗口**，窗口内没发现完就返回 `Publisher count: 0`。在同一个活着的 dispatcher 上连查 5 次 `/dispatcher/events_proto` 得到 `1,0,1,1,1`，而 daemon 模式同样 5 次是 `1,1,1,1,1`。节点日志同期完整打印了三条「发布 proto 话题」并有 `activated`，所以是查询抖动，不是发布器没建。
**Why:** 两种模式漏报的**原因不同**——daemon 是缓存陈旧，`--no-daemon` 是发现窗口太短。所以「换一种模式」只能覆盖一半情况。
**How to apply:** 任何「查询结果为空但进程确实活着」的判断，都不能依赖单次查询。写脚本时**轮询取多次的最大值**（间隔 1s，5–6 次，见到 ≥1 即停）；手工诊断时至少连查 3 次再下结论。断言「话题不应存在」时方向相反——抖动只会造成假通过不会假失败，但仍要查 3 次压低假通过率。另外：节点启动日志里的「发布 proto 话题」行比任何 `topic info` 都可信，先用它建立事实。

相关：[[deploy-only-failure-modes]]、[[protobuf-typesupport-jazzy-constraints]]
