---
name: phase11-ops-observability
description: 相位 11 的探针/守护/拓扑热生效实现要点与踩过的坑（域 92/93/94、overlay 语义、accept 快车道）
metadata: 
  node_type: memory
  type: project
  originSessionId: 6a40e643-c3c3-49a0-ae96-1b0f7e7e3a27
  modified: 2026-10-30T04:04:10.068Z
---

相位 11（生产可观测性 + 在线拓扑热生效）已于 2026-10-29/30 完成，11 个提交在 `feat/phase11-observability-topology`。以下是非显而易见的决定与坑，改动相关代码前必读：

**测试域/端口固定分配**：92 = 探针/守护/告警（REST 18100、webhook 18111），93 = 拓扑热更新（REST 18101、MQTT 18102），94 = L2 守护契约（launch_testing）。生产 42/8080/1883 绝不触碰。两套新 e2e 支持 `AMR_E2E_INNER=1`（`scripts/lib/inner_docker_shim.sh` 顶掉 docker exec/cp/ps），已在容器内实测通过。

**叠加层语义（不要改）**：封路 = `enabled=false`，单向 = `bidirectional=false` + 必要时**交换 from/to**；`map_hash` 原样拷贝——叠加层**不是**新地图版本。提交顺序是安全铁律：落盘 → 预约新锁 → 释放废弃锁 → Block → Unblock → 换内存 → 同步 planner → 下发改道。`topology_overlay_file` 为空时必须返回 `overlay_disabled`(501)。

**accept 快车道四件套**（rest_gateway）：
1. `head_complete` 与 `head_ok` 必须分开；
2. 读头用 **`O_NONBLOCK`**，**不要用 `SO_RCVTIMEO={0,0}`** —— Linux 里那是"禁用超时/永远阻塞"，实测能把 accept 线程永久卡死（全站停摆、重启也没用）；
3. `ReadRequestHead` 的 `prebuffered` 既是入参也是出参；
4. `HandleClient` 必须把 `kTimeout` 与 `kClosed` 一起静默关闭 —— 否则空 HttpRequest 会走到路由兜底，变成 `404 {"error":"not_found"}`（浏览器侧表现：控制台偶发打开成一页 JSON）。worker 慢路径的超时要用"从 accept 起算的剩余预算"。

**协调器 `MarkDispatcherUnavailable`**：`get_state` 不可用时不能直接 return（会让缓存 `active` 永久撒谎）——"容器 Up 8 days"事故的直接病根。

**`/readyz` 字段名以 `health_probe.hpp` 为准**：数组键 `components`、组件名键 `component`（不是 `name`）、liveness 是 `uptime_seconds`（不是 `uptime_sec`）。

**e2e 测试前提的坑（都是"测试错、代码对"）**：拓扑夹具的旁路必须**比主链长**且必须**两条**（只一条时封第一条先撞 `would_disconnect_topology`）；VDA 订单节点 id 只能从车辆落盘的 `raw_order` 读；kill 节点后必须 `ros2 daemon stop` 再发 lifecycle 调用；`web_console_check.mjs` 的 `Runtime.evaluate` 要检查 `res.error`（否则 CDP "Cannot find context" 被吞成 undefined，轮询看着在跑却永远看不到页面）。

**已知未处理**：半截请求（只发头不结束）要等满 5s 才静默关闭，因为 worker 在 `ReadRequestHead` 的 `EAGAIN` 分支上循环而非返回（`kTimeout` 分支在当前超时配置下不可达）。不影响正确性，仅拖慢单连接。

相关：[[vda5050-fleet-contract-rulings]]、[[resume-kb-sync-workflow]]
