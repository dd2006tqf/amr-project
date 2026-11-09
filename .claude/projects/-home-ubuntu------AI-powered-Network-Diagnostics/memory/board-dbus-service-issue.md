---
name: board-dbus-service-issue
description: 板端 systemd 服务问题已解决（系统总线迁移 + caps + looper 修复）；mDNS .local 跨网段不可达是拓扑限制
metadata: 
  node_type: memory
  type: project
  originSessionId: 5840bf4f-d5b8-4903-89bc-fbea4d384c84
  modified: 2026-08-31T11:37:02.101Z
---

2026-08-31 板端 systemd weaknet-server 无法启动的问题已定位并修复（提交 0d130a8 / dafc882），三层根因：

1. **root 连不上用户总线**：service 以 root 连 `unix:path=/run/user/1000/bus` 立即 NoReply（裸 dbus-send 可复现；radxa 自身正常，dbus 策略无 uid 限制，疑似 systemd 用户总线对 uid 0 的底层行为）。→ **迁移到系统总线**：server/client 均改 `DBUS_BUS_SYSTEM`，新增 `tools/com.example.WeakNet.conf` 策略文件（ci.sh 自动装到 /etc/dbus-1/system.d/ 并 reload dbus）。
2. **服务 caps 不够**：CapabilityBoundingSet 只有 NET_RAW/NET_ADMIN，eBPF 加载报 Operation not permitted。→ 补 CAP_BPF + CAP_PERFMON。
3. **looper 两个既有 bug**（系统总线上必现）：`while (read(pipe) > 0)` 阻塞读排空导致 SIGTERM 后主线程卡死 pipe_read（stop 只能等 90s SIGKILL）；`while (dbus_connection_read_write_dispatch(conn,0))` 无界循环导致单核满载。→ pipe 读端 O_NONBLOCK + 每次 POLLIN 单次派发。修复后 stop 11s、CPU 0.2%。

另外修复：DnsMonitor `dns_stats map lookup failed` —— BPF 侧首个 DNS 事件才创建聚合条目，空闲期 ENOENT 被当读取错误；init 预创建 + ENOENT 容错后 healthy=true。

**Why:** 这些结论决定板端部署拓扑与排障方向，避免重走弯路。

**How to apply:** 板端验证走系统总线（不要再用 dbus-run-session 包一层）；板端测试脚本 `sudo weaknet-test-full.sh`（自带停/恢复 systemd 实例）；ci.sh 已在部署步上传测试脚本（rsync --delete 会删 dist 外文件）。mDNS 注意事项见 [[board-network-access]]。
