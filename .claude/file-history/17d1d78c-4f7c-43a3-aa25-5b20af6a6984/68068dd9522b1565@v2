---
name: dbus-dispatch-drain-semantics
description: Looper 派发层三条硬约束（循环派发/不能只看 POLLIN/64 轮上限）与 dbus-send 测延迟陷阱
metadata: 
  node_type: memory
  type: project
  originSessionId: 17d1d78c-4f7c-43a3-aa25-5b20af6a6984
  modified: 2026-11-02T05:42:34.137Z
---

2026-11-01 修复的 D-Bus 方法调用 P99 抖动（最坏 25s，呈 13ms/300ms/1000ms/3000ms 离散阶梯），根因在 `server/src/looper.cpp` 派发层，不是处理逻辑。形成的三条硬约束必须保持，回归测试 `server/test/unit/test_looper_dispatch_gtest.cpp`（4 用例，自建私有 dbus-daemon，不依赖会话总线）：

1. **派发必须循环** — `dbus_connection_read_write_dispatch()` 一次只派发**一条**消息。积压 8 条时调一次回调只 +1，`dispatch_status` 仍为 `DATA_REMAINS`。故有 `dispatchUntilDrained(conn, kMaxDispatchRounds)`。
2. **不能只看 POLLIN** — 内部队列有积压但 socket 已读空时 `poll()` 不再报可读（实测 300ms 超时返回 0）。主循环每轮都派发，poll 超时分支还要查 `dispatch_status == DATA_REMAINS` 主动排空。
3. **必须有轮次上限** — `kMaxDispatchRounds = 64`。无界 while 会饿死 self-pipe 停止信号，SIGTERM/SIGINT 失效，systemd 停服只能等 `TimeoutStopSec` 后 SIGKILL。上限测试断言"恰好处理 64 条"，不能只断言"还有积压"（会被单次派发假通过）。

**测量陷阱**：用 `dbus-send` 子进程逐次调用测延迟会混入每次 fork+exec+总线认证的固定 11~13ms，把健康服务测成超标。必须复用单条连接（`tools/interference_test.py` 已改，保留 `--legacy-cli` 对照）。修复后板端 P99：空载 1.33ms、干扰负载 4.04–6.14ms。

**排障顺序**：再遇到方法调用抖动，先看 `dispatch_status` 是否长期 `DATA_REMAINS`，再怀疑处理逻辑——派发层已不是可疑点。

相关：[[single-source-capture-principle]]（另一条"先证据后结论"的实测类教训）
