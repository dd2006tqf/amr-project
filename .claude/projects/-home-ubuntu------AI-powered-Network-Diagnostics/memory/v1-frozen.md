---
name: v1-frozen
description: WeakNet Assurance v1 已于 2026-09-15 由 Freeze Candidate 升级为 FROZEN；两处审计判据误报已修正
metadata: 
  node_type: memory
  type: project
  originSessionId: be424889-2efd-43c3-b73a-b1bc558b1943
  modified: 2026-09-15T04:56:36.727Z
---

WeakNet Assurance Model v1 于 2026-09-15 升级为 **FROZEN**（此前为 Freeze Candidate）。

依据：分段累计 24.32h 长稳审计（段1 `soak_3h69_prepowerloss.jsonl` 3.69h PID 2133 断电中断 + 段2 `soak_24h.jsonl` 20.64h PID 511），`analyze_soak.py` 正式审计 18/18 全绿。定级记录在 `docs/网络体验评价体系.md` 第十二节。

审计期间修正的两处**判据误报**（非产品缺陷，commit f111e06）：
1. **跨午夜回绕**：journal 时间戳不含日期，`h*3600` 秒内换算在 23:59→00:00 回绕，轮首差值出现 ~86400s 伪影（实测 86332 = 23:59:16 与 00:00:24 的误配对）。回绕差是"误配对的轮首距离"（0~2 个间隔），不是单轮间隔——剔除并计数披露，不要试图"折算成真实间隔"（折算值 68/69s 不是真间隔）。
2. **劣化链路轮耗时**：tcp capability=BAD 时每目标 TCP 连接等满超时（3s×N），轮耗时合法增长至 ~21s（轮间隔 49~51s）。判据按窗口自身 tcp capability 分档；转换窗口（下一 10s 快照已 BAD）也按劣化档。

**Why**: 长稳判据的 FAIL 必须先在板端 journal 上逐项溯源，区分"产品缺陷"与"判据缺陷"——本例 18 项中 17 项首跑即 PASS，唯一 FAIL 是判据模型太窄，不是调度器问题。

**How to apply**: 后续长稳审计若再遇 FAIL，先在板端 journal 复现该窗口的原始日志再定性；修改判据必须连同回归测试（跨午夜窗口、劣化档窗口）一起提交，并同步更新定级文档。相关 [[v1-soak-campaign]]。
