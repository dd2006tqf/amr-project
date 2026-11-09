---
name: v1-soak-campaign
description: v1 24h 长稳的活动状态、时间点与判据，以及 probe 目标故障域与断电事实
metadata: 
  node_type: memory
  type: project
  originSessionId: 9f6f73c9-45d6-45d3-ada8-bae10cd4eed4
  modified: 2026-09-14T11:14:31.658Z
---

WeakNet Assurance Model v1 的 24h 长稳验收。

**当前活动：分段累计 24h（口径已由用户确认）**

用户于 2026-09-14 确认：断电前的 3.69h 数据**可作为有效证据计入累计**，
不必作废（分段留存是长稳常见做法，记录断点是纪律，丢弃数据不是）。

- 段1：2026-09-14 04:15:22–07:56:37 UTC，3.69h，PID 2133，已归档 `soak_3h69_prepowerloss.jsonl`
- 段2：2026-09-14 08:01:20 UTC 起，PID 511，`soak_24h.jsonl`（进行中）
- **累计满 24h 预计**：2026-09-15 04:19:56 UTC（本地 12:19）
- 6h 门禁预计：2026-09-14 10:19:56 UTC（本地 18:19）
- 工具：`tools/soak_monitor.py` 采集、`tools/analyze_soak.py` 合并审计、`tools/verify_v1_contract.sh` 契约门禁

**合并审计用法**（分析器支持多段）：
```
python3 tools/analyze_soak.py \
  --input soak_3h69_prepowerloss.jsonl \
  --input soak_24h.jsonl --min-hours 24
```
分析器的四条分段纪律（都已实现，勿退化）：
1. 时间轴必须用 `normalize_timeline()` 的 `_t_h`，原始 `uptime_sec` 分段会回绕
2. D/E/F 是段内累计计数器，必须分段算净增再求和，不能全局首末相减
3. A/B 是**进程级指标**，分段后基线会跳变，必须用 `seg_slopes()` 分段回归
   （实测段1末FD=148、段2恒定151，混算得 +0.643/h 假增长，实际是零增长）
4. C 项 map 水位必须用 `sawtooth_stats()` 降采样后判升降段，不能逐样本比较
   （逐样本会把缓慢下降误算成不回落，实测误报"单调爬升"）
5. `--min-hours` 用 `active_h`（各段有效运行之和），不是 `span_h`（含停机跨度）

**6h 门禁已通过**（2026-09-14 11:10 UTC，累计 6.85h）：PASS=18 FAIL=0 SKIP=0。

**2026-09-14 断电事件**：07:56:37 UTC 断电，07:57 重启。3.69h 数据完整幸存
（1298 条，PID 2133 全程未变），该段审计 PASS=17/FAIL=0。

**当前定级是 Freeze Candidate，不是 FREEZE。** 只有 24h 数据落地且
17 项审计全过、被测 PID 全程未变，才能升级。

**开发板 probe 目标故障域事实**（踩过的坑）：`one.one.one.one` 与
`example.com` 在本网络都解析到 Cloudflare（104.20.23.154），是**同一个
CDN 故障域**，且都不可达（TCP:443 0/3、1/3）。用它们会让 https capability
持续假失败。已换成实测 3/3 稳定的三个独立 ASN：www.baidu.com（百度）、
www.taobao.com（阿里）、www.apple.com（Apple）。
教训：配置里写的域标识名 ≠ 真实故障域，必须核对解析地址归属。

**已确认的健康行为（勿误判为故障）**：
- `dns_self_endpoints` 呈锯齿（峰值可达 243/256）而非贴顶，说明 30s TTL 回收生效
- 偶发 `https=UNKNOWN(insufficient_probe_targets)` 是保守语义在正常工作：
  3 目标中 1 个瞬时 TCP 抖动时，TCP 侧判 GOOD 而 HTTPS 侧诚实降级为 UNKNOWN，
  约 36s（一个探测周期）后自动恢复

相关：[[arm64-build-environment]] [[passive-provenance-coverage]]
