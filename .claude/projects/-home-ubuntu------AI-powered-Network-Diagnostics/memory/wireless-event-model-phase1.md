---
name: wireless-event-model-phase1
description: Phase 1 蓝牙设备事件领域模型——WHAT/WHY 分离、RawBtObservation→Normalizer→Canonical Event
metadata: 
  node_type: memory
  type: project
  originSessionId: d3a9e2e5-7ae5-4b9b-85f4-eefa0f94d189
  modified: 2026-09-29T13:33:33.133Z
---

2026-09-29 落地的蓝牙设备事件主线（**不是** OpenSpec change，用户明确要求不走工作流）。

产品定位已拍板为"**工业现场无线信号检测与诊断网关**"，不是蓝牙音频诊断。现有 `a2dp_media`/`audio_quality` 是 legacy 音频能力，**新主线不依赖它**，也不重构不重命名。

**Why:** 此前产品定位被现有音频代码带偏过一轮；用户多次强调 schema 要克制、不要为了泛化提前造大模型。四个已拍板决策：① `WirelessDevice` 只实例化 Bluetooth，`protocol` 留 reserved；② 1 Gateway = 1 Site，schema 留 `site_id`/`gateway_id`；③ 音频路径标 legacy 保留；④ Event 与 Incident 分离（本次只做 Event）。

**How to apply（五条硬约束，改代码前必须遵守）:**
1. **WHAT 与 WHY 分离**：`DeviceEventType` 只回答发生了什么，`DisconnectReason` 回答为什么。`hci_conn_timeout` 不产出独立事件，它是 `LinkDisconnected` 的一条证据。
2. **原始事实无损**：`raw_reason_code`（原始）与 `reason`（归一化）同时保存。
3. **事实与推断永不合并**：`reason` 是事实，`suspected_cause` 是推断。**采集层永远不填 suspected_cause**（恒 nullopt/NULL），留给 Phase 2/3 的 LinkAnomalyDetector。
4. **未采集 != 0**：`rssi_at_event_dbm` 用 `std::optional<int>` / SQL NULL。
5. **`details_json` 只放审计性证据**（`raw_evidence` 数组），需查询/聚合的字段一律升为正式列。

数据流不可跳步：`kprobe → RawBtObservation → BtEventNormalizer（时间窗合并）→ WirelessDeviceEvent → WirelessEventStore → device_events 表`。合并键是 `(gateway_id, hci_index, address_type, device_address, event_type)`——不是光看 MAC。原因定案优先级 `mgmt raw > 其余 raw > hint > Unknown`，但所有来源都进 raw_evidence。

**2026-09-30 真机验收后钉死的四个域/语义坑（改蓝牙代码前必读）：**
1. **mgmt 的 reason 是 `MGMT_DEV_DISCONN_*` 枚举（0x00..0x05），不是 HCI error**——内核 `hci_to_mgmt_reason()` 已转换过。按 HCI 码映射会把 0x02（本地断开）归成 Other。按 source 选映射：`disconnectReasonFromMgmtCode` vs `disconnectReasonFromHciCode`。
2. **addr_type 是 HCI 域 `ADDR_LE_DEV_*`（0=public,1=random,2/3=resolved），且必须配 `link_type`（ACL=0x01/LE=0x80）归类**——照抄内核 `link_to_bdaddr()`；单看 addr_type 会把 LE Random 误标 LE_PUBLIC，把 ACL 误标 LE。归类函数 `btAddressTypeFromKernel(link_type, addr_type)`。
3. **只有已建立的链路才产出断连事件**：conn 类 hook 一律读 `conn->handle`（偏移 50），handle==0 = 从未连上，丢弃——否则失败的连接尝试会刷出假"断连"（实测连不上 5 次刷 5 条）。失败尝试属 `ConnectionAttempt` 域，Phase 1 不产出。
4. **`source_detail` 长度必须 ≥25**（`mgmt_device_disconnected` 含 NUL=25），原 24 会截断 hook 名；ringbuf 结构相应 56 字节（用户态 static_assert 守住）。

**验收基线（vivo 手机 + btmon 对照全 PASS）**：一次真实断连 → 恰好 1 条 canonical 事件、3 条证据、`LE_RANDOM`、`LOCAL_HOST_TERMINATED raw=2`（≡ btmon 0x16）、hci_disconnect 证据 raw=19 ≡ btmon 命令 0x13、`suspected_cause`/`rssi` 均 NULL、失败连接尝试 0 条事件。

**Phase 1 状态（2026-09-30 用户定义）**：技术上闭环 = 真实断连 → canonical event → 持久化，**不是"对外可用功能"**（查询仍只有 sqlite 直查）。event_id 跨重启碰撞是用户确认的 P0（已修：btev_<nonce>_<seq> + 跨实例回归测试）。

**执行状态（2026-09-30 全部完成并已 push）**：CI 修复（三类真实缺陷：板测退出码被 `|| true` 吞、显示管道 SIGPIPE/解析误杀、weaknet_cli 不在 ci.sh 构建目标清单导致部署陈旧 CLI）→ 反向+正向全链验证 → 8 批提交全部推送 → 产品定位文档 `docs/产品定位-工业无线诊断网关.md` + README 一句话定位 → **phase1b 查询出口已实现并板端验收**（D-Bus `QueryDeviceEvents` / `weaknet-cli events` / `history_query_tool --events`，过滤 time/device_address/event_type/limit）。**更正**：曾误判"ci.sh 构建失败被 tee 吞掉报成功"——实际 pipefail 正确拦截（exit 2、零 ✅、未部署），误判源于后台命令包装隐藏退出码；结论不可再引用。**下一项 = Phase 2（RSSI 基线/LinkDegraded）→ Phase 3 SiteIncident → Phase 4 云端诊断**；a2dp 偏移缺陷已登记 `docs/ideas/fix-a2dp-l2cap-chan-offset.md`（🔴高，独立排期）。

**重要更正（勿再犯）**：
- "用户拍板不加产品定位文档"是错误归因——用户意见相反，要求 README 一句话定位 + 正式产品文档（设备/链路/Site、四大支柱、诊断不控制、边云职责、Bluetooth-first 边界）。
- `rssi_at_event_dbm` 保持 NULL 到 Phase 2 正式定义"断连前最近有效 RSSI"语义，**不要**为填字段回改 Phase 1。
- a2dp_media.bpf.c dst 偏移 bug（24→33）要登记成独立高严重度 issue/change，不混入主线。

相关：[[board-bluetooth-no-tracepoint]]、[[single-source-capture-principle]]、[[planning-fingerprint-immutability]]
