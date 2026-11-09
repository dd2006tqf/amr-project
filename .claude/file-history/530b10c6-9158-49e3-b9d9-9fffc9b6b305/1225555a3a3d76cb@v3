# 端云因果双桥协同自治诊断系统实施计划 (Edge-Cloud Dual-Bridge Plan v3)

## 一、定位与架构原则 (Architecture Core)

本方案遵循严格的工业网关架构准则：
**Edge Sentinel (边缘客观事实与初筛) + Cloud Assurance (云端全局统筹与复核) + Governed Execution (受控策略执行链)**

### 核心铁律
1. **L1（事实）与 L2（推断）逻辑与持久化双重隔离**：
   - 在同一个 HTTP 遥测信封中，`snapshots` 与 `edge_ai_hypotheses` 是两个独立的顶级字段（**逻辑隔离**）；
   - 在云端持久化时，L1 存入 `network_snapshots` 表，L2 存入独立的 `network_edge_hypotheses` 表（**持久化隔离**），由受限写入路径和类型化契约共同保证。
2. **端侧推理工业级非阻塞与硬超时清理**：
   - 端侧 AI 推理必须与 eBPF 采样、SLE 评估及遥测上报主线程完全解耦；
   - 冷却键（Cooldown Key）：`gateway_id + interface + anomaly_type`（包含网关身份，且网关重启后状态重置清晰）；
   - 冷却时间：300s；
   - 并发上限：1；任务队列：有界（容量 1，新任务覆盖尚未执行的排队任务，绝不打断或产生并发占用唯一的执行槽）；
   - 15s **真正硬超时（Hard Kill）**：当子进程或 HTTP 调用满 15s 未返回时，显式执行 `kill(pid, SIGKILL)` 或强制断开 socket，绝不留下持续占用资源的僵尸后台任务。
3. **证据引用强一致性（`cloud_eligible` 双重校验）**：
   - `cloud_eligible=true` 不仅要求 `snapshot_id` 非空，更要求在上传及云端摄取时，验证 `evidence_refs` 指向的 `(device_id, network_epoch, sequence_id)` 快照**真实存在于当前批次或已入库的可信上下文中**，且关联的网关身份与接口（`interface`）严格一致。
4. **四权分立受控闭环与防循环证据**：
   - 端侧假设仅作为云端 Council 专家只读输入；云端 Council 严禁将端侧 AI 推论作为独立于物理快照的新观测累加置信度；
   - 动作生成必须遵循 `Council 提议 → Policy 准入 → Approval 人审 (需审批时) → PendingAction 队列 → Edge Registry 白名单执行 → ActionOutcome 回流`。严禁任何 LLM 直接操纵执行队列。
5. **新旧版本部署顺序铁律**：
   - 由于 Python 云端 `NetworkTelemetryBatch` 设置了 `extra="forbid"`，**必须遵循“先部署云端（支持可选新字段并默认为空列表 `[]`），再部署边缘端”的上线顺序**，彻底杜绝边缘端先行发送新字段导致整批遥测被 422 拒绝。

---

## 二、端云数据流与契约设计（基于真实源码核对）

基于 `contracts.py`、`edge_telemetry_serializer.hpp`、`edge_telemetry_exporter.cpp`、`verify_telemetry_contract.py` 四个核心源码文件的真实实现：

### 1. 遥测上行信封 (Uplink Telemetry Wire Contract)
```json
{
  "schema_version": "network.edge.telemetry.v1",
  "snapshots": [
    {
      "device_id": "radxa-cubie-a7a",
      "display_name": "radxa-cubie-a7a",
      "sequence_id": 42,
      "network_epoch": 3,
      "config_generation": 1,
      "captured_at": "2026-10-11T13:16:20Z",
      "snapshot": {
        "interface": "wlan0",
        "assessment_profile": "INTERNET_ACCESS",
        "overall_state": "DEGRADED",
        "overall_coverage": "FULL_FOR_PROFILE",
        "display_score": 55,
        "network_health": { ... },
        "service_health": { ... },
        "warnings": [],
        "primary_issue": "High latency or excessive jitter",
        "link_type": "WIFI_2_4G"
      }
    }
  ],
  "edge_ai_hypotheses": [
    {
      "hypothesis_id": "eh-39c263a96272",
      "device_id": "radxa-cubie-a7a",
      "network_epoch": 3,
      "sequence_id": 42,
      "interface": "wlan0",
      "source": "EDGE_LOCAL_LLM",
      "model_id": "qwen2.5-0.5b-instruct-q4_k_m",
      "summary": "1. 网络信号强度低，导致弱信号干扰。\n2. 高并发流量导致TCP拥塞，影响连接数。",
      "evidence_refs": ["snap-radxa-cubie-a7a-3-42"],
      "recommended_actions": [
        "PROBE_LOCAL_GATEWAY",
        "CHECK_CONNECTION_BANDWIDTH"
      ],
      "status": "UNVERIFIED",
      "created_at": "2026-10-11T13:16:20Z",
      "ttl_seconds": 300,
      "cloud_eligible": true
    }
  ]
}
```
* **关联身份校验**：每条 `EdgeAiHypothesis` 显式携带 `(device_id, network_epoch, sequence_id, interface)`，与 `NetworkDeviceTelemetry` 的唯一主键 `(tenant_id, device_id, network_epoch, sequence_id)` 形成严格外键/引用映射。

---

## 三、严格阶段划分与实施任务清单 (T0 ~ T6)

### T0: 基线审计与未提交资产盘点 (已完成)
- [x] 审计工作区，固化 `b0af090` 基线提交；
- [x] 核对 `edge_telemetry_exporter.cpp:435-464` 的 `buildRecord` 与 `signBody` 源码：确认 C++ 先拼接完整 JSON `body`，再对 `body` 原文字节执行 `EVP_DigestSign`。

### T1: 端侧结构化输出规范化 (已完成真机验证)
- [x] 应用层权威生成元数据、动作白名单过滤（`ALLOWED_ACTIONS`）；
- [x] 在开发板完成三项实测（正常推理 11.3s、证据绑定、异常端口 9999 优雅降级）。

### T2: 类型化 EdgeAiHypothesis 契约与跨语言验证
- [ ] **Python 云端（先行修改）**：在 `LMA/src/industrial_ops_agent/network_assurance/contracts.py` 中定义 `_ClosedModel` 子类 `EdgeAiHypothesis`（严控字段长度、正则与动作白名单），并在 `NetworkTelemetryBatch` 中新增 `edge_ai_hypotheses: list[EdgeAiHypothesis] = Field(default_factory=list, max_length=4)`，增加跨字段 validator 验证 `hypothesis.(device_id, network_epoch, sequence_id, interface)` 必须匹配同批次中的某条 `snapshot`；
- [ ] **C++ 边缘端**：在 `server/include/assurance/edge_telemetry_serializer.hpp` 与 `server/src/edge_telemetry_exporter.cpp` 的 `buildRecord` 中序列化 `edge_ai_hypotheses` 数组，确保被 `signBody(out->body)` 原文覆盖；
- [ ] **跨语言契约守卫**：扩展 `tools/verify_telemetry_contract.py`，增加第 5 项检查 `_check_edge_ai_hypothesis_contract`，验证 C++ 发射字段与 Python `EdgeAiHypothesis` 完全对齐，以及新旧版本双向兼容。

### T3 (Gate A): 端侧后台异步调度器、硬超时终止与主流程隔离
- [ ] **异步调度器**：在 C++ 服务端实现有界队列（容量=1，并发=1）；当队列已有待处理任务时，新异常覆盖等待槽，绝不并发拉起多个推理进程；
- [ ] **冷却键与去重**：冷却键定义为 `device_id + ":" + interface + ":" + primary_issue`，冷却窗口 300s；
- [ ] **15s 硬超时清理（Hard Kill）**：采用 `fork/exec` + `waitpid(WNOHANG)` 轮询，满 15s 立即发送 `SIGKILL` 并回收子进程句柄，确保绝不留下僵尸进程或后台占满 CPU 的挂起任务；
- [ ] **Gate A 验收**：验证模型停服、超时强杀、300s 防抖及 telemetry 主流程零阻塞。

### T4: 云端接收、独立表持久化与快照关联校验
- [ ] **独立建表**：在 LMA `persistence/models.py` 及 Alembic 迁移中新增 `NetworkEdgeHypothesisRecord`（表名 `network_edge_hypotheses`），包含唯一约束 `UNIQUE(tenant_id, device_id, hypothesis_id)` 及快照关联索引 `(tenant_id, asset_id, network_epoch, sequence_id)`；
- [ ] **摄取与关联验证**：在 `network_assurance/service.py` 的 `ingest_batch` 中，验证每条 `cloud_eligible=True` 的假设所引用的快照已真实入库且接口一致，随后写入 `network_edge_hypotheses`（严禁污染 `network_snapshots` 表）。

### T5 (Gate B): 云端 Council 只读接入、防循环证据与策略边界
- [ ] **Council 只读上下文注入**：在 `network_council_service.py` 的 `build_request_from_tables` 中，查询未过期的（`created_at + ttl_seconds > now`）端侧假设作为附加参考上下文；
- [ ] **防循环证据硬约束**：在 Council Prompt 与 Coordinator 校验逻辑中明确规定：端侧 AI 假设仅作为“待验证线索”，不能作为独立于 L1 快照的第二份证据累加置信度；
- [ ] **坚守四权分立**：严禁端侧假设或 Council 直接写 `network_pending_actions`，必须完整经过 `RiskPolicy` 准入与 `Approval` 审批；
- [ ] **Gate B 验收**：单元测试与集成测试证明快照关联正确、过期假设被过滤、无循环证据累加、未经审批绝不入队。

### T6 (Gate C): 真机端云全链路联调与故障注入验证
- [ ] **部署顺序执行**：先重启/更新云端 LMA 服务，再通过 `BOARD=board ./tools/ci.sh --skip-push` 部署边缘开发板；
- [ ] **全链路 E2E 与故障注入**：在开发板上完成真实弱网触发 → 端侧异步生成假设 → Ed25519 签名上报 → 云端独立落表 → Council 参考 → Policy/Approval 审批 → 下行执行与 `ActionOutcome` 回流。
