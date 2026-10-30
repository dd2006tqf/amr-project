# amr_dispatcher 告警规则

`amr_dispatcher.rules.yml` 是 Prometheus 规则文件，配套 `docs/DEPLOYMENT.md`
的探针/告警章节。指标来源是网关的 `/metrics`（见 `docs/API.md` §`GET /metrics`）。

## 规则一览

| 告警 | 表达式要点 | `for` | 严重度 | 它想抓的真实事故 |
|---|---|---|---|---|
| `AmrNotReady` | `max_over_time(amr_ready[2m]) == 0` | 1m | critical | 整套栈持续不可用 |
| `AmrDispatcherStateStale` | `amr_dispatcher_state_age_seconds > 10` | 1m | critical | **dispatcher 进程死了但容器还在**（"Up 8 days" 那次） |
| `AmrDispatcherStateNeverSeen` | `amr_dispatcher_state_age_seconds < 0` | 3m | warning | 冷启动链路坏了（域 ID / lifecycle 卡住） |
| `AmrSafetyCriticalLatched` | `amr_safety_critical == 1` | 5m | critical | 急停锁存，现场无人复位 |
| `AmrSafetyStateNeverSeen` | `amr_safety_severity_level < 0` | 3m | warning | 安全门缺席（就绪门禁永远拦单） |
| `AmrAlertChannelBroken` | `increase(amr_alert_webhook_failures_total[15m]) > 0` | — | warning | **告警通道本身**坏了 |
| `AmrAlertQueueSaturated` | `increase(amr_alert_dropped_total[15m]) > 0` | — | warning | 告警风暴或 webhook 长期不可用 |
| `AmrTopologyOverlayActive` | `amr_topology_overlay_active == 1` | 4h | warning | **施工区忘了撤**，临时改道变永久 |
| `AmrVehicleQuarantined` | `amr_vehicle_quarantined > 0` | 30m | warning | 隔离车没人恢复，运力悄悄减少 |

## 部署

```yaml
# prometheus.yml
rule_files:
  - /etc/prometheus/rules/amr_dispatcher.rules.yml

scrape_configs:
  - job_name: amr_dispatcher
    scrape_interval: 15s
    static_configs:
      - targets: ["amr-host:8080"]
    # 若网关配了 metrics_token（回退 auth_token），抓取端需携带：
    # authorization:
    #   credentials: <token>
```

## 刻意不做的事

- **没有任何规则触发 restart / autoheal。** `/readyz` 为 503 可能是
  "健康但安全锁存"（急停未解除）。因 503 重启容器会拆掉整套栈、丢掉调度队列
  与在途任务的路权状态，把一个"需要人到场复位"的场景变成"反复重启 + 反复丢单"。
  探针与告警的作用是**让人知道**。
- **没有按边名打标签的规则。** 拓扑指标只给总量（`amr_topology_blocked_edges` 等），
  标签基数不随地图规模增长。要看具体哪条边，走
  `GET /api/v1/topology/overlay`。
- **没有"告警条数"类规则。** 告警事件是状态快照，只计数不重发；
  恢复通知每次故障恰好一条（状态机不变量由 `test_alerting.cpp` 与
  `check_health_supervision_e2e.sh` T5 双重锁定）。
