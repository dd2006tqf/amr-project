---
name: vda5050-fleet-contract-rulings
description: 多车车队化与 VDA 5050 运维的接口契约裁定（执行后端分离、选车三态、强清隔离）
metadata: 
  node_type: memory
  type: project
  originSessionId: 6a40e643-c3c3-49a0-ae96-1b0f7e7e3a27
  modified: 2026-10-21T10:02:58.025Z
---

2026-10-21 起，本项目的多车化与 VDA 5050 运维层受以下契约约束（评审已否决过早期设计，改动前先读本条）：

1. **执行后端分离**：`local_sim`（调度发 `/plan/{vid}`，path_tracker/chassis 执行）与 `vda5050`（调度只发 MQTT order，车辆自带控制器）共用任务/选车/路权，**不共用运动控制链**。VDA 5050 车**不启动** tracker/gate/chassis，其"完成"由 state 报文 lastNodeId 推进判定，不是 `/tracking/error` 的 arrived。
2. **身份语义分离**：`Mission.fleet_id` = 车队归属（原义，不挪用）；`Mission.assigned_vehicle_id` = 承接车辆。
3. **选车三态**：`kAssigned` / `kDeferred`（车忙 → 任务留队，不判负）/ `kIndeterminate`（无状态报文 → 暂停自动分配）/ `kRejected`（永久不满足）。位置无效或状态超龄即**不合格**，不盲派。
4. **强清 = 隔离而非摘除**：通信失联不等于可释放路权。强清需 `operator_id` + `reason` + `confirm_vehicle_id`；执行后车辆进 `quarantined`（不参与自动分配但仍可见）；恢复需 `RestoreVda5050Link` 且操作员声明已核实位置。

**Why:** 评审明确指出"把 VDA 5050 车与本地模拟车混成一套执行链"会导致重复控制与状态串线；"通信失联即释放路权"会撞车。

**How to apply:** 新增多车/强清功能前先核对这四条；相关纯函数在 `core/fleet/vehicle_assignment.*` 与 `core/fleet/force_clear_link.*`，准入规则一律下沉核心而非写在 ROS 回调里。相关：[[vda5050-epic-status]]
