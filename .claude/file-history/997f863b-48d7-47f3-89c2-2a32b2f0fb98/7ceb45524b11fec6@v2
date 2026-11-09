---
name: cloud-ota-receipt-lifecycle
description: 云端下发 OTA 的回执三段教训——假 APPLIED（trap rc=0）、服务 cgroup 连带杀死、无 CAP_CHOWN 属主锁死，及现行不变量
metadata: 
  node_type: memory
  type: project
  originSessionId: 997f863b-48d7-47f3-89c2-2a32b2f0fb98
  modified: 2026-10-20T17:55:14.804Z
---

云端 `system.ota_upgrade_url` 下发升级于 2026-10-21 闭环（gen=8/9 已验证）。首版出现过"假 APPLIED"：云端记成功、板子停在 1.0.0 且服务被停。三层根因与现行不变量：

1. **回执终态只能由升级脚本写**：调度只写 `result=IN_PROGRESS`；`SUCCEEDED` 只能在 `.committed` 之后由脚本**显式**写；EXIT 陷阱一律写 FAILED、**不看退出码**（bash 死于 SIGTERM 时陷阱里 `$?` 是上一条命令的状态，实测为 0——这是假 APPLIED 的直接来源）。exporter 的 `checkAndEmitPendingOtaReceipt()` 在 start() 与主循环 ≤5s 轮询，`IN_PROGRESS` 超时 600s 按 FAILED 上报。
2. **升级脚本必须先迁出服务 cgroup**：fork 自 weaknet-dbus-server 的脚本身处 `weaknet-server.service` cgroup（setsid 脱离不了 cgroup），`systemctl stop weaknet-server`（KillMode=control-group 默认）会连脚本一起 SIGTERM，死在翻转前。`escape_service_cgroup()` 写 `/sys/fs/cgroup/cgroup.procs` 迁到根（板子是 cgroup v2，root 可写）。
3. **服务子进程无 CAP_CHOWN**：unit `CapabilityBoundingSet=CAP_NET_RAW CAP_NET_ADMIN CAP_BPF CAP_PERFMON CAP_DAC_OVERRIDE`——tar 改不了属主（root 属主），归档根目录权限又随构建机 umask 漂移（踩过 700），radxa 被锁在 `releases/<ver>` 外（weaknet-cli Permission denied）。`normalize_release_perms()`：chown 尽力而为 + 目录 755/文件 `u=rwX,go=rX` 必做（root 改自己文件不需要 CAP_FOWNER，chmod 必成功）。

诊断入口：板端 `data/ota_upgrade.log`（子进程 stdout/stderr，最初丢 /dev/null 害排查只能侧写）；回执文件 `data/pending_ota_receipt.json`。关联 [[board-network-access]] [[cloud-rehearsal-harness]]。
