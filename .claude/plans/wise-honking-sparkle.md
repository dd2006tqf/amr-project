# OTA Phase 1 本地升级 — 实施计划

## Context

为什么做：当前板端升级靠 `tools/ci.sh` 从开发机 rsync 覆盖（`--delete`），无版本概念、无原子性、无回滚。两块板以上即不可运维。OTA Phase 1 只做**板端本地闭环**（解包校验 → 原子切换 → 健康窗口 → 自动回滚），零云端依赖，是后续云侧制品链（Phase 2）与审批执行链（Phase 3）的前置地基，也是运维刚需。

用户已选：OTA Phase 1 本地升级（DPDK 学习实验与 ideas-only 备选已搁置）。本计划只覆盖 Phase 1；Phase 2/3 另立 change。

前置事实（只读调研已确认）：
- 板端布局：`/home/radxa/weaknet/`（server/bin、server/build/*.bpf.o、data/、config/），服务单元 `tools/weaknet-server.service`（`ExecStart=…/server/bin/weaknet-dbus-server --config /etc/weaknet/config.yaml`，`WorkingDirectory=…/server`，root + 5 capabilities）。
- `dist-arm64/` 结构：server/bin、server/build、server/test、client/bin、client/lib、lib（28MB，zstd 后约 10MB）。
- 升级必须原位保留的 5 个状态文件：`data/history.db`、`data/network-epoch`、`data/wireless-uplink-state`、`data/runtime-overrides.bin`、`data/config_txn_state`（路径解析 `server/include/common.hpp`）。
- 可复用模式：`ConfigTransaction` 180s 试用看门狗（`server/include/weaknet_config.hpp:343`、`server/src/weaknet_config.cpp:1073`，tmp+rename 持久化 + 开机崩溃回滚）；板端下行 `EVP_DigestVerify` 验签模式（`server/src/edge_telemetry_exporter.cpp:679`）；健康判据 `HealthCheck`/`GetEbpfMonitorHealth`（`server/src/dbus_service.cpp:394,1348`）。
- 无版本来源：`config.yaml`、CMake（`project(weaknet VERSION 1.0.0)` 静态）、`WeakNetConfig` 均无固件版本键。`ai_snapshot.json` 为 idle，无 active change，可新建 change。

## 推荐方案（只写这一个，不展开备选）

**独立 shell 脚本 + CLI 薄封装 + 服务端只读状态方法**，releases 目录 + `current` 软链接原子翻转：

```
板端 /home/radxa/weaknet/
├── releases/<version>/   # server/ client/ lib/ + manifest.json（sha256 清单 + 版本）
├── current -> releases/<version>   # 原子翻转点（rename）
├── data/                 # 5 个状态文件，升级永不触碰
└── config/               # edge_priv.pem，升级永不触碰
```

1. **版本单一来源**：仓库根新增 `VERSION` 文件（一行 semver）。CMake 读入编译进二进制（`--version` 输出）；`ci.sh --release`（或本地打包命令）用它给 manifest 盖戳；服务端新增只读 D-Bus 方法 `GetUpdateStatus` 上报当前版本 + slot 状态。单调递增检查拒绝降级（防回滚攻击的本地版）。
2. **包格式**：`weaknet-<version>.tar.zst` + `manifest.json`（文件→sha256、版本、最低兼容标记）。Phase 1 做 sha256 全量校验 + 版本单调检查；Ed25519 分离式签名**验签位先占后实现**（验签函数按 `edge_telemetry_exporter.cpp:679` 的 `EVP_DigestVerify` 模式写，若发布公钥不存在则告警通过——避免 Phase 2 返工，不阻塞 Phase 1 闭环）。
3. **执行体**：新增 `tools/weaknet-local-update.sh`（`apply/status/rollback/verify` 四子命令，随包部署到板）。`weaknet-cli` 新增 `update` 子命令，**apply/rollback 直接 fork+exec 该脚本**（不停留 D-Bus——服务端不能在调用中途杀死自己），`update status` 走 D-Bus `GetUpdateStatus` 只读查询。root 限权沿用现有 `requireRootCaller` 惯例。
4. **原子切换**：解包到 `releases/<version>/` → 校验 → `ln -sfn` 到临时名 → `rename` 翻转 `current` → `systemctl restart weaknet-server`。服务单元改为经 `current` 寻址（`ExecStart` + `WorkingDirectory` 同步改，`bpf_obj` 相对路径解析随工作目录走——实现时以 `server.cpp` 实际解析逻辑为准，必要时改为绝对路径）。
5. **健康窗口 + 自动回滚**：重启后脚本轮询（默认 180s，复用看门狗窗口语义）：D-Bus `HealthCheck` 有响应 + `GetEbpfMonitorHealth` 探针 attach 数正常 + 进程存活。全绿则 commit（写 `releases/<version>/.committed` 标记）；任一失败则翻回上一版 symlink 并重启。`data/` 与 `/etc/weaknet/config.yaml` 全程只读。
6. **目录漂移清单**：任何改动 `ActionRegistry`/trial 白名单的版本，发布 checklist 必须含"云端同步审阅目录"（`docs/网关动作目录版本契约.md` fail-closed 防线，否则升级后远程动作全被自拦）。

## 关键文件

- 新增：`VERSION`（根）、`tools/weaknet-local-update.sh`、`server/src/update_status.cpp`（+ `server/include/update_status.hpp`，读 `current` symlink 与 manifest）。
- 修改：`client/weaknet_cli.cpp`（`update` 子命令分发，仿现有 `action` 命令风格）、`server/src/dbus_service.cpp`（新增 `GetUpdateStatus` 只读方法）、`tools/weaknet-server.service`（经 `current` 寻址）、`tools/ci.sh`（打包 manifest + 部署脚本；**不改部署语义**，只增产物）、`CMakeLists.txt`/`server/CMakeLists.txt`（读 `VERSION`、编译新源）、`docs/weaknet_cli_usage.md`（同步文档，项目铁律：文档与代码同步）。
- 不碰：`data/` 下 5 文件、`/etc/weaknet/config.yaml`、`config/` 密钥、`scripts/`（harness 禁改）。

## 执行路径（工作流外，按用户指示不走 OpenSpec change）

按 CLAUDE.md「工作流外任务必须留记录」铁律执行（不走 change 仍必须留记录）：

1. 改代码前先按 `test-driven-development` 写 RED 测试（gtest：manifest 校验、版本单调比较、symlink 翻转纯函数逻辑），再实现转绿；shell 部分 `sh -n` + 沙盒演练。
2. 本地验证全绿（`test-all` 48 套件不回归）后，**经你批准**再跑 `./tools/ci.sh` 部署到板做真机演练（项目规则：禁止跳过本地验证直接 push；Git 提交也只在你明确要求时执行）。
3. 完成后建 `openspec/tasks/2026-10-19-ota-local-update.md`，按 `openspec/tasks/README.md` 三节模板记录构思/实现方式/结果（该目录不受 harness 约束，但无验收效力——真机演练结论写进"结果"节）。
4. 同步文档铁律：`docs/weaknet_cli_usage.md` 补 `update` 子命令用法。

## 验证

- x86：`test-all` 全绿（新增 gtest 覆盖：manifest 校验、版本单调比较、symlink 翻转逻辑纯函数；shell 脚本逻辑配 `sh -n` + 本地沙盒演练）。
- 板端（blocking 证据）：① 同版本 apply 幂等成功；② 篡改包 sha256 被拒；③ 正常升级 commit；④ 注入坏二进制（启动即崩）触发自动回滚到上一版且 `data/` 5 文件完好、`history.db` 可读。经 `weaknet-test-full.sh` 风格冒烟 + `task_verify.sh --project-command` 留痕。
- 回归：`HealthCheck`/`GetEbpfMonitorHealth`/上行游标（`wireless-uplink-state`）升级前后连续，`catalog_version` 无漂移误报。
