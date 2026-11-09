---
name: clean-build-only-failures
description: 干净工作区首次构建+全量测试才暴露的故障（install 漏项、CI 不跑 test），开发机增量构建全绿
metadata: 
  node_type: memory
  type: project
  originSessionId: e1788e8c-1686-4b78-b185-8f8ed807d83c
  modified: 2026-11-08T10:57:22.661Z
---

本项目的「干净环境」验证必须包含 **`colcon build` + `colcon test` 两步**，只做 build 会漏掉整类故障。

**2026-11-08 实例：`libamr_dispatcher_proto_projection.so` 未安装。**

`src/amr_dispatcher_ros/CMakeLists.txt` 的 `install(TARGETS ...)` 列表里只有 `amr_chassis_hardware_interface`，漏了同目录下 `add_library` 出来的 `amr_dispatcher_proto_projection`。后果：

- 该 `.so` 在 `build/amr_dispatcher_ros/` 下**确实生成**，所以编译阶段全绿；
- `dispatcher_node` / `safety_gate_node` 通过 DT_NEEDED 依赖它，运行时 `error while loading shared libraries: ... cannot open shared object file`，进程 exit 127；
- `test_lifecycle_contracts` 与 `test_supervision_contracts` 整组失败（7 项），报错却表现为「夹具里没找到 dispatcher_node 进程」「safety_gate_node exited with code 127」——**指向的是测试夹具，不是缺库**；
- `test_proto_projection` 单测一直通过，因为它链接 build 树里的那份，绕过了 install 检查。

**为什么长期潜伏（最关键的部分）：**

1. `.github/workflows/ci.yml` 只跑 `colcon build`，**从来没有 `colcon test`**。全仓库 1252 个测试在 CI 里一个都没执行过。
2. 开发机 `install/` 里有历史产物，增量构建不会触发安装规则的重扫。
3. 报错形态是「测试夹具找不到进程」，第一反应会去查夹具和 ROS 域，而不是 `ldd` 看缺哪个 `.so`。

**How to apply:**

- 新增 `add_library` / `add_executable` 后，**立刻检查是否有对应 `install(TARGETS ...)`**。可执行文件缺安装只在 `ros2 run` 时才炸，共享库缺安装会让所有链接它的节点 127。
- 看到 l2 launch 测试报「找不到某节点进程」或「exited with code 127」，**先 `ldd <可执行文件> | grep 'not found'`**，别先怀疑测试夹具或 ROS_DOMAIN_ID。
- 判据是 `colcon test-result` 的 `failures` 计数，不是 `colcon test` 的退出码（后者在有失败时仍可能为 0）。
- 干净克隆验证的完整流程：`git clone` → `colcon build` → `colcon test` → `colcon test-result`。缺最后一步等于没验。

**CI 缺口（尚未修）**：`ci.yml` 的 `build-ros2-jazzy` job 和 `e2e-fleet` job 都只 build。补一个 `colcon test && colcon test-result --verbose` 步骤能永久堵住这类缺陷，但 l2 launch_testing 用例耗时较长，需要评估 CI 时间预算。

相关：[[build-entry-package-lists]]、[[deploy-only-failure-modes]]、[[low-memory-build-constraints]]
