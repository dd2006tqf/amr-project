---
name: build-entry-package-lists
description: 新增 ROS 包或包间依赖后，必须同步更新所有构建入口的 --packages-select 列表，否则干净部署会失败
metadata: 
  node_type: memory
  type: project
  originSessionId: 62c5f5b5-e2e0-46a3-b7d7-6fd3f687b7fc
  modified: 2026-11-06T07:22:58.566Z
---

仓库里有**四处独立的构建入口**，各自硬编码包列表，新增包或新增包间依赖时必须同步，否则只有在「干净工作区首次构建」时才暴露：

1. `docker/entrypoint.server.sh` — 生产部署实际走的路径（compose 启动入口）。两阶段构建。
2. `.github/workflows/ci.yml` — 两个 job（build-ros2-jazzy / e2e-fleet）各有一份。
3. `docker/Dockerfile.jazzy` + `scripts/container_build.sh` — 全量 `--base-paths src`，通常无需改。
4. `docs/DEPLOYMENT.md` 的验证表 + `README.md` 的 CI Gates 段 — 写着「N 个包」，数量变了要改。

**2026-11-06 的实际故障**：阶段 2 给 `amr_dispatcher_ros` 加了 `<depend>amr_dispatcher_proto</depend>`，CI 里同步了，但 `entrypoint.server.sh` 漏了。干净工作区按旧列表构建时 colcon 报：

```
Failed to find install/amr_dispatcher_proto/share/amr_dispatcher_proto/package.sh
Check that the following packages have been built: amr_dispatcher_proto
Failed  <<< amr_dispatcher_tools
Aborted <<< amr_dispatcher_bt
```

注意报错指向 `amr_dispatcher_tools` 和 `amr_dispatcher_bt`，**真凶是缺失的 proto 节点**——列出的两个包只是被依赖图连带中断，排查时极易误判成那两包坏了。

**Why**：colcon 在配置阶段就先校验 `--packages-select` 集合的依赖闭包；缺一个被依赖的包时，报错对象是「依赖它的包」，不是「缺失的包」。而开发机 `install/` 里往往早有旧产物，增量构建看不出问题——只有干净环境才炸。

**How to apply**：
1. 新增包 → 搜 `--packages-select` 和「N 个包」的文档表述，全部补齐。
2. 新增 `<depend>` → 检查所有构建入口的列表是否包含被依赖包。
3. 验证方式必须是**干净工作区**（或先 `rm -rf build/<pkg> install/<pkg>`），增量构建不算数。
4. 报错里出现 `Aborted`/`Failed to find ... package.sh` 时，先看「Check that the following packages have been built」那一行，别从报错包本身查。

相关：[[protobuf-typesupport-jazzy-constraints]]
