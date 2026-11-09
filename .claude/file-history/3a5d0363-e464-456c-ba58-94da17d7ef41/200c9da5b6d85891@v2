---
name: deploy-only-failure-modes
description: 两类只在部署环境显形、开发机全绿的故障（protobuf API 版本、镜像缺 .so），及为什么它们能长期潜伏
metadata: 
  node_type: memory
  type: project
  originSessionId: 3a5d0363-e464-456c-ba58-94da17d7ef41
  modified: 2026-11-07T04:26:38.872Z
---

本项目踩过两次「开发机全绿、部署环境直接瘫」的故障，形态完全不同但潜伏机制相同：**验证链路里缺了会失败的那一环**。

**故障 1：`ros2 amr echo` 在部署环境不可用。**
`GetMessageClass` 是 protobuf **4.22+** 的 API，Ubuntu 24.04（Jazzy 基线）自带 **4.21.x**，只有旧的 `MessageFactory().GetPrototype()`。旧路径必须显式传 descriptor **自己的**池，用默认全局池会报 `not in the same pool`。
后果不是「功能不好用」——`ros2 amr echo` 是**唯一**能读懂 `*_proto` 话题的工具（`ros2 topic echo` 对 proto 类型架构性不可用），坏了等于整套 proto 链路在运维侧变成黑盒。

**故障 2：镜像缺 `libprotobuf.so.32`，所有节点 exit code 127。**
Dockerfile 一直列着 `libprotobuf-dev` / `protobuf-compiler` / `python3-protobuf`，但镜像里一个都没有。

**为什么都能潜伏很久 —— 这是最该记住的部分：**

- 故障 1 潜伏是因为 `amr_dispatcher_proto_cli` **零测试**。修法不是只改 API，而是补一个用 `monkeypatch` 摘掉 `GetMessageClass` 的用例——否则测试在新 protobuf 的 CI 上仍然全绿，漏掉的就是同一个 bug。
- 故障 2 潜伏是因为 `docker restart` 保留容器可写层（里面有手工装的库），而 `docker compose up -d` 重建容器会丢弃它。**「昨天还好好的，今天一 up 就全挂」是这类缺陷的典型时间线。**
- 不能靠「RUN 成功」推断「包装上了」：`apt-get install` 在包名于该发行版不存在时会以 0 退出而实际什么都没装。判据必须是 `dpkg -s ... && ldconfig -p | grep -q`。已写进 `docker/Dockerfile.jazzy`。

**How to apply:** 看到 `exit code 127` + `cannot open shared object file` 直接查镜像依赖，不要查代码。看到 Python 侧 `AttributeError: ... has no attribute 'X'` 且 X 是较新 API，先查 `python3 -c "import google.protobuf; print(google.protobuf.__version__)"`。写兼容层用 `getattr` 探测而非比较版本号——版本号形态太多（`4.21.12` / `5.28.0rc1`）且重打包后不可信。

**排查陷阱（会误判成「没数据」）：** `ros2 topic hz` 读不到 `*_proto` 话题，报 `does not appear to be published yet`。它靠 rosidl introspection，对 proto 类型不可用。判断 proto 话题有没有数据要用 `ros2 amr echo` 或看 `/readyz` 的 `age_seconds`（持续刷新 = 在收数据）。

相关：[[protobuf-typesupport-jazzy-constraints]]、[[build-entry-package-lists]]、[[low-memory-build-constraints]]
