---
name: arm64-container-binfmt-reset
description: 宿主重启后 weaknet-arm64-dev 容器启动报 exec format error，需重新注册 qemu binfmt 再 docker start
metadata: 
  node_type: memory
  type: project
  originSessionId: 9d6ab6dd-2294-42dd-a2f0-cacbcc00538d
  modified: 2026-09-10T07:02:54.867Z
---

2026-09-10：宿主机（x86_64 CentOS）重启后 `/proc/sys/fs/binfmt_misc/` 下 qemu-aarch64 注册丢失，`weaknet-arm64-dev` 容器 `docker start` 后 2 秒退出（exit 255，logs 显示 `exec /bin/sleep: exec format error`）。

**Why:** binfmt_misc 注册不是持久的，容器依赖 QEMU 用户态模拟 ARM64。

**How to apply:** 遇到容器 started→immediately exited 时先看 `docker logs weaknet-arm64-dev`；修复命令：
`docker run --rm --privileged multiarch/qemu-user-static --reset -p yes`（镜像本机已有），然后 `docker start weaknet-arm64-dev`，用 `docker exec weaknet-arm64-dev uname -m` 验证输出 aarch64。

相关：[[board-network-access]]
