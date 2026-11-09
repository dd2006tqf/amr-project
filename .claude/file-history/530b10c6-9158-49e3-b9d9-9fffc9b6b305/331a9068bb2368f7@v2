---
name: arm64-build-environment
description: ARM64 裸机原生交叉编译（Clang + Bullseye Sysroot）：架构演进、glibc 2.31 约束与容器解耦
metadata: 
  node_type: memory
  type: project
  originSessionId: 530b10c6-9158-49e3-b9d9-9fffc9b6b305
  modified: 2026-10-11T08:49:28.246Z
---

WeakNet 的 ARM64 构建体系架构演进事实（2026-10-11 升级复核）：

- **彻底脱离 Docker QEMU 容器**：旧版常驻 `weaknet-arm64-dev` 容器与 `build-arm64/` 目录已彻底删除。编译过程直接在 x86_64 宿主机运行原生 Clang 18（`-j$(nproc)` 并发），全量构建耗时从 15~20 分钟降至 ~40 秒。
- **Sysroot 与 glibc 2.31 约束**：宿主机是 Ubuntu 24.04（glibc 2.39），开发板是 Debian 11.11（glibc 2.31）。通过 `cmake/toolchain-arm64-cross.cmake` 配置 `--sysroot=/opt/sysroot-bullseye-arm64 --gcc-toolchain=/opt/sysroot-bullseye-arm64/usr`，强行锁死 GCC 10 / glibc 2.31 运行时树，产出 ELF 最高符号依赖为 `GLIBC_2.30`，与目标板 100% 兼容。
- **环境初始化**：新增 `scripts/setup_arm64_sysroot.sh` 实现 sysroot 与 GNU ld 相对符号链接的秒级自动化导出配置。
- **eBPF 探针**：11 个探针通过 `board-assets/vmlinux.h` + Clang bpf 目标编译，~1.5 秒完成，彻底规避跨架构 `user_pt_regs` 错误。

相关：[[openssl-sysroot-facts]] [[bluetooth-stack-architecture]]
