---
name: openssl-sysroot-facts
description: OpenSSL 1.1 私有 sysroot 的正确构造方式，以及现有脚本/CMake 的实际缺陷
metadata: 
  node_type: memory
  type: project
  originSessionId: 9f6f73c9-45d6-45d3-ada8-bae10cd4eed4
  modified: 2026-09-13T13:56:12.253Z
---

OpenSSL 1.1 私有 sysroot（W1）已实测验证的事实：

**Debian 侧只能从 snapshot 的 `bullseye/main` 取**：
`https://snapshot.debian.org/archive/debian/20260912T000000Z/pool/main/o/openssl/libssl-dev_1.1.1w-0+deb11u1_arm64.deb`
SHA256 `6223f761bd4b961aa0c7c1662a6e99ab31150a1a7e6529d440428b361a9c8c4a`（已下载校验通过）。
`bullseye-security` 那条路（deb11u8）**deb 在 pool 里不存在**，加 snapshot security archive 也一样 404；而 bullseye/main 只有 deb11u1，会与镜像已装的 libssl1.1 (deb11u8) 冲突。所以脚本里"从 Packages.gz 反查 SHA256"的那套逻辑注定失败。

**解压后必须有两条 include 路径**：`usr/include` 与 `usr/include/aarch64-linux-gnu`（`opensslconf.h` 在后者）。只给前者会报 `openssl/opensslconf.h: No such file or directory`。容器内实测编译通过，`readelf -d` 的 NEEDED = `libssl.so.1.1` / `libcrypto.so.1.1`，最高 GLIBC 符号 `GLIBC_2.17`。

**现有实现的两个缺陷**（`scripts/setup_openssl_sysroot.sh` + `server/CMakeLists.txt`）：
1. 脚本从 Packages.gz 反查 SHA256 必然返回空 → `[ ! "$actual" != "$expected" ]` → 必失败；且 deb URL 拼的是 `pool/main/...`，与"Updates"注释矛盾。
2. CMake 里定义的是 `WEAKNET_OPENSSL_SSL`，判断用的却是 `WEAKNET_OPENSSL_LIB` → 条件恒假。而且两个变量从未用于 `target_include_directories` / `target_link_libraries`，探测结果是死变量。

相关：[[arm64-build-environment]]
