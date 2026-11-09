---
name: low-memory-build-constraints
description: 3.7 GB 开发机上构建必须两级限并发，否则整机 swap 颠簸导致 SSH 卡死（无 oom-kill）
metadata: 
  node_type: memory
  type: project
  originSessionId: 3a5d0363-e464-456c-ba58-94da17d7ef41
  modified: 2026-11-06T15:09:58.073Z
---

本机 3.7 GB / 4 核。构建事故（2026-11-05 连续三次）的表现是 **SSH 会话卡死超时，而 dmesg/journalctl 里一条 oom-kill 都没有**——这个"没有 OOM 记录"正是最强佐证：不是内存不够被杀，是所有进程一起等待换页（thrashing），连 sshd 都抢不到 CPU。

**两级并发都必须限，只限一级是错觉**：
- `colcon --parallel-workers` **只管包级**，包内部的 make/ninja 仍按 CPU 数开 `-j`。只设它 → 4 核机器上照样 4 个 cc1plus 同时编译。
- 必须同时 `export CMAKE_BUILD_PARALLEL_LEVEL`。真实峰值 = workers × 单 TU(~1.5–2 GB) × 1，而不是 workers²。

**推导只有一份实现**：`scripts/lib/build_workers.sh`（每 worker 预留 2 GB，被 nproc 封顶，至少 1；本机 → 1，CI 16 GB/4 核 → 4）。
- 它是**脚本不是函数库**：`source` 进去拿不到值（末尾是裸 `echo`），还会把它的 `set -u` 泄漏进调用方的 shell 选项。要**执行**并读 stdout。
- 必须**先读 cgroup**（v2 `/sys/fs/cgroup/memory.max`、v1 `limit_in_bytes`）再读 `/proc/meminfo`：容器里的 meminfo 报宿主机总量，不反映 `--memory` 限额。
- 兜底值必须是 **1**，绝不能退回 `$(nproc)`。已有三条分支各自验证过（覆盖 / 推导 / 缺脚本）。

**单进程兜底**：`scripts/gcc_mem_guard.sh` 通过 `CMAKE_CXX_COMPILER_LAUNCHER` 注入 `--param ggc-min-expand/heapsize` 压 GCC 内部 GC 堆峰值，**默认关闭**（内存充裕时是纯拖慢）。与 ccache 同用时 **ccache 必须在最外层**，否则 GC 参数进缓存键，历史缓存全失效。

**容器的第二个作用**：`mem_limit 2560m` / `memswap_limit 4g` 不是为了省内存，而是把故障模式从「整机冻结、SSH 卡死」改成「容器内进程被 OOM-kill，可见、可重试，宿主机保持可用」。

**ccache 的静默陷阱**：CMake 只在**首次配置**时把 `CMAKE_CXX_COMPILER_LAUNCHER` 环境变量作为缓存初值。`build/` 里存过空串之后再设环境变量完全不生效，且无任何报错——表现为「日志说已启用，实际一次都没走缓存」。必须用 `-D` 显式传。

**Why:** 这三条（两级限并发、cgroup 优先、launcher 必须 -D）都无法从代码或文档推导，只有实际踩过才知道；而踩一次的代价是整台机器失联。

**How to apply:** 改构建入口前先读这一条。仓库有四处构建入口（`scripts/build.sh`、`scripts/container_build.sh`、`docker/entrypoint.server.sh`、`.github/workflows/ci.yml`），并发推导必须全部走同一份 `build_workers.sh`，不要内联复制——曾经内联的那份还配了一句「check_all.sh 会比对」的假注释。

相关：[[build-entry-package-lists]]、[[protobuf-typesupport-jazzy-constraints]]
