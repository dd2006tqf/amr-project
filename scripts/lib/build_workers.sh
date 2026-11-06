#!/usr/bin/env bash
# 输出本次构建应使用的并发数（单个整数）。
#
# ============================================================================
# 为什么不再用 $(nproc)
# ============================================================================
# nproc 只看 CPU 数，完全不看内存。本项目的重灾区是模板密集的 ROS TU：
# 单个 cc1plus 峰值约 1.5–2 GB。4 核机器上 -j4 意味着瞬时峰值 6–8 GB，
# 而本项目的开发/部署机只有 3.7 GB。
#
# 关键区别在于**失败模式**：内核并没有 OOM-kill（还有 6 GB swap），进程存活，
# 但一起进入 swap 颠簸（thrashing）——所有进程都在等页换入换出，连 sshd 都
# 抢不到 CPU。表现是 SSH 会话直接卡死超时，而不是收到一条 OOM 通知。
# 2026-11-05 连续三次「构建把 SSH 打断」都是这个机制：dmesg 与 journalctl 里
# 一条 oom-kill 记录都没有，正是最强佐证。
#
# 所以这里的取舍不是「快一点还是慢一点」，而是「慢一点还是整机冻结」。
#
# ============================================================================
# 公式
# ============================================================================
#   每 worker 预留 2 GB，结果再被 CPU 数封顶，且至少为 1。
#
# 为什么用物理总量而不是 MemAvailable：
#   MemAvailable 随 IDE / 其他会话实时波动，同一台机器两次构建会算出不同的
#   并发度。构建脚本的输出不该有这种不确定性——排查构建问题时那是噪音。
#
# 为什么先看 cgroup 再看 /proc/meminfo：
#   容器里的 /proc/meminfo 报告的是**宿主机**总内存，不反映 --memory 限额。
#   若只看 meminfo，一个 --memory=2g 的容器在 64 GB 宿主机上仍会算出 8 并发，
#   实际可用只有 2 GB，照样炸。cgroup v2 的 memory.max 才是容器内的真实上限。
#   memory.max 为 "max"（未限制）时回退到物理总量——这正是当前开发机的状态
#   （HostConfig.Memory=0），两者等价。
#
# 本机 3.7 GB -> 3/2 = 1 worker；CI 16 GB / 4 vCPU -> min(4, 8) = 4，行为不变。
#
# 覆盖方式：设置 AMR_COLCON_WORKERS=N 直接跳过推导（CI 固定并发、临时压测）。

set -u

mem_kb=""

# cgroup v2：memory.max 单位是字节，"max" 表示未限制。
if [ -r /sys/fs/cgroup/memory.max ]; then
  cg_max=$(cat /sys/fs/cgroup/memory.max 2>/dev/null || echo max)
  case "$cg_max" in
    max | "" | *[!0-9]*) : ;;                 # 未限制或非法：回退到 meminfo
    *) mem_kb=$(( cg_max / 1024 )) ;;
  esac
fi

# cgroup v1：limit_in_bytes，极大值表示未限制。
if [ -z "$mem_kb" ] && [ -r /sys/fs/cgroup/memory/memory.limit_in_bytes ]; then
  cg_max=$(cat /sys/fs/cgroup/memory/memory.limit_in_bytes 2>/dev/null || echo "")
  case "$cg_max" in
    "" | *[!0-9]*) : ;;
    *)
      # v1 用 9223372036854771712 表示"无限制"
      if [ "$cg_max" -lt 9223372036854771712 ] 2>/dev/null; then
        mem_kb=$(( cg_max / 1024 ))
      fi
      ;;
  esac
fi

# 兜底：物理内存总量。
if [ -z "$mem_kb" ] || [ "$mem_kb" -le 0 ] 2>/dev/null; then
  mem_kb=$(awk '/^MemTotal:/ { print $2 }' /proc/meminfo 2>/dev/null || echo 0)
fi

mem_gb=$(( mem_kb / 1048576 ))
by_mem=$(( mem_gb / 2 ))
[ "$by_mem" -lt 1 ] && by_mem=1

cpu=$(nproc 2>/dev/null || echo 1)

if [ "$cpu" -lt "$by_mem" ]; then
  echo "$cpu"
else
  echo "$by_mem"
fi
