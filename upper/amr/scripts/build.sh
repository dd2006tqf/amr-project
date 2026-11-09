#!/usr/bin/env bash
set -euo pipefail

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
ROOT_DIR="$DIR/.."

# 并发度按**内存**推导，不再用 $(nproc)：见 scripts/lib/build_workers.sh 里的
# 完整说明（3.7 GB 机器上 -j4 会让整机进 swap 颠簸、SSH 直接卡死）。
#
# ⚠️ lib/build_workers.sh 是**可执行脚本**（stdout 输出一个整数），不是
# shell 函数库，所以必须「执行」它并读 stdout，不能 `source` 后调函数。
# 之前的写法是 `source ...; $(build_workers)`，函数并不存在 → WORKERS 为空
# → CMAKE_BUILD_PARALLEL_LEVEL="" → cmake/ninja 视为**不限并发**，
# 「两级限并发」实际上完全没生效（2026-11-09 发现并修复）。
# 同样重要的是：执行失败时必须回退到 1，绝不能让空值漏进 -j。
WORKERS="${AMR_COLCON_WORKERS:-}"
if [ -z "$WORKERS" ]; then
  WORKERS="$(bash "$DIR/lib/build_workers.sh" 2>/dev/null)" || WORKERS=""
fi
case "$WORKERS" in
  '' | *[!0-9]*) WORKERS=1 ;;
esac

# colcon 的 --parallel-workers 只管**包级**并发，包内部的 make/ninja 仍按
# CPU 数开 -j。不设这个变量的话，`--parallel-workers 1` 在 4 核机器上依然
# 会同时跑 4 个 cc1plus——「限了并发」是错觉。必须两级一起限。
export CMAKE_BUILD_PARALLEL_LEVEL="$WORKERS"

# 第二道保险：限制单个 cc1plus 的 GCC 内部 GC 堆峰值。
#
# 并发压到 1 仍然不够时（超大 TU 单独就能吃掉 2 GB），AMR_GCC_MEM_GUARD=1
# 会让编译多回收几次，用编译时间换内存峰值。默认关闭：对内存充裕的机器
# 它是纯拖慢，没有收益。完整说明见 scripts/gcc_mem_guard.sh。
#
# 通过 CMAKE_CXX_COMPILER_LAUNCHER 注入，对所有子项目透明——包括
# amr_dispatcher_core 这种独立 CMake 包，不需要它配合改任何 CMakeLists。
# 注意只在启用时才追加 -D：传空串会写进 CMake 缓存并触发一次重新配置，
# 而「关闭」语义应当是「什么都不做」。
LAUNCHER_ARGS=()
if [ "${AMR_GCC_MEM_GUARD:-0}" = "1" ]; then
  LAUNCHER_ARGS=(-DCMAKE_CXX_COMPILER_LAUNCHER="$DIR/gcc_mem_guard.sh")
fi

echo "=== Building amr_dispatcher_core (Standalone CMake, -j$WORKERS) ==="
cmake -S "$ROOT_DIR/src/amr_dispatcher_core" -B "$ROOT_DIR/build/core" \
      -DBUILD_TESTING=ON -DCMAKE_CXX_FLAGS="-std=c++20" -DCMAKE_BUILD_TYPE=Release \
      "${LAUNCHER_ARGS[@]}"
cmake --build "$ROOT_DIR/build/core" -j"$WORKERS"

echo "=== Running Core Unit Tests ==="
ctest --test-dir "$ROOT_DIR/build/core" --output-on-failure

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  echo "=== Sourcing ROS 2 Jazzy and building with colcon ==="
  # 必须在此**临时**关闭 -u：ROS 的 setup.bash 会引用未定义变量
  # （实测报 `AMENT_TRACE_SETUP_FILES: unbound variable`），在 `set -u`
  # 下 source 直接失败。这个坑只在「宿主机装了 ROS 且真的走到这一支」时出现，
  # 而 CI 的 core job 与无 ROS 的开发机都会跳过这一支，所以长期没被发现。
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
  cd "$ROOT_DIR"
  # --base-paths src 不可省：仓库根目录有一个 CMakeLists.txt，colcon 会把它
  # 当成包 amr_dispatcher_workspace 并**忽略 src/ 下所有真实包**。
  # 实测：不带该参数时 `colcon list` 只输出 amr_dispatcher_workspace，
  # 构建"成功"但 install/ 里一个 amr_dispatcher_* 都没有——这是静默失败，
  # 比报错更危险。用法与 docker/entrypoint.server.sh 保持一致。
  #
  # --parallel-workers 必须显式传：colcon 默认就用 nproc，不传等于没修。
  #
  # --cmake-args 只在启用内存保险时才追加：colcon 会把 --cmake-args 之后的
  # 全部单词透传给所有包的 cmake 配置阶段，多传一个空的 -D 会让每个包都
  # 多一次无效配置。
  if [ "${#LAUNCHER_ARGS[@]}" -gt 0 ]; then
    colcon build --base-paths src --symlink-install --parallel-workers "$WORKERS" \
      --cmake-args "${LAUNCHER_ARGS[@]}"
  else
    colcon build --base-paths src --symlink-install --parallel-workers "$WORKERS"
  fi
else
  echo "[INFO] ROS 2 Jazzy not found in host environment. Colcon build skipped (run inside docker if needed)."
fi

echo "=== Build Complete ==="
