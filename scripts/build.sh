#!/usr/bin/env bash
set -euo pipefail

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
ROOT_DIR="$DIR/.."

echo "=== Building amr_dispatcher_core (Standalone CMake) ==="
cmake -S "$ROOT_DIR/src/amr_dispatcher_core" -B "$ROOT_DIR/build/core" \
      -DBUILD_TESTING=ON -DCMAKE_CXX_FLAGS="-std=c++20" -DCMAKE_BUILD_TYPE=Release
cmake --build "$ROOT_DIR/build/core" -j$(nproc)

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
  colcon build --base-paths src --symlink-install
else
  echo "[INFO] ROS 2 Jazzy not found in host environment. Colcon build skipped (run inside docker if needed)."
fi

echo "=== Build Complete ==="
