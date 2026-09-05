#!/usr/bin/env bash
set -e

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
  source /opt/ros/jazzy/setup.bash
  cd "$ROOT_DIR"
  colcon build --symlink-install
else
  echo "[INFO] ROS 2 Jazzy not found in host environment. Colcon build skipped (run inside docker if needed)."
fi

echo "=== Build Complete ==="
