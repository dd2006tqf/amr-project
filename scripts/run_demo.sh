#!/usr/bin/env bash
set -euo pipefail

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
ROOT_DIR="$DIR/.."

echo "=== AMR Dispatcher Demo Runner ==="
echo "1. Checking Core build..."
if [ ! -f "$ROOT_DIR/build/core/libamr_dispatcher_core.a" ]; then
  bash "$ROOT_DIR/scripts/build.sh"
fi

echo "2. Running Core Unit & Integration Tests (ctest)..."
ctest --test-dir "$ROOT_DIR/build/core" --output-on-failure

echo "3. Running Demo Finished Successfully!"
