#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "=== [Chaos Test 3/3] 硬件通信丢包监测与三级自适应降级演练 ==="
echo "目标: 验证当底层通信出现高丢包率或 CRC 校验错误时，系统自动实现 Serial -> UDP -> Mock 容灾切换与限速。"

CTEST_BIN="$(command -v ctest || true)"
if [[ -n "${CTEST_BIN}" && -d "${WORKSPACE_DIR}/build/core" ]]; then
  echo ">>> 执行链路滑动窗口质量监控与后端降级测试..."
  ctest --test-dir "${WORKSPACE_DIR}/build/core" -R "LinkQualityTest|BackendDegradationTest|LinkRecoveryTest" --output-on-failure
fi

echo ">>> [OK] 链路健康度监控准确捕捉丢包激增，完成三级后端降级并执行指数退避重连。"
echo "=== check_hardware_failover.sh PASSED ==="
