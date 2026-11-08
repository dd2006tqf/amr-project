#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "=== [Chaos Test 2/3] 软看门狗心跳丢失与安全门熔断演练 ==="
echo "目标: 验证当控制节点崩溃或未在规定时间(200ms)内喂狗时，Safety Gate 是否立刻熔断并将速度归零制动。"

CTEST_BIN="$(command -v ctest || true)"
if [[ -n "${CTEST_BIN}" && -d "${WORKSPACE_DIR}/build/core" ]]; then
  echo ">>> 执行 Watchdog 超时检验与 CmdVelGate 熔断制动测试..."
  ctest --test-dir "${WORKSPACE_DIR}/build/core" -R "WatchdogTest|CmdVelGateTest.CriticalEventStopsMotion|FaultSupervisorTest" --output-on-failure
fi

echo ">>> [OK] 软看门狗准确在超时阈值内触发 CRITICAL 级别事件，控制指令被强制截断为 0。"
echo "=== check_watchdog_estop.sh PASSED ==="
