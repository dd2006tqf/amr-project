---
name: project-profile-cmake-migration
description: .ai-harness/project-profile.json 已从 make 时代迁移到 CMake；profile_sha256 变更为 802da63a...，归档证据无需迁移
metadata: 
  node_type: memory
  type: project
  originSessionId: c18c9c46-652e-4c79-8de2-41f56df36b24
  modified: 2026-09-24T16:27:20.779Z
---

2026-09-24，`.ai-harness/project-profile.json` 从 Makefile 时代化石修正为 CMake 事实，`profile_sha256` 由 `37cfdb19...` 变为 `802da63a98f95bb03a32ffac6a2bfca90d3dede37342a4284757c17665cbad77`。

变更内容：
- `adapter`: `make` → `cmake`；新增 `configure` capability（`configure-x86` 命令）
- `build-server`: `make -C server` → `cmake --build build-x86 --parallel`（timeout 600）
- `test-all`: `bash server/test/run_all_tests.sh --skip-build` → `ctest --test-dir build-x86/server --output-on-failure`（timeout 120）
- `path_roles.generated`: `server/build`/`server/test/bin`/`server/reports` → `build-x86`/`build-arm64`/`dist-arm64`
- `path_roles.build_metadata`: `server/Makefile`/`Makefile` → 三个 CMakeLists.txt

**Why:** 旧 profile 里每条命令与每条路径都不存在，`--project-command` 证据链完全不可用（`make -C server` exit 2、`run_all_tests.sh` exit 127）。同时 `source_fingerprint.sh` 的 generated 排除集失效。

**How to apply:** 28 个已归档证据文件仍绑定旧 hash `37cfdb19...`，这是**设计如此、无需迁移**——`validateProjectCommandEvidence` 只对当前 active change 的 verification 调用（`evaluator_check.sh:298`），归档 receipt 只被 `campaign.js`/`event_audit.sh` 做哈希摘要、不重算 profile。实测 `event_audit.sh <archived-change> --check` 仍通过。归档证据永不被 re-validate，因此改 profile 不会碎历史；但这是一次真实基线变更，需用户批准后才做。

相关：[[planning-fingerprint-immutability]]、[[workflow-harness-snapshot-bug]]
