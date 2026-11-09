---
name: lma-nested-workflow-dormant
description: LMA 嵌套 .github/workflows 未被 GitHub 注册；2026-10-09 已把 OpenAPI/客户端/前端三道门禁迁入根 ci.yml contract-and-web 作业
metadata: 
  node_type: memory
  type: project
  originSessionId: 5ceebcb5-2ecb-4a68-b482-f59365603b14
  modified: 2026-10-09T15:15:32.185Z
---

2026-10-08 排查 `dev/feature` CI 连续 failure 时发现的结构性事实：

**GitHub Actions 只扫描仓库根 `.github/workflows/`。** 实测
`GET /repos/dd2006tqf/internet-bluetooth-diagnose/actions/workflows` 只返回两个：
`CI`（根 `.github/workflows/ci.yml`）与 `ARM64 Build & Deploy`（手动触发）。

`Large-Model-Application/.github/workflows/ci.yml`（name `M1 CI`，含
`generate_openapi.sh --check`、`generate_openapi_client.sh --check`、`secret-scan`）
**完全不在注册列表里**——即使写了 `on: pull_request` 也不会运行。目录嵌套即失效。

**实际会红的门禁**（根 CI，三个 job）：
- 静态规范检查：CRLF、受限二进制、`tools/verify_telemetry_contract.py --strict`
- 云端 Python 测试与连接路径静态检查：`pytest -m "not live"` + **`ruff 连接路径门禁`**
- x86 编译与全量单元测试：`cmake -B build-x86 -DBUILD_EBPF=OFF` + `ctest`

ruff 门禁覆盖 `network_assurance` / `edge` / `persistence` / `auth` / `knowledge`
/ `api/routes/network_assurance.py` + 一批测试文件；`tests/` 一并纳入
（"测试是契约的一部分，新测试文件不得带 lint 债进来"）。

**教训（本人踩过）**：
- 修 OpenAPI 漂移 ≠ 修 CI。那两道 `--check` 是休眠的；真正让 CI 红的是 ruff。
  在提交信息里把因果归错过一次，事后纠正。
- 本地要主动跑契约检查：`bash Large-Model-Application/scripts/generate_openapi.sh --check`
  与 `generate_openapi_client.sh --check`（`generate_openapi*` 是 **shell** 脚本，
  别用 `.venv/bin/python` 调）。
- 本机系统 `python3` 缺 pydantic；跑 `tools/verify_telemetry_contract.py` 要用
  `Large-Model-Application/.venv/bin/python`。

相关：[[cloud-integration-division-of-labor]]、[[workflow-harness-snapshot-bug]]

**2026-10-09 更新（门禁已迁移，CI 全绿）**：根 `ci.yml` 新增 `contract-and-web` 作业，
迁入 `generate_openapi.sh --check`、`generate_openapi_client.sh --check`、
`pnpm typecheck && pnpm build`（node 22.23.1、pnpm 11.20.0/corepack）。
**Python 环境必须用 `requirements-test.txt` 建 .venv（与 cloud-tests 同源），
不要用 uv sync**——首跑 57fd8e8 之前曾用 `uv sync --frozen --extra dev`，
因 dev extra 缺 pillow（只在 data-pipeline/local-ai/document/training extra 里）
在 generate_openapi 步骤 ModuleNotFoundError: PIL 红掉；且 uv.lock 与生产 .venv
有 60+ 版本漂移（starlette 1.4.0 vs 1.7.0），契约快照录制于 pip 环境。
嵌套 `M1 CI` 文件加了 DORMANT 头注仅作归档，gitleaks/mypy/terraform/vault/
compose-lite 等完整环境专属步骤**未迁移**。最终 `57fd8e8` 四个 job 全部 success。
