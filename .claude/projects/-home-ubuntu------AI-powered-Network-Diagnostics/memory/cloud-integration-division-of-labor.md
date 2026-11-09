---
name: cloud-integration-division-of-labor
description: 云端接入由并行会话实现（Phase 4b 无线因果诊断），本会话只做规划+评审；本机 Python 工具链实为可用（见正文，2026-10-09 修正）
metadata: 
  node_type: memory
  type: project
  originSessionId: 71256ec0-8b40-4486-a6c7-50fff1561007
  modified: 2026-10-09T14:24:04.116Z
---

2026-10-01 起，WeakNet ↔ Large-Model-Application 全量接入的分工：

- **并行会话（另一 writer）负责实现**：截至当日 11:40+ 正在写 `network_assurance/wireless_contracts.py`、`wireless_rules.py`、`wireless_diagnosis.py`、`upstream.py`、alembic `0089_wireless_incident_diagnosis.py` 及配套 tests（Phase 4b 无线事件因果诊断 + 上行）。
- **本会话角色 = 规划 + 评审**：不碰 `Large-Model-Application/src/industrial_ops_agent/network_assurance/` 目录，不做实现提交。
- 铁律提醒：一个工作树一个 writer；`change_new.sh` 要求工作区干净——并行期间大量未提交文件会让 OpenSpec change 创建受阻，走工作流前须先提交。

环境事实（2026-10-09 实测修正，旧结论"本机无 pip/venv/uv"作废）：
- `python3 -m pip` 24.0 **存在**（PEP 668 externally-managed，装包须 `python3 -m venv /tmp/...` 后在 venv 内用 pip，或 `--break-system-packages`）；
- **PyPI 直连极慢/不可达**（pypi.org、github.com releases、astral.sh 均超时），装包必须走腾讯镜像 `-i https://mirrors.cloud.tencent.com/pypi/simple`；uv 可由此装进临时 venv，`uv lock --check` 可离线校验 lock；
- LMA 的 `.venv` **真实存在**（pip 安装、含 pytest 9.1.1 / ruff / openapi 生成所需全部依赖），且正是 `weaknet-cloud-api.service` 的运行时 → `Large-Model-Application/tests/` 的 pytest、`scripts/generate_openapi*.sh --check` 均可本机直接跑（此前"只能靠 CI"的说法过时）；
- `corepack enable` 需 sudo（写 /usr/local/bin），pnpm 11.20.0 已激活，web 的 typecheck/build 本机可跑。

**Why:** 避免与并行会话撞写、避免把“本机没跑测试”误报为“测试通过”。
**How to apply:** 报告 Python 测试状态时必须注明运行来源（CI vs 本机）；评审网络保障切片时只读、不改文件。相关 [[lma-attribution-mit]]。
