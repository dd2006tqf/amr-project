---
name: cloud-tests-starlette-pin
description: 云端 CI 读 requirements-test.txt 而非 uv.lock，starlette 必须显式锁版本
metadata: 
  node_type: memory
  type: project
  originSessionId: 17d1d78c-4f7c-43a3-aa25-5b20af6a6984
  modified: 2026-11-02T05:42:34.172Z
---

`Large-Model-Application` 的云端测试在 CI 里有两条独立的依赖来源：本地 `uv.lock`，CI 则是每次全新 `pip install requirements-test.txt`。后者对 `starlette` 原本无约束，而 fastapi 只声明 `starlette>=X` 不设上限，于是 pip 装到 1.10.0。

starlette 1.10.0 的 `_exception_handler` 里 `exc.__traceback__ = None` 对 AppError 抛 `TypeError: super(type, obj): obj must be an instance or subtype of type`，导致 `tests/test_e2e_http_smoke.py` 11 项全红（其余 288 项通过）。

2026-11-01 在 `requirements-test.txt` 显式锁 `starlette==1.4.0`（与 uv.lock 一致），CI 恢复 299 passed。

**判别要点**：`Large-Model-Application/` 零改动却只有 cloud-tests job 红 → 先怀疑依赖漂移，不要先怀疑代码回归。用同 venv 只换 starlette 版本做对照实验即可确认。

**另一条 CI 计数事实**：`ctest --test-dir build-x86/server` 是 54 套件（README 写的就是这个数），但 CI 的 build-and-test job 跑的是 `build-x86`，还包含 `client/` 注册的 `test_lifecycle_api`，所以 CI 报 55/55。两个数都对，别互相纠正。
