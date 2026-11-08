#!/usr/bin/env python3
"""容器健康检查（仅标准库）。

为什么不用 curl
---------------
`ros:jazzy-ros-base` 不保证带 curl/wget。HEALTHCHECK 里装包失败会让容器永远
处于 "unhealthy"，比没有检查更糟——它会训练所有人忽略红灯。

判据
----
`GET /readyz`：200 -> 健康（退出 0），其余 -> 不健康（退出 1），任何异常 -> 1。
刻意**不**用 /healthz：那只证明 HTTP 线程活着，而我们要的是"整栈真的能干活"。
DDS 卡死时网关仍能回 /healthz，这正是必须看 /readyz 的原因。

重要边界（不要误解这个检查的能力）
---------------------------------
HEALTHCHECK **只提供可见性与 `docker compose up --wait` 的判定**，
它不触发自动重启。自愈靠 launch 里每个节点的 respawn。
严禁加 autoheal 之类的"看到 unhealthy 就重启容器"的守护：
安全门锁存 CRITICAL 时 /readyz 恒 503，重启会拆掉一个"健康但安全锁存"的栈，
并丢掉内存里的调度状态（队列、路权、时空窗），让一件可解释的事故变成一团乱账。
"""

import json
import os
import sys
import urllib.error
import urllib.request

TIMEOUT_SEC = 4.0


def main() -> int:
    port = os.environ.get("AMR_REST_PORT", "8080")
    url = f"http://127.0.0.1:{port}/readyz"
    try:
        with urllib.request.urlopen(url, timeout=TIMEOUT_SEC) as resp:  # noqa: S310 (loopback only)
            body = resp.read(8192).decode("utf-8", "replace")
            if resp.status != 200:
                print(f"not ready: HTTP {resp.status}: {body}")
                return 1
            # 复核 JSON（网关契约）；解析失败仍判健康——HTTP 200 已是权威判据，
            # 这里只为把结论打到 docker inspect 的日志里方便排障。
            try:
                verdict = json.loads(body)
                print(
                    "ready: state={state} components={n}".format(
                        state=verdict.get("state", "?"),
                        n=len(verdict.get("components", [])),
                    )
                )
            except ValueError:
                print(body)
            return 0
    except urllib.error.HTTPError as exc:
        detail = exc.read(8192).decode("utf-8", "replace") if exc.fp else ""
        print(f"not ready: HTTP {exc.code}: {detail}")
        return 1
    except Exception as exc:  # noqa: BLE001 (任何异常都必须判不健康)
        print(f"probe failed: {type(exc).__name__}: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
