---
name: board-network-access
description: 板子连法：本机是云端服务器，走 127.0.0.1:2222 反向隧道（ssh board），不要用板子 IP
metadata: 
  node_type: memory
  type: project
  originSessionId: d3a9e2e5-7ae5-4b9b-85f4-eefa0f94d189
  modified: 2026-10-09T17:39:27.460Z
---

**日常开发发生在云端 Ubuntu 服务器上（后端 IP 81.71.76.133，本地 `10.1.0.13/22`，hostname `VM-0-13-ubuntu`），不是接入开发板所在局域网的机器。** 因此：

- 板子的 `192.168.2.77` / `192.168.137.210` 从这台机器**直连不通**（不同网段、无路由）。
- `radxa-cubie-a7a.local`（mDNS）同样解析不到 —— 跨网段组播过不去。
- **正确连法是反向隧道别名 `ssh board`**：板端 `cloud-tunnel.service` 主动 `ssh -R 2222:localhost:22` 拨到云端，云端监听 `127.0.0.1:2222`；`~/.ssh/config` 里 `Host board` → `127.0.0.1:2222`，`/etc/hosts` 有 `127.0.0.1 board`（满足 `ci.sh` 的 ping 可达性检查），`~/.bashrc` 里 `export BOARD=radxa@board`。
- 隧道偶发不稳（`ssh` 首次可能报 `kex_exchange_identification: read: Connection reset by peer`），**重试即可**，不要因此去改连 IP。
- **上行与隧道同生共死（2026-10-10 实测确认）**：板端 `config.yaml` 的 `edge.url = http://127.0.0.1:8000`，走的正是这条隧道的板端 `ssh -L` 转发——隧道断则遥测/事件上行一起停（云端 access log 源地址恒为 127.0.0.1 就是这个原因）。**板端家庭网络/供电不稳**：2026-10-09 约19:17、2026-10-10 约00:48 两次掉线（无 RST，云端只剩半死 ESTAB 与 Recv-Q 堆积，2222 监听消失），云端无法主动恢复，只能等板侧重连或现场检查电源/路由。判断"板子掉线"而非"代码坏了"：`journalctl -u weaknet-cloud-api` 停止收到 edge 请求 + `ss -tlnp | grep 2222` 空 + `journalctl -u ssh` 无新 Accepted。

**Why:** 2026-09-29 我按记忆里的"两个候选 IP"去连，两个都不通，浪费了大量时间排查网络，最后才想起本机在云端、必须走隧道。本机为云端这一事实写在 `docs/云端开发环境与开发板部署.md`，不是隐藏配置。

**How to apply:**
- 连板子一律 `ssh board "<cmd>"`；部署 `BOARD=radxa@board ./tools/ci.sh`。
- 板端 sudo 免密（NOPASSWD），非交互 ssh 可直接跑 sudo。
- 板端**没有 sqlite3 CLI**，查库用 `sudo python3 -c "import sqlite3; ..."`。
- 板端 `bpftool` 是 Debian 11 老版（不支持 `-B/--base-btf`），解析模块 BTF 会报 `Invalid argument` —— 这是工具版本问题，不是 BTF 损坏；要提取签名/偏移就把 `/sys/kernel/btf/<mod>` scp 到云端，用云端 bpftool v7.4 + `/sys/kernel/btf/vmlinux` 做 base。
- `ci.sh` 内部用 `| head -80` 截断板端输出；要完整证据单独跑 `ssh board 'sudo /home/radxa/weaknet/weaknet-test-full.sh'`。
- 完整冒烟 5–10 分钟，放后台跑再轮询。

相关：[[board-dbus-service-issue]]、[[board-bluetooth-no-tracepoint]]
