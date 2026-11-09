---
name: board-tunnel-and-network-baseline
description: 板端三条易踩的基线：唯一隧道单元 weaknet-tunnel.service（旧的 cloud-tunnel 必须禁用）、热点下 DNS 要手动指定公网、NTP 要用 IP 直连
metadata: 
  node_type: memory
  type: project
  originSessionId: 997f863b-48d7-47f3-89c2-2a32b2f0fb98
  modified: 2026-10-21T06:43:41.126Z
---

2026-10-21 在板端踩实并修好的三条基线（`ssh board` 连不上 / 域名解析不了 / 时间漂移都归到这三条）：

1. **隧道只能有一个单元**：板端 `weaknet-tunnel.service`（`-R 2222:localhost:22 -L 8000 -L 8090`，ServerAlive 10/3，RestartSec=3）是唯一保留的隧道单元。旧的 `cloud-tunnel.service`（Phase 3 遗留）两者都占板端 `-L 8000`，后启动者必报 `bind [127.0.0.1]:8000: Address already in use` 并反复重启，云端 2222 反向端口随之消失——表现就是 `ssh board` 一律 `Connection refused`/banner 超时。处置：`systemctl disable --now cloud-tunnel.service`。
2. **热点 DHCP 下板端 DNS 失效**：网关 192.168.137.1 不做 DNS 解析，板子 `getent hosts` 全超时，但裸 IP 出网正常（`curl --resolve` 可通）。修法：`nmcli con mod tan ipv4.dns "223.5.5.5 119.29.29.29" ipv4.ignore-auto-dns yes`（连接名 `tan`）。
3. **NTP 主机名同样解析不了 → 时钟漂移一天多**（tar 报"时间戳在未来"）。修法：`/etc/systemd/timesyncd.conf.d/ntp.conf` 写 `NTP=203.107.6.88 120.25.115.20 202.108.22.5`（IP 直连），重启 timesyncd，`NTPSynchronized=yes`。

另：云端侧 127.0.0.1:2222 若被僵死 sshd 会话占住（`ss -tlnp` 显示 sshd 而非 ssh），`ssh board` 会卡在 banner 交换；杀掉该 sshd 会话或重启 sshd 即可。关联 [[board-network-access]] [[cloud-ota-receipt-lifecycle]]。
