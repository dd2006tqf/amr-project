---
name: deploy-credential-ordering
description: 凭据注入部署必须先于服务重启；验证窗口只能算部署完成之后的流量
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7b80da08-bb70-42a2-9551-950a346c64f9
  modified: 2026-10-09T15:38:10.794Z
---

2026-10-09 edge token 迁移踩的两个连环坑（都在 tools/ci.sh 部署路径）：

1. **注入顺序 = 正确性**：把 token 注入放在 `systemctl restart` 之后 → 服务启动读到空占位 → 上行被静默关闭（日志只有 `edge.token 未配置`，无外显故障）。凭据注入必须发生在进程读取配置**之前**（先注入 /tmp 副本，再 cp 到 /etc，再重启）。
2. **验证窗口陷阱**：查"最近 N 分钟 200"时把部署**前**的正常流量算进了窗口，误报验证通过（当时 17×200 全是部署前的）。验证凭据/配置类部署，窗口起点必须 ≥ 部署完成时刻，并交叉检查服务端启动后有无对应报错。

**Why:** 这两个错误组合会造成"以为部署成功、实际链路已断"的假绿灯，且断链静默（板端只在 journal 里留一行 error）。
**How to apply:** 改部署脚本时审计每个"写配置 → 起服务"对的先后；验证时先记部署完成时间戳，只统计其后的窗口，并 grep 服务启动日志确认无配置类 error。相关 [[cloud-integration-division-of-labor]]。
