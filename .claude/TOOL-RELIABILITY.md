# 工具调用可靠性守则

> 来源：2026-11-02 一次真实事故。我用 `pkill -f 'laneai-usage-proxy.py'` 重启本地代理，
> 结果把承载本会话 API 流量的进程一起杀了，用户连续几轮消息全都 Connection refused，
> 完全无法与我对话。本文是那次事故 + 同期多次 Write 截断 + 退出码误读的复盘产物。
>
> 适用范围：本机所有 Claude Code 会话。

---

## 规则 1 · 不杀自己

**触发场景**：重启/停止任何后台服务、代理、守护进程。

### 绝对禁止

```bash
pkill -f 'xxx'        # 模式会匹配到执行 pkill 的 shell 自身命令行
killall python3       # 误伤面完全不可控
pgrep -f 'xxx' | xargs kill
```

实证（2026-11-02 现场抓取）：

```
3282034  python3 .../laneai-usage-proxy.py                          ← 目标
3311136  bash -c ... pgrep -af 'laneai-usage-proxy.py' ...          ← 我自己
```

两行都命中。**注意：即使用中括号技巧 `[l]aneai-...`，那个 shell 依然被命中**，
因为模式串还出现在命令行的另一处（`pgrep -af '[l]aneai-...'` 这个字面量本身）。
中括号技巧只在"模式串仅出现一次"时有效，这里不成立。

### 正确做法

```bash
# 首选：服务单元原子重启（不停服务窗口、不涉及模式匹配）
systemctl --user restart <service>
systemctl restart <service>

# 备选：按 PID 精确操作，且必须核对，避免 PID 复用误伤
PID=$(systemctl --user show -p MainPID --value <service>)
if [ "$PID" != "0" ] && ps -o cmd= -p "$PID" | grep -q '<期望的脚本名>'; then
    kill -TERM "$PID"
fi

# 再备选：pidfile
[ -f /run/xxx.pid ] && kill -TERM "$(cat /run/xxx.pid)"
```

### 前置自问（动任何后台组件之前）

> **这东西挂掉之后，我还能不能跟用户说话？**

能 → 可以 stop-then-start。
不能 → 必须原子切换（restart / reload / 双实例热切），且把验证命令写在同一条命令的后半段。

---

## 规则 2 · 写完必自检

**触发场景**：用 Write 工具或 heredoc 写超过约 100 行的文件。

### 背景

Write 工具单次输出有长度上限，**超长会被静默截断**，不报错，末尾可能出现字面量 `…`。
2026-11-02 写 258 行的 Python 代理时连续被截断 3 次；写 PowerShell 安装脚本时
尾部停在 `# UTF-8 无 BOM 写入，避…`。

### 规定

长文件**一律走 heredoc**，落盘后立刻四项自检，四项全过才算写完：

```bash
cat > /path/file.ext <<'EOF'
...完整内容...
EOF

# 四项自检
python3 -m py_compile /path/file.py    # 1. 语法（按语言替换：node --check / pwsh -c 等）
wc -l /path/file.py                     # 2. 行数符合预期
tail -3 /path/file.py                   # 3. 尾部是完整语句，不是半句
grep -c '…' /path/file.py               # 4. 截断特征 = 0
```

heredoc 的额外好处：`<<'EOF'` 单引号形式不做变量展开，内容原样落盘。

### 检查脚本完整性时，别只看工具回显

工具回显本身也会截断。判断文件是否完整，只看**磁盘上的真实内容**
（`wc -l` / `tail` / `grep`），不看工具输出的末尾。

---

## 规则 3 · 退出码只当线索

**触发场景**：任何返回非零退出码的命令。

### 背景

2026-11-02 的 `pkill` 返回 144。144 = 128 + 16（SIGSTKFLT），是 shell 包装层被信号带走的
副作用，**不携带"哪个环节出错"的信息**。同一时期 `nohup ... &` 也返回 144，
一度被误判成"启动失败"，实际是 shell 被带走的假象。

### 规定

退出码不构成结论，必须去查真实状态：

```bash
ss -ltn | grep <port>                  # 端口在听吗
systemctl --user is-active <service>    # 服务活着吗
curl -s http://127.0.0.1:<port>/healthz # 应用层通吗
ps -o pid,etime,cmd -p <pid>           # 进程还在、活了多久
```

**推论**：给关键后台组件加 `/healthz` 之类的应用层探针，让"到底活没活"可查询，
而不是靠退出码猜。laneai-usage-proxy 就为此加了 `/healthz`。

---

## 规则 4 · 先定判据再验证

**触发场景**：任何验证/检查类命令。

### 背景

同一次事故中，一条乱码自检的 grep 已经输出结果为空（= 通过），我却又重跑了一遍，
用户发「？」。这是**该收敛时没收敛**。

### 规定

每条验证命令发出**之前**先明确：

- 什么输出算通过
- 什么输出算失败
- 失败时下一步做什么

通过就往前走，不过才重试。不允许"先跑跑看"，然后盯着输出临时决定要不要再跑。

**同一条检查不要重复跑第二遍**，除非上一次的结论确实不成立。

---

## 规则 5 · 工具能力边界

**触发场景**：正则/管道/复杂 shell 表达式。

### 已知限制

| 工具 | 限制 | 绕法 |
|---|---|---|
| ugrep（本机 `grep`） | `.{0,120}xxx.{0,120}` 这类大重复量会报 `exceeds complexity limits` | 用 Python `re` 后处理 |
| Write 工具 | 长内容静默截断 | 改用 heredoc + 四项自检 |
| heredoc 内嵌多层引号 | 容易写坏 | 先把探针写成 `.py` 文件再跑，别在命令行内联多行 |

复杂文本处理一律降级到 Python 脚本文件，不在 shell 里硬拼。

---

## 事故速查表

| 现象 | 大概率根因 | 第一动作 |
|---|---|---|
| exit 144 / 143 / 137 | 被信号带走（自我杀伤或包装层连带） | 查目标服务实际状态，不要重试原命令 |
| 文件末尾有 `…` 或停在半句 | Write 输出截断 | 改用 heredoc 重写并四项自检 |
| 用户几轮没回音 / 我发出去没回应 | 通信链路上的组件被我杀掉 | 立刻查 `systemctl status` + 端口 |
| `grep` 报复杂度超限 | ugrep 重复量保护 | 改用 Python |
| 命令"看起来失败"但实际生效 | 退出码误读 | 去查状态，不看退出码 |

---

## 一句话总结

**不杀自己·写完必检·退出码只当线索·先定判据·复杂处理降级到脚本。**
