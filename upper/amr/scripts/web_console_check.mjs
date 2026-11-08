#!/usr/bin/env node
// ==============================================================================
// web_console_check.mjs - Web 控制台（tools/operator_console.html）浏览器端验收
//
// 用 CDP 驱动 headless Chrome 真实点击页面按钮，断言：
//   * 受管车辆面板渲染（表格 + 后端 + 在线态）
//   * 强清缺审计字段 -> 前端拦下
//   * 强清在线车 -> **服务端**复验拒绝（证明"前端隐藏按钮不是安全边界"）
//   * 强清离线车 -> 受理 + 面板显示隔离态
//   * 恢复未勾选"已核实物理位置" -> 前端拦下
//   * 恢复（已勾选）-> 解除隔离，面板恢复在线
//
// 用法：node web_console_check.mjs <phase> <base_url>
//   phase: panel | force_clear_online | force_clear_offline | restore_guard | restore
// 退出码：0 = 该 phase 全部断言通过；非 0 = 有断言失败（stdout 输出 JSON 明细）
// ==============================================================================

const PHASE = process.argv[2];
const BASE = (process.argv[3] || '').replace(/\/$/, '');
const CDP_PORT = process.env.CDP_PORT || '9333';

if (!PHASE || !BASE) {
  console.error('usage: node web_console_check.mjs <phase> <base_url>');
  process.exit(2);
}

const checks = [];
function check(name, ok, detail = '') {
  checks.push({ name, ok: !!ok, detail: String(detail) });
}

// ---------------------------------------------------------------------------
// CDP 连接
// ---------------------------------------------------------------------------
async function fetchJson(url, options) {
  const res = await fetch(url, options);
  return res.json();
}

async function waitForCdp(deadlineMs = 15000) {
  const started = Date.now();
  while (Date.now() - started < deadlineMs) {
    try {
      return await fetchJson(`http://127.0.0.1:${CDP_PORT}/json/version`);
    } catch {
      await new Promise((r) => setTimeout(r, 300));
    }
  }
  throw new Error('CDP 端口未就绪: ' + CDP_PORT);
}

await waitForCdp();

const target = await fetchJson(
  `http://127.0.0.1:${CDP_PORT}/json/new?${encodeURIComponent(BASE + '/')}`,
  { method: 'PUT' }
);

const ws = new WebSocket(target.webSocketDebuggerUrl);
let msgId = 0;
const pending = new Map();
const consoleErrors = [];

ws.addEventListener('message', (ev) => {
  const msg = JSON.parse(ev.data);
  if (msg.id && pending.has(msg.id)) {
    pending.get(msg.id)(msg);
    pending.delete(msg.id);
  }
  if (msg.method === 'Runtime.exceptionThrown') {
    const d = msg.params.exceptionDetails;
    consoleErrors.push(d.exception?.description || d.text || 'unknown exception');
  }
});

await new Promise((resolve, reject) => {
  ws.addEventListener('open', resolve);
  ws.addEventListener('error', reject);
});

function send(method, params = {}) {
  const id = ++msgId;
  ws.send(JSON.stringify({ id, method, params }));
  return new Promise((resolve) => pending.set(id, resolve));
}

async function evalJs(expression) {
  const res = await send('Runtime.evaluate', {
    expression,
    awaitPromise: true,
    returnByValue: true,
  });
  // CDP 层面的错误（渲染进程还没建好执行上下文时就是这条：
  // "Cannot find context with specified id"）以前被静默吞成 undefined，
  // 上层 JSON.parse 得到一个空对象，于是轮询"看起来在正常跑"却永远看不到
  // 页面 —— 20s 后报"页面没加载出来"，而当时页面其实早就好了。
  // 这里必须把它抛出去，让等待循环当成"还没好，继续等"。
  if (res.error) {
    throw new Error('CDP ' + res.error.code + ': ' + (res.error.message || ''));
  }
  if (res.result?.exceptionDetails) {
    const d = res.result.exceptionDetails;
    throw new Error(d.exception?.description || d.text || 'eval error');
  }
  return res.result?.result?.value;
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

// 等到某个元素真的在 DOM 里。
//
// 为什么不能只 sleep：`Page.navigate` 返回的是"已发出导航"，页面在
// 慢机器 / 冷缓存 / 容器网关首次读盘时可能还没解析完。原来固定 sleep(2500)
// 后直接 `getElementById('baseUrl').value = ...` 会抛
// `TypeError: Cannot set properties of null`，而这个错误发生在**任何断言之前**，
// 现场看起来像"控制台坏了"，实际是测试自己在抢跑。
// 这里轮询到元素出现（默认 15s），超时则给出带页面标题/URL 的明确报错——
// 元素真的缺失时仍然失败，只是失败信息能定位问题。
async function waitForElement(id, timeoutMs = 15000) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    const found = await evalJs(`!!document.getElementById(${JSON.stringify(id)})`);
    if (found) return;
    await sleep(200);
  }
  const where = await evalJs(
    `JSON.stringify({url: location.href, title: document.title, ready: document.readyState})`);
  throw new Error(`页面在 ${timeoutMs}ms 内没有出现 #${id}（${where}）`);
}

await send('Runtime.enable');
await send('Page.enable');

// 等待页面正文真的加载出来。
//
// 背景：`/json/new?<url>` 已经在建 target 时带上了 URL，但**不保证**那次导航
// 已完成（更不保证成功）。此前的写法是 navigate 后固定 sleep(2500)，
// 在慢一点的机器上就会拿到空白文档，随后 `getElementById('baseUrl')` 抛
// `Cannot set properties of null` —— 一个发生在任何断言之前的失败，
// 现场看起来像"控制台坏了"。
//
// 这里改成：先轮询"正文长度 + 是否有预期的根元素"，超时则显式再
// `Page.navigate` 一次；仍然失败才报错，并把 URL/title/正文长度/HTTP 探测结果
// 一起打出来 —— 元素真的缺失时依然失败，但失败信息能定位问题。
//
// 预算 45s：Chrome 冷启动（新 profile、headless 首次建渲染进程）在负载高的
// 机器上确实会超过 10s。等待不弱化任何断言 —— 断言仍然要求页面真的加载出来，
// 只是给"浏览器自己还没准备好"留出余量。
async function pageState() {
  return evalJs(`JSON.stringify({
    url: location.href,
    title: document.title,
    ready: document.readyState,
    bodyLen: (document.body ? document.body.innerHTML.length : 0),
    hasBaseUrl: !!document.getElementById('baseUrl')
  })`);
}

async function waitForConsoleDocument(timeoutMs = 45000) {
  const deadline = Date.now() + timeoutMs;
  let reloaded = false;
  while (Date.now() < deadline) {
    // pageState() 抛异常不能终止轮询：导航途中 CDP 会对 target 短暂报
    // "Cannot find context with specified id"（页面正在换代），那是**中间态**，
    // 不是失败。之前这里没有 try/catch，一次这样的抖动就把整个 phase 判死，
    // 现场看起来是"控制台没加载出来"，而实际页面早就好了。
    let st = {};
    try {
      st = JSON.parse((await pageState()) || '{}');
    } catch (err) {
      st = { error: String(err && err.message ? err.message : err) };
    }
    if (st.hasBaseUrl) return st;
    // 给了 5s 还是空文档 -> 主动再导航一次（幂等：页面本身无副作用）
    if (!reloaded && Date.now() > deadline - timeoutMs + 5000) {
      try {
        await send('Page.navigate', { url: BASE + '/' });
      } catch (err) {
        // 导航本身失败也继续轮询：下一轮 pageState() 会给出真实状态
      }
      reloaded = true;
    }
    await sleep(250);
  }
  throw new Error(`页面未加载出控制台：${await pageState()}`);
}

const pageSt = await waitForConsoleDocument();
await waitForElement('vehicleLinksTable', 10000);
await sleep(500);

// headless 下 alert()/confirm() 会阻塞页面：打桩记录内容（不吞掉断言价值）
// 同时包一层 fetch：记录真实发出的 REST 请求，用来断言"前端拦下时根本没发请求"
await evalJs(`
  window.__alerts = [];
  window.__confirmText = null;
  window.__fetches = [];
  const __origFetch = window.fetch.bind(window);
  window.fetch = (input, init) => {
    const url = typeof input === 'string' ? input : (input && input.url) || String(input);
    window.__fetches.push(url);
    return __origFetch(input, init);
  };
  window.alert = (m) => { window.__alerts.push(String(m)); };
  window.confirm = (m) => { window.__confirmText = String(m); return true; };
  true;
`);

// 控制台的 REST 地址来自输入框：统一改到本轮网关地址
await evalJs(`document.getElementById('baseUrl').value = ${JSON.stringify(BASE)}; true;`);
// 关掉 1s 自动轮询：轮询会周期性覆写状态行，导致"前端拦下"的提示被冲掉
await evalJs(`document.getElementById('autoPoll').checked = false; true;`);
await evalJs(`document.getElementById('refreshBtn').click(); true;`);
await sleep(1500);

const statusText = () => evalJs(`document.getElementById('statusLine').innerText`);
const panelText = () => evalJs(`document.getElementById('vehicleLinksTable').innerText`);
const alerts = () => evalJs(`window.__alerts`);
const fetches = () => evalJs(`window.__fetches`);

async function waitStatus(predicate, timeoutMs = 8000) {
  const started = Date.now();
  let last = '';
  while (Date.now() - started < timeoutMs) {
    last = await statusText();
    if (predicate(last)) return last;
    await sleep(200);
  }
  return last;
}

async function fillAudit(vehicleId, operatorId, reason) {
  await evalJs(`document.getElementById('forceClearVehicleId').value = ${JSON.stringify(vehicleId)}; true;`);
  await evalJs(`document.getElementById('forceClearOperator').value = ${JSON.stringify(operatorId)}; true;`);
  await evalJs(`document.getElementById('forceClearReason').value = ${JSON.stringify(reason)}; true;`);
}

// 面板数据来自 /dispatcher/state（2Hz 发布）+ REST 侧缓存：
// 状态变更（如隔离标记）要等下一帧状态报文才反映到面板，
// 因此读面板前主动刷一次并留出传播时间。
async function refreshPanel() {
  await evalJs(`document.getElementById('refreshBtn').click(); true;`);
  await sleep(1200);
}

// ---------------------------------------------------------------------------
// 各 phase 的断言
// ---------------------------------------------------------------------------
switch (PHASE) {
  case 'panel': {
    const title = await evalJs('document.title');
    check('页面标题正确', /AMR Dispatcher/i.test(title), title);

    const hasTable = await evalJs(`!!document.querySelector('#vehicleLinksTable table')`);
    check('受管车辆面板渲染出表格', hasTable);

    const headers = await evalJs(
      `Array.from(document.querySelectorAll('#vehicleLinksTable table tr:first-child th')).map(t => t.innerText.trim())`
    );
    check(
      '表头含车辆/后端/状态/位置/电量/状态时延',
      ['车辆', '后端', '状态', '位置', '电量', '状态时延'].every((h) => headers.includes(h)),
      JSON.stringify(headers)
    );

    const panel = await panelText();
    check('面板列出受管车辆行', /\S/.test(panel) && panel.includes('vda5050'), panel);
    check('面板显示在线态', panel.includes('在线'), panel);
    break;
  }

  case 'force_clear_online': {
    // 在线车强清：必须被**服务端**拒绝（前端隐藏按钮不是安全边界）
    await fillAudit('wc_amr_01', 'operator_ui', 'UI 验收：在线车强清应被拒');
    await evalJs(`document.getElementById('forceClearBtn').click(); true;`);
    await sleep(2000);
    const messages = (await alerts()).join(' | ');
    check(
      '在线车强清被服务端拒绝',
      /拒绝|ONLINE|online/i.test(messages),
      messages
    );
    const panel = await panelText();
    check('拒绝后车辆仍在线', panel.includes('在线'), panel);
    check('拒绝后车辆未被误隔离', !panel.includes('隔离'), panel);
    break;
  }

  case 'force_clear_offline': {
    // 缺审计字段 -> 前端拦下（不发请求）
    await evalJs(`document.getElementById('forceClearVehicleId').value = 'wc_amr_01'; true;`);
    await evalJs(`document.getElementById('forceClearOperator').value = ''; true;`);
    await evalJs(`document.getElementById('forceClearReason').value = ''; true;`);
    await evalJs(`window.__fetches.length = 0; true;`);
    await evalJs(`document.getElementById('forceClearBtn').click(); true;`);
    await sleep(600);
    const blocked = await statusText();
    check('缺审计字段被前端拦下', /需要填写|操作员|原因/.test(blocked), blocked);
    check(
      '缺审计字段时未发出强清请求',
      !(await fetches()).some((u) => u.includes('force_clear_link')),
      JSON.stringify(await fetches())
    );

    // 填齐审计字段 -> 弹确认框 -> 提交 -> 受理并转入隔离
    await fillAudit('wc_amr_01', 'operator_ui', 'UI 验收：车辆报废下线');
    await evalJs(`window.__confirmText = null; true;`);
    await evalJs(`document.getElementById('forceClearBtn').click(); true;`);
    await sleep(2500);
    const prompt = await evalJs(`window.__confirmText`);
    check('强清前弹出确认框并回显车辆 id', /wc_amr_01/.test(prompt || ''), prompt || '(no prompt)');
    check(
      '确认框写明"通信失联不代表车辆已停止"',
      /通信失联|物理位置/.test(prompt || ''),
      prompt || '(no prompt)'
    );

    const messages = (await alerts()).join(' | ');
    check('离线车强清被受理', /强制清除完成|QUARANTINED/i.test(messages), messages);

    // 隔离标记经由 /dispatcher/state 传播到面板：刷新后读
    await refreshPanel();
    const panel = await panelText();
    check('面板显示隔离态', panel.includes('隔离'), panel);
    check('面板显示离线态', panel.includes('离线'), panel);
    break;
  }

  case 'restore_guard': {
    // 未勾选"已核实车辆物理位置" -> 前端必须拦下恢复
    await fillAudit('wc_amr_01', 'operator_ui', 'UI 验收：恢复前置校验');
    await evalJs(`document.getElementById('positionVerified').checked = false; true;`);
    await evalJs(`window.__fetches.length = 0; true;`);
    await evalJs(`document.getElementById('restoreLinkBtn').click(); true;`);
    await sleep(600);
    const blocked = await statusText();
    check('未核实位置时恢复被前端拦下', /核实|物理位置/.test(blocked), blocked);
    check(
      '未核实位置时未发出恢复请求',
      !(await fetches()).some((u) => u.includes('restore_link')),
      JSON.stringify(await fetches())
    );
    break;
  }

  case 'restore': {
    await fillAudit('wc_amr_01', 'operator_ui', 'UI 验收：车辆已恢复在线，位置已现场核实');
    await evalJs(`document.getElementById('positionVerified').checked = true; true;`);
    await evalJs(`window.__confirmText = null; true;`);
    await evalJs(`document.getElementById('restoreLinkBtn').click(); true;`);
    await sleep(2500);
    const prompt = await evalJs(`window.__confirmText`);
    check('恢复前弹出确认框', /恢复/.test(prompt || ''), prompt || '(no prompt)');

    // 恢复成功路径走状态行提示（失败路径才弹窗）
    const restored = await waitStatus((t) => /已恢复|恢复/.test(t), 8000);
    check('恢复被受理', /已恢复/.test(restored), restored);

    await refreshPanel();
    const panel = await panelText();
    check('隔离态已解除', !panel.includes('隔离'), panel);
    check('车辆恢复在线', panel.includes('在线'), panel);
    break;
  }

  // -------------------------------------------------------------------------
  // 相位 11：就绪红灯表与临时拓扑管制
  // -------------------------------------------------------------------------
  case 'topology_panel': {
    // 就绪表：不管 200 还是 503 都要能渲染出组件行（503 的 body 同样要解析）
    const rows = await evalJs(
      `Array.from(document.querySelectorAll('#readyzTable tr')).map(tr => tr.innerText)`
    );
    const flat = rows.join(' | ');
    check(
      '就绪表渲染 4 个组件',
      ['dispatcher', 'safety', 'chassis', 'coordinator'].every((c) => flat.includes(c)),
      flat
    );
    check('就绪表显示判据列', /ok|never_seen|stale|unhealthy|未监控/.test(flat), flat);

    const state = await evalJs(`document.getElementById('readyzState').innerText`);
    check('就绪总状态非空', !!state && state !== '-', state);
    // 未监控的组件必须诚实标注，不能伪装成 OK
    if (flat.includes('chassis') && /未监控/.test(flat)) {
      check('未监控组件被显式标注', true, 'chassis 标为未监控');
    }

    // 叠加层计数与 revision 必须在位（无叠加层时也应有 "0 封路 / 0 单向"）
    const counts = await evalJs(`document.getElementById('overlayCounts').innerText`);
    check('叠加层计数已渲染', /\d+\s*封路/.test(counts), counts);
    const rev = await evalJs(`document.getElementById('overlayRevision').innerText`);
    check('叠加层 revision 已渲染', /\d+/.test(rev), rev);
    const badge = await evalJs(`document.getElementById('overlayBadge').innerText`);
    check('叠加层徽标已渲染', /叠加层/.test(badge), badge);
    break;
  }

  case 'topology_preview_reject': {
    // 断连叠加层（封掉唯一桥）-> 预演必须被拒，且"应用"按钮保持 disabled
    const from = process.env.TOPO_BRIDGE_FROM || 'n1';
    const to = process.env.TOPO_BRIDGE_TO || 'leaf';
    const yaml = `blocked_edges:\n  - from: "${from}"\n    to: "${to}"\n`;
    // 预演同样要带审计原因：服务端校验的是"这条管制有没有理由"，
    // 与 dry_run 无关 —— dry_run 只决定写不写盘。
    await evalJs(
      `document.getElementById('topoOperator').value = 'operator_ui'; true;`
    );
    await evalJs(
      `document.getElementById('topoReason').value = 'UI 验收：断连预演应被拒'; true;`
    );
    await evalJs(`document.getElementById('topoYaml').value = ${JSON.stringify(yaml)}; true;`);
    // 输入事件触发"文本变了就重新上锁"
    await evalJs(
      `document.getElementById('topoYaml').dispatchEvent(new Event('input', {bubbles: true})); true;`
    );
    await evalJs(`document.getElementById('topoPreviewBtn').click(); true;`);
    await sleep(2500);

    const result = await evalJs(`document.getElementById('topoResult').innerText`);
    check(
      '预演被拒绝并给出 would_disconnect_topology',
      /would_disconnect_topology/.test(result),
      result
    );
    check('预演被拒时列出不可达节点', /unreachable_nodes/.test(result), result);
    const disabled = await evalJs(`document.getElementById('topoApplyBtn').disabled`);
    check('预演被拒后"应用"保持禁用', disabled === true, String(disabled));

    // 端口/文件侧红线由 shell 侧断言（"拒绝时不写盘"），这里只锁 UI 语义
    break;
  }

  case 'topology_block': {
    const from = process.env.TOPO_FROM || 'b';
    const to = process.env.TOPO_TO || 'c';
    const before = await evalJs(`document.getElementById('overlayCounts').innerText`);
    await evalJs(`document.getElementById('topoFrom').value = ${JSON.stringify(from)}; true;`);
    await evalJs(`document.getElementById('topoTo').value = ${JSON.stringify(to)}; true;`);
    await evalJs(`document.getElementById('topoAction').value = 'block'; true;`);
    await evalJs(
      `document.getElementById('topoOperator').value = 'operator_ui'; true;`
    );
    await evalJs(
      `document.getElementById('topoReason').value = 'UI 验收：临时封路'; true;`
    );
    await evalJs(`window.__confirmText = null; true;`);
    await evalJs(`document.getElementById('topoBlockBtn').click(); true;`);
    await sleep(3000);

    const prompt = await evalJs(`window.__confirmText`);
    check('封路前弹出二次确认框', /执行 block/.test(prompt || ''), prompt || '(no prompt)');

    const after = await evalJs(`document.getElementById('overlayCounts').innerText`);
    check('封路后叠加层计数变化', after !== before, `${before} -> ${after}`);
    const result = await evalJs(`document.getElementById('topoResult').innerText`);
    check('封路响应带 revision', /overlay_revision/.test(result), result);

    // 读回 YAML：textarea 应被"读取当前叠加层"填上刚封的边
    await evalJs(`document.getElementById('topoLoadBtn').click(); true;`);
    await sleep(1500);
    const yamlText = await evalJs(`document.getElementById('topoYaml').value`);
    check('读回的 YAML 含刚封的边', yamlText.includes(from) && yamlText.includes(to), yamlText);
    check('读回后"应用"仍保持禁用（必须先预演）',
          (await evalJs(`document.getElementById('topoApplyBtn').disabled`)) === true);
    break;
  }

  case 'readiness_degraded': {
    // dispatcher 被杀后：/readyz 变 503，但页面必须继续轮询并渲染 503 body
    // （把它渲染成"加载失败"就等于把可诊断的事故变成不可诊断）
    const stateBefore = await evalJs(`document.getElementById('readyzState').innerText`);
    check('就绪态初始已渲染', !!stateBefore && stateBefore !== '-', stateBefore);

    // 等 shell 侧 kill 生效（由外层脚本负责杀进程，这里只观察页面）
    let sawRed = false;
    let lastState = '';
    const started = Date.now();
    while (Date.now() - started < 20000) {
      lastState = await evalJs(`document.getElementById('readyzState').innerText`);
      if (/NOT READY|DEGRADED|INITIALIZING/.test(lastState)) {
        sawRed = true;
        break;
      }
      await sleep(500);
    }
    check('dispatcher 停止后就绪态转红', sawRed, lastState);

    const rows = await evalJs(
      `Array.from(document.querySelectorAll('#readyzTable tr')).map(tr => tr.innerText).join(' | ')`
    );
    check('503 的 body 仍被渲染成组件明细', /dispatcher/.test(rows), rows);
    check(
      'dispatcher 行给出具体原因（不是"加载失败"）',
      /never_seen|stale|unhealthy|no_state|异常/.test(rows),
      rows
    );
    break;
  }

  default:
    check('未知 phase', false, PHASE);
}

check('页面无未捕获 JS 异常', consoleErrors.length === 0, consoleErrors.join(' | '));

const failed = checks.filter((c) => !c.ok);
console.log(JSON.stringify({ phase: PHASE, base: BASE, checks, failed: failed.length }, null, 2));

ws.close();
process.exit(failed.length === 0 ? 0 : 1);
