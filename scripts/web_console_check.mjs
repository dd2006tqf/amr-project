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
  if (res.result?.exceptionDetails) {
    const d = res.result.exceptionDetails;
    throw new Error(d.exception?.description || d.text || 'eval error');
  }
  return res.result?.result?.value;
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

await send('Runtime.enable');
await send('Page.enable');
await send('Page.navigate', { url: BASE + '/' });
await sleep(2500);

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

  default:
    check('未知 phase', false, PHASE);
}

check('页面无未捕获 JS 异常', consoleErrors.length === 0, consoleErrors.join(' | '));

const failed = checks.filter((c) => !c.ok);
console.log(JSON.stringify({ phase: PHASE, base: BASE, checks, failed: failed.length }, null, 2));

ws.close();
process.exit(failed.length === 0 ? 0 : 1);
