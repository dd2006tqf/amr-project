#!/usr/bin/env python3
"""扫描代理原始 SSE dump，定位 DeepSeek 特殊 token 泄漏。"""
import json, re, sys, os

DUMP = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/.claude/laneai-raw-dump.log")

PATTERNS = {
    "DSML":       re.compile(r"</?\|[|\]]DSML[|]\|[|]>"),
    "ds_special": re.compile(r"<\|[A-Za-z_]+\|>"),
    "tool_call":  re.compile(r"</?tool_call\b"),
    "pipe_any":   re.compile(r"[<]/[|]{2}"),
}

def scan_text(txt):
    out = []
    for name, pat in PATTERNS.items():
        m = pat.search(txt)
        if m:
            out.append((name, m.start(), m.end()))
    return out

def main():
    if not os.path.exists(DUMP):
        print("dump 不存在:", DUMP)
        return 1
    size = os.path.getsize(DUMP)
    print("dump 大小: %.1f KB" % (size / 1024))
    if size == 0:
        print("暂无数据，复现后再跑")
        return 0

    raw = open(DUMP, "rb").read().decode("utf-8", "replace")
    streams = raw.split("===== NEW STREAM")
    print("流次数:", len(streams) - 1)

    hits = {k: 0 for k in PATTERNS}
    samples = []
    etypes = {}

    for s in streams[1:]:
        for line in s.splitlines():
            ls = line.strip()
            if not ls.startswith("data:"):
                continue
            p = ls[5:].strip()
            if not p or p == "[DONE]":
                continue
            try:
                ev = json.loads(p)
            except Exception:
                continue
            t = ev.get("type", "?")
            etypes[t] = etypes.get(t, 0) + 1

            blobs = []
            d = ev.get("delta") or {}
            if isinstance(d, dict):
                if d.get("text"):
                    blobs.append(("delta.text", d["text"]))
                if d.get("thinking"):
                    blobs.append(("delta.thinking", d["thinking"]))
            cb = ev.get("content_block") or {}
            if isinstance(cb, dict):
                if cb.get("text"):
                    blobs.append(("content_block.text", cb["text"]))
                if cb.get("thinking"):
                    blobs.append(("content_block.thinking", cb["thinking"]))

            for where, txt in blobs:
                for name, st, en in scan_text(txt):
                    hits[name] += 1
                    if len(samples) < 8:
                        samples.append((name, where, t, txt[max(0, st-50):en+50]))

    print("\n=== 事件类型 ===")
    for k, v in sorted(etypes.items(), key=lambda x: -x[1]):
        print("  %-22s %d" % (k, v))

    print("\n=== 特殊 token ===")
    total = sum(hits.values())
    if total == 0:
        print("  未发现泄漏")
    else:
        for k, v in hits.items():
            if v:
                print("  %-12s %d" % (k, v))
        print("\n=== 样本 ===")
        for name, where, t, txt in samples:
            print("  [%s] %s (%s)" % (name, where, t))
            print("     %r" % txt)
    return 0

if __name__ == "__main__":
    sys.exit(main())
