"""Minimal client for Autodesk Fusion's local MCP server (http://127.0.0.1:27182/mcp, JSON-RPC over HTTP).

  python scripts/fusion_mcp.py tools                      list the tools
  python scripts/fusion_mcp.py call <tool> '<json args>'  call one, print the result
"""
import json, sys, urllib.error, urllib.request

URL = "http://127.0.0.1:27182/mcp"
_id = 0
_session = None


def rpc(method, params=None, timeout=600):
    global _id, _session
    _id += 1
    body = json.dumps({"jsonrpc": "2.0", "id": _id, "method": method, "params": params or {}}).encode()
    headers = {"content-type": "application/json", "accept": "application/json, text/event-stream"}
    if _session:
        headers["MCP-Session-Id"] = _session
    req = urllib.request.Request(URL, body, headers)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        _session = r.headers.get("MCP-Session-Id") or _session
        txt = r.read().decode()
    if txt.lstrip().startswith("data:") or "\ndata:" in txt:          # SSE framing
        txt = [l[5:].strip() for l in txt.splitlines() if l.startswith("data:")][-1]
    d = json.loads(txt)
    if "error" in d:
        raise RuntimeError(d["error"])
    return d["result"]


def notify(method, params=None):
    headers = {"content-type": "application/json", "accept": "application/json, text/event-stream", "MCP-Session-Id": _session}
    body = json.dumps({"jsonrpc": "2.0", "method": method, "params": params or {}}).encode()
    try:
        urllib.request.urlopen(urllib.request.Request(URL, body, headers), timeout=30).read()
    except urllib.error.HTTPError as e:      # 202 Accepted / empty bodies are fine
        if e.code >= 400:
            raise


def init():
    rpc("initialize", {"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "airframe_designer", "version": "0"}})
    notify("notifications/initialized")


def call(tool, args=None, timeout=600):
    res = rpc("tools/call", {"name": tool, "arguments": args or {}}, timeout)
    out = []
    for c in res.get("content", []):
        out.append(c.get("text") if c.get("type") == "text" else json.dumps(c)[:500])
    return "\n".join(out), res.get("isError", False)


if __name__ == "__main__":
    init()
    if sys.argv[1] == "tools":
        for t in rpc("tools/list")["tools"]:
            print(f"== {t['name']}: {t.get('description', '')}\n   args: {json.dumps(t.get('inputSchema', {}).get('properties', {}))}")
    elif sys.argv[1] == "resources":
        print(json.dumps(rpc("resources/list"), indent=1))
    elif sys.argv[1] == "call":
        text, err = call(sys.argv[2], json.loads(sys.argv[3]) if len(sys.argv) > 3 else {})
        print(("ERROR: " if err else "") + text)
