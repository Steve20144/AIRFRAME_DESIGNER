"""A live progress page for a headless study that writes one *_ts.json per finished flight (nose_lift_smoothing.py).

  python scripts/study_progress.py --dir results/nose_lift_smoothing/hover24 --total 80 --port 8098

Shows flights done, time left (from the average pace so far), and each finished flight's nose peak, with the peak
coloured against --limit. Standard library only, so it runs from Windows Python next to a study running in WSL.
"""
import argparse
import json
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HOVER = 24.0          # the time series' pitch is in the hover frame; the nose angle is pitch + hover pitch
_cache = {}


def flight(path):
    """Headline numbers of one finished flight, cached by modification time."""
    mt = path.stat().st_mtime
    hit = _cache.get(path)
    if hit and hit[0] == mt:
        return hit[1]
    try:
        d = json.loads(path.read_text())
    except (OSError, ValueError):                       # still being written
        return None
    r, ts = d.get("result", {}), d.get("timeseries", {})
    col = {k: i for i, k in enumerate(ts.get("columns", []))}
    peaks = {}
    for row, ph in zip(ts.get("rows", []), ts.get("phase", [])):
        if ph in ("raise", "switch_off", "lowered"):
            nose = row[col["pitch"]] * 57.29578 + HOVER
            key = "raise" if ph == "raise" else "after"
            peaks[key] = max(peaks.get(key, -99.0), nose)
    out = {"id": path.name[:-8].replace("___", " | ").replace("_", " "), "done_at": mt, "ok": bool(r.get("ok")),
           "status": r.get("status"), "raise_max": round(peaks["raise"], 1) if "raise" in peaks else None,
           "after_max": round(peaks["after"], 1) if "after" in peaks else None}
    _cache[path] = (mt, out)
    return out


def status(folder, total, start):
    files = sorted(folder.glob("*_ts.json"), key=lambda p: p.stat().st_mtime)
    rows = [f for f in (flight(p) for p in files) if f]
    now = time.time()
    done = len(rows)
    elapsed = now - start
    left = elapsed / done * max(total - done, 0) if done else None
    return {"done": done, "total": total, "elapsed_s": round(elapsed), "left_s": None if left is None else round(left),
            "finished": done >= total, "flights": rows[::-1], "name": folder.name}


PAGE = """<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Study Progress</title>
<style>
:root{--bg:#f6f7f9;--card:#fff;--ink:#16181d;--mute:#5d6470;--line:#e2e5ea;--bar:#2f6fed;--ok:#1f8a4c;--warn:#b7791f;--bad:#c53030}
@media (prefers-color-scheme:dark){:root:not([data-theme=light]){--bg:#0f1115;--card:#171a21;--ink:#e8eaee;--mute:#9aa1ad;--line:#2a2f3a;--bar:#5b8cff;--ok:#48bb78;--warn:#ecc94b;--bad:#fc8181}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.45 system-ui,-apple-system,Segoe UI,sans-serif}
main{max-width:860px;margin:0 auto;padding:24px 16px}
h1{font-size:20px;margin:0 0 4px}.sub{color:var(--mute);margin:0 0 20px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:18px;margin-bottom:16px}
.big{display:flex;gap:28px;flex-wrap:wrap;align-items:baseline}.big b{font-size:34px;font-variant-numeric:tabular-nums}
.big span{color:var(--mute);display:block;font-size:13px}
.track{height:12px;background:var(--line);border-radius:6px;overflow:hidden;margin-top:14px}
.fill{height:100%;background:var(--bar);width:0;transition:width .6s ease}
.done .fill{background:var(--ok)}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}
th,td{text-align:left;padding:7px 6px;border-bottom:1px solid var(--line)}th{color:var(--mute);font-weight:500;font-size:13px}
td.n{text-align:right}.ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}
.wrap{overflow-x:auto}.pulse{display:inline-block;width:9px;height:9px;border-radius:50%;background:var(--bar);margin-right:6px;animation:p 1.2s infinite}
.done .pulse{background:var(--ok);animation:none}@keyframes p{50%{opacity:.25}}
</style></head><body><main id="m">
<h1><span class="pulse"></span><span id="title">Study</span></h1><p class="sub" id="sub">Connecting…</p>
<div class="card"><div class="big">
<div><b id="done">0</b><span>flights done</span></div><div><b id="left">…</b><span>time left</span></div>
<div><b id="elapsed">0:00</b><span>elapsed</span></div></div><div class="track"><div class="fill" id="fill"></div></div></div>
<div class="card wrap"><table><thead><tr><th>Finished flight</th><th class="n">Peak rising</th><th class="n">Peak after</th><th>Result</th></tr></thead>
<tbody id="rows"></tbody></table></div></main>
<script>
const LIMIT = __LIMIT__;
const fmt = s => s == null ? "…" : `${Math.floor(s / 60)}:${String(s % 60).padStart(2, "0")}`;
const cls = v => v == null ? "" : v <= LIMIT ? "ok" : v <= LIMIT + 4 ? "warn" : "bad";
const esc = s => String(s).replace(/[&<>]/g, c => ({"&": "&amp;", "<": "&lt;", ">": "&gt;"}[c]));
async function tick() {
  try {
    const s = await (await fetch("status")).json();
    document.getElementById("title").textContent = s.finished ? `${s.name}: finished` : `${s.name}: running`;
    document.getElementById("sub").textContent = `${s.done} of ${s.total} flights · peaks coloured against ${LIMIT} deg`;
    document.getElementById("done").textContent = `${s.done}/${s.total}`;
    document.getElementById("left").textContent = s.finished ? "done" : fmt(s.left_s);
    document.getElementById("elapsed").textContent = fmt(s.elapsed_s);
    document.getElementById("fill").style.width = `${Math.min(100, 100 * s.done / s.total)}%`;
    document.getElementById("m").classList.toggle("done", s.finished);
    document.getElementById("rows").innerHTML = s.flights.map(f => `<tr><td>${esc(f.id)}</td>` +
      `<td class="n ${cls(f.raise_max)}">${f.raise_max == null ? "-" : f.raise_max.toFixed(1)}</td>` +
      `<td class="n ${cls(f.after_max)}">${f.after_max == null ? "-" : f.after_max.toFixed(1)}</td>` +
      `<td class="${f.ok ? "ok" : "bad"}">${f.ok ? "flew" : esc(f.status || "failed")}</td></tr>`).join("");
  } catch (e) { document.getElementById("sub").textContent = "Lost the progress server; retrying…"; }
}
tick(); setInterval(tick, 2000);
</script></body></html>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--total", type=int, required=True)
    ap.add_argument("--limit", type=float, default=26.0, help="nose peak shown green up to this, deg")
    ap.add_argument("--start", type=float, default=None, help="epoch the study started (default: the folder's creation)")
    ap.add_argument("--port", type=int, default=8098)
    a = ap.parse_args()
    folder = Path(a.dir)
    start = a.start or folder.stat().st_ctime       # Windows: the folder's creation time
    page = PAGE.replace("__LIMIT__", repr(a.limit)).encode()

    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.startswith("/status"):
                body, kind = json.dumps(status(folder, a.total, start), allow_nan=False, default=str).encode(), "application/json"
            else:
                body, kind = page, "text/html; charset=utf-8"
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *_):
            pass

    ThreadingHTTPServer(("127.0.0.1", a.port), H).serve_forever()


if __name__ == "__main__":
    main()
