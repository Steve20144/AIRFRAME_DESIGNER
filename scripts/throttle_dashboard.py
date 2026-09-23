"""Live throttle dashboard for the aircraft over the ground telemetry radio.

Run on the machine the ground radio is plugged into (Windows: COM6), with QGC closed or left to connect over UDP:
  python scripts/throttle_dashboard.py --port COM6
then open http://127.0.0.1:8095. Needs pymavlink and pyserial.

- Every PWM output (1-16) as a bar, labelled with its motor from PWM_MAIN/AUX_FUNC; the nose-lift fans large.
- The nose-lift module's own command, state, abort reason and pitch (DEBUG_VECT "NLIFT").
- Armed, battery, radio RSSI, the last status texts; an Altitude mode button.
- Every run (arm to disarm) saved to results/telemetry_runs/run_<time>.json with its messages, nose-lift states,
  mode changes and peak outputs, and listed under Runs on the page.
- Sends a GCS heartbeat (PX4 sends no status texts otherwise). With NAV_DLL_ACT other than 0, closing the dashboard
  would count as losing the ground station.
- PX4 only streams outputs 9-16 (SERVO_OUTPUT_RAW_1) when asked, and forgets at every reboot: the dashboard asks
  again through the board's shell whenever the heartbeat comes back after a gap.
- Everything received is forwarded to udp 127.0.0.1:14550, so QGC connects at the same time (and can command back).
"""
import argparse
import json
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from pymavlink import mavutil

STATES = ["disabled", "disarmed", "parked", "raising the nose", "holding", "handing over", "flying",
          "lowering the nose", "aborted"]
ABORTS = ["none", "kill switch", "switch off", "radio lost", "attitude lost", "roll limit", "overshoot",
          "left the ground", "lift timeout", "motors cannot hold the nose", "hold timeout", "lowering timeout"]
FUNC_PARAMS = [f"PWM_MAIN_FUNC{i}" for i in range(1, 9)] + [f"PWM_AUX_FUNC{i}" for i in range(1, 9)]

lock = threading.Lock()
state = {
    "link": False, "last_hb": 0.0, "armed": False, "mode": 0,
    "pwm": [0] * 16, "pwm_t": [0.0] * 16, "funcs": [0] * 16, "lift": [],
    "nl": None, "nl_t": 0.0, "volt": None, "rssi": None, "remrssi": None, "texts": [], "rate": 0.0, "ack": None,
}
link = {}  # the aircraft connection, for commands sent from the page

# PX4 custom modes: main mode in bits 16-23, auto sub-mode in bits 24-31
MAIN_MODES = {1: "Manual", 2: "Altitude", 3: "Position", 4: "Auto", 5: "Acro", 6: "Offboard", 7: "Stabilized"}
AUTO_MODES = {1: "Ready", 2: "Takeoff", 3: "Hold", 4: "Mission", 5: "RTL", 6: "Land", 8: "Follow", 9: "Precland"}
SET_MODES = {"altitude": 2}
RESULTS = ["accepted", "temporarily rejected", "denied", "unsupported", "failed", "in progress", "cancelled"]


def mode_name(custom):
    main, sub = custom >> 16 & 0xFF, custom >> 24 & 0xFF
    if main == 4:
        return "Auto " + AUTO_MODES.get(sub, str(sub))
    return MAIN_MODES.get(main, f"mode {main}")


def set_mode(name):
    main = SET_MODES[name]
    m = link["m"]
    m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, 0, 0, 0, 0, 0)
    with lock:
        state["ack"] = {"text": f"{MAIN_MODES[main]} requested…", "ok": None, "t": time.time()}


class Recorder:
    """One JSON file per run, from arming to 3 s after disarming, with the 15 s before arming as context: every
    status text, mode change, nose-lift state or abort change, link drop, and the peak of every output. Saved every
    few seconds while it runs, so a crash or a closed window keeps what was recorded."""

    def __init__(self, folder):
        self.dir = folder
        self.dir.mkdir(parents=True, exist_ok=True)
        self.run = None
        self.before = []          # (t, kind, text) while disarmed
        self.disarm_t = None
        self.saved_t = 0.0
        self.outcome = None       # the nose lift's abort reason in this run

    def event(self, now, kind, text):
        if self.run is not None:
            self.run["events"].append({"t": now, "kind": kind, "text": text})
        else:
            self.before = [e for e in self.before if now - e["t"] < 15] + [{"t": now, "kind": kind, "text": text}]

    def arm(self, now):
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(now))
        self.run = {"name": f"run_{stamp}", "start": now, "end": None, "events": [], "peak_pwm": {}, "peak_cmd": 0.0}
        for e in self.before:
            self.run["events"].append(dict(e, before=True))
        self.before = []
        self.disarm_t = None
        self.outcome = None
        self.event(now, "armed", "armed")

    def disarm(self, now):
        if self.run is not None:
            self.event(now, "armed", "disarmed")
            self.disarm_t = now

    def peak(self, out, pwm, cmd=None):
        if self.run is None or self.disarm_t is not None:
            return
        if pwm is not None and pwm > self.run["peak_pwm"].get(out, 0):
            self.run["peak_pwm"][out] = pwm
        if cmd is not None:
            self.run["peak_cmd"] = max(self.run["peak_cmd"], cmd)

    def tick(self, now):
        if self.run is None:
            return
        done = self.disarm_t is not None and now - self.disarm_t > 3
        if done or now - self.saved_t > 5:
            self.run["end"] = self.disarm_t if done else None
            self.save()
            self.saved_t = now
        if done:
            self.run = None

    def save(self):
        r = self.run
        r["outcome"] = f"stopped: {self.outcome}" if self.outcome else ("recording" if r["end"] is None else "no abort")
        (self.dir / f"{r['name']}.json").write_text(json.dumps(r, indent=1))

    def summaries(self):
        out = []
        for f in sorted(self.dir.glob("run_*.json"), reverse=True)[:50]:
            try:
                r = json.loads(f.read_text())
            except (OSError, ValueError):
                continue
            out.append({"name": r["name"], "start": r["start"], "end": r["end"], "outcome": r.get("outcome"),
                        "peak_pwm": r["peak_pwm"], "peak_cmd": r["peak_cmd"],
                        "messages": sum(e["kind"] == "message" for e in r["events"])})
        return out

    def load(self, name):
        f = self.dir / f"{name}.json"
        if f.parent != self.dir or not f.exists():
            return None
        if self.run is not None and self.run["name"] == name:
            return self.run
        return json.loads(f.read_text())


rec = None
texts_partial = {}  # STATUSTEXT id -> text so far (PX4 splits long texts into 50-character chunks)


def as_int(f):
    return struct.unpack("<i", struct.pack("<f", f))[0]


def shell(m, cmds):
    """Run NSH commands on the board through MAVLink SERIAL_CONTROL (works over the radio)."""
    flags = mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND
    for c in ["", *cmds]:
        b = (c + "\n").encode()
        m.mav.serial_control_send(mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL, flags, 0, 0, len(b), list(b) + [0] * (70 - len(b)))
        time.sleep(0.4)
    m.mav.serial_control_send(mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL, 0, 0, 0, 0, [0] * 70)


def configure(m, dev):
    shell(m, [f"mavlink stream -d {dev} -s SERVO_OUTPUT_RAW_1 -r 10",
              f"mavlink stream -d {dev} -s SERVO_OUTPUT_RAW_0 -r 5",
              f"mavlink stream -d {dev} -s DEBUG_VECT -r 10"])
    for i, n in enumerate(FUNC_PARAMS):
        m.mav.param_request_read_send(1, 1, n.encode(), -1)
        time.sleep(0.05)
    m.mav.param_request_read_send(1, 1, b"NL_MOT_MSK", -1)


def reader(args):
    """Keeps (re)opening the port: the radio or cable may be unplugged and plugged back in."""
    while True:
        try:
            read_link(args)
        except OSError as e:  # serial.SerialException is an OSError
            link.pop("m", None)
            with lock:
                state["link"], state["last_hb"] = False, 0.0
            print(f"{args.port}: {e}; retrying")
            time.sleep(1)


def read_link(args):
    m = link["m"] = mavutil.mavlink_connection(args.port, baud=args.baud, source_system=253)
    qgc = mavutil.mavlink_connection(f"udpout:127.0.0.1:{args.qgc_port}", source_system=253) if args.qgc_port else None
    count, t_rate, t_hb = 0, time.time(), 0.0
    t_conf, boot_ms = 0.0, 0

    def reconfigure(why):
        nonlocal t_conf
        if time.time() - t_conf > 10:
            t_conf = time.time()
            print("re-adding the streams:", why)
            threading.Thread(target=configure, args=(m, args.dev), daemon=True).start()

    while True:
        # a quick reboot can hide inside the heartbeat gap: outputs 1-8 arriving while 9-16 do not means the
        # streams were lost (PX4 forgets them at every boot)
        with lock:
            fresh = time.time() - state["pwm_t"][0] < 2
            stale = [i for i in range(8, 16) if state["funcs"][i] and time.time() - state["pwm_t"][i] > 3]
        if state["link"] and fresh and stale:
            reconfigure(f"outputs {stale[0] + 1}+ missing")
        # announce a ground station once a second: PX4 sends no STATUSTEXT on a link without one (and the SiK radio
        # no RADIO_STATUS). Safe with NAV_DLL_ACT 0: closing the dashboard triggers no data-link-loss failsafe.
        if time.time() - t_hb > 1:
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            t_hb = time.time()
        if qgc:  # QGC -> aircraft (Windows refuses recvfrom on the socket until it has sent once)
            try:
                while (q := qgc.recv_msg()) is not None:
                    if q.get_type() != "BAD_DATA":
                        m.write(q.get_msgbuf())
            except OSError:
                pass
        msg = m.recv_match(blocking=True, timeout=0.05)
        now = time.time()
        with lock:
            if now - t_rate > 1:
                state["rate"], count, t_rate = count / (now - t_rate), 0, now
            up = now - state["last_hb"] < 3
            if up != state["link"]:
                rec.event(now, "link", "radio link " + ("back" if up else "lost"))
            state["link"] = up
            rec.tick(now)
        if msg is None or msg.get_type() == "BAD_DATA":
            continue
        count += 1
        if qgc:
            qgc.write(msg.get_msgbuf())
        if msg.get_srcSystem() != 1:
            if msg.get_type() == "RADIO_STATUS":
                with lock:
                    state["rssi"], state["remrssi"] = msg.rssi, msg.remrssi
            continue
        ty = msg.get_type()
        with lock:
            if ty == "HEARTBEAT" and msg.get_srcComponent() == 1:
                gap = now - state["last_hb"] > 5
                armed = bool(msg.base_mode & 128)
                if armed and not state["armed"]:
                    rec.arm(now)
                elif state["armed"] and not armed:
                    rec.disarm(now)
                if msg.custom_mode != state["mode"] and not gap:
                    rec.event(now, "mode", mode_name(msg.custom_mode))
                state["last_hb"], state["armed"], state["mode"] = now, armed, msg.custom_mode
                if gap:  # first heartbeat, or the board rebooted: ask for the streams again
                    t_conf = 0.0
                    reconfigure("new heartbeat")
            elif ty == "ATTITUDE":
                if msg.time_boot_ms + 1000 < boot_ms:  # uptime went backwards: the board rebooted
                    rec.event(now, "link", "board rebooted")
                    t_conf = 0.0
                    reconfigure("board rebooted")
                boot_ms = msg.time_boot_ms
            elif ty == "SERVO_OUTPUT_RAW" and msg.port in (0, 1):
                for i in range(8):
                    pwm = getattr(msg, f"servo{i + 1}_raw")
                    state["pwm"][msg.port * 8 + i] = pwm
                    state["pwm_t"][msg.port * 8 + i] = now
                    if pwm:
                        rec.peak(str(msg.port * 8 + i + 1), pwm)
            elif ty == "DEBUG_VECT" and msg.name.startswith("NLIFT"):
                s = int(msg.x + 1e-3)
                a = int(round((msg.x - s) * 100))
                nl = {"state": STATES[s] if s < len(STATES) else str(s),
                      "abort": ABORTS[a] if a < len(ABORTS) else str(a), "pitch": msg.y, "cmd": msg.z}
                old = state["nl"]
                if old is None or (old["state"], old["abort"]) != (nl["state"], nl["abort"]):
                    text = nl["state"] + (f" ({nl['abort']})" if nl["abort"] != "none" else "")
                    rec.event(now, "nose lift", f"{text}, pitch {nl['pitch']:.1f}, cmd {nl['cmd']:.2f}")
                    if nl["abort"] != "none":
                        rec.outcome = nl["abort"]
                rec.peak("cmd", None, nl["cmd"])
                state["nl"], state["nl_t"] = nl, now
            elif ty == "SYS_STATUS":
                state["volt"] = msg.voltage_battery / 1000 if msg.voltage_battery != 65535 else None
            elif ty == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                r = RESULTS[msg.result] if msg.result < len(RESULTS) else str(msg.result)
                state["ack"] = {"text": f"mode change {r}", "ok": msg.result == 0, "t": now}
            elif ty == "STATUSTEXT":
                text = texts_partial.pop(msg.id, "") + msg.text if msg.id else msg.text
                if msg.id and len(msg.text) >= 50:  # a full chunk: more of this text follows
                    texts_partial[msg.id] = text
                    continue
                text = text.strip()
                state["texts"] = ([time.strftime("%H:%M:%S ") + text] + state["texts"])[:8]
                rec.event(now, "message", text)
            elif ty == "PARAM_VALUE":
                if msg.param_id in FUNC_PARAMS:
                    state["funcs"][FUNC_PARAMS.index(msg.param_id)] = as_int(msg.param_value)
                elif msg.param_id == "NL_MOT_MSK":
                    mask = as_int(msg.param_value)
                    state["lift"] = [k + 1 for k in range(16) if mask >> k & 1]


PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><title>Throttle</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#0f1115;--card:#181b22;--fg:#e6e8ec;--dim:#8a909c;--bar:#3fa7ff;--hot:#ff5a4f;--warn:#ffb020;--ok:#35c26b;--track:#262a33}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif;padding:16px}
.top{display:flex;gap:10px;flex-wrap:wrap;margin-bottom:14px}.pill{background:var(--card);padding:8px 12px;border-radius:8px}
.pill b{font-variant-numeric:tabular-nums}.btn{border:1px solid var(--bar);color:var(--fg);font:inherit;cursor:pointer}
.btn:hover{background:var(--bar);color:#fff}.btn.on{background:var(--bar);color:#fff;cursor:default}.armed{background:var(--hot);color:#fff}.ok{color:var(--ok)}.bad{color:var(--hot)}
.big{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px;margin-bottom:14px}
.card{background:var(--card);border-radius:10px;padding:12px}.lbl{color:var(--dim);font-size:12px}
.vbar{height:220px;background:var(--track);border-radius:8px;position:relative;overflow:hidden;margin:8px 0}
.vfill{position:absolute;bottom:0;left:0;right:0;background:var(--bar)}
.pct{font-size:34px;font-weight:700;font-variant-numeric:tabular-nums}.us{color:var(--dim);font-variant-numeric:tabular-nums}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(88px,1fr));gap:8px}
.hbar{height:10px;background:var(--track);border-radius:5px;overflow:hidden;margin-top:6px}.hfill{height:100%;background:var(--bar)}
.runs .run{background:var(--card);border-radius:8px;margin-bottom:6px}.runs summary{padding:9px 12px;cursor:pointer;display:flex;gap:14px;flex-wrap:wrap}
.runs table{width:100%;border-collapse:collapse;font-size:12px;font-family:ui-monospace,monospace}.runs td{padding:3px 12px;border-top:1px solid var(--track);vertical-align:top}
.runs td:first-child{color:var(--dim);white-space:nowrap;width:1%}.runs .k{color:var(--dim);white-space:nowrap;width:1%}.runs .pre td{opacity:.55}
.stale{opacity:.35}.texts{font-family:ui-monospace,monospace;font-size:12px;color:var(--dim);white-space:pre-wrap;margin-top:14px}
</style></head><body>
<div class="top"><div class="top" id="top" style="margin:0"></div>
<button class="pill btn" onclick="setMode('altitude')">Altitude</button><div class="pill" id="ack" style="display:none"></div></div><div class="big" id="big"></div><div class="grid" id="grid"></div><div class="texts" id="texts"></div>
<h3 class="lbl" style="margin:20px 0 8px;font-size:13px">Runs</h3><div id="runs" class="runs"></div>
<script>
const FN=f=>f>=101&&f<=112?'M'+(f-100):f>=201&&f<=208?'S'+(f-200):f?('f'+f):'-';
const pct=u=>u<=0?0:Math.max(0,Math.min(100,(u-1000)/10));
const col=p=>p>=95?'var(--hot)':p>=80?'var(--warn)':'var(--bar)';
function render(s){
  const now=s.now, age=i=>now-s.pwm_t[i];
  const lift=new Set(s.lift.map(m=>100+m));
  let top=`<div class="pill ${s.armed?'armed':''}">${s.armed?'ARMED':'disarmed'}</div>`+
    `<div class="pill">link <b class="${s.link?'ok':'bad'}">${s.link?'OK':'LOST'}</b> ${s.rate.toFixed(0)} msg/s</div>`+
    `<div class="pill">battery <b>${s.volt==null?'-':s.volt.toFixed(2)+' V'}</b></div>`+
    `<div class="pill">RSSI <b>${s.rssi??'-'}/${s.remrssi??'-'}</b></div>`;
  if(s.nl){const st=now-s.nl_t>1.5;top+=`<div class="pill ${st?'stale':''}">nose lift <b>${s.nl.state}</b>${s.nl.abort!='none'?' <span class="bad">('+s.nl.abort+')</span>':''} · pitch <b>${s.nl.pitch.toFixed(1)}°</b> · cmd <b>${(s.nl.cmd*100).toFixed(0)}%</b></div>`}
  top=`<div class="pill">mode <b>${s.mode_name}</b></div>`+top;
  document.getElementById('top').innerHTML=top;
  document.querySelector('.btn').classList.toggle('on',s.mode_name=='Altitude');
  const ack=document.getElementById('ack');
  if(s.ack&&now-s.ack.t<8){ack.style.display='';ack.innerHTML=`<b class="${s.ack.ok===false?'bad':s.ack.ok?'ok':''}">${s.ack.text}</b>`}else ack.style.display='none';
  let big='',grid='';
  for(let i=0;i<16;i++){
    const u=s.pwm[i],p=pct(u),st=age(i)>1.5,f=s.funcs[i],name=`out ${i+1} · ${FN(f)}`;
    if(!f && s.funcs.some(x=>x)) continue;  // unassigned output (once the functions are known)
    if(lift.has(f)) big+=`<div class="card ${st?'stale':''}"><div class="lbl">${name} · nose lift</div><div class="vbar"><div class="vfill" style="height:${p}%;background:${col(p)}"></div></div><div class="pct">${p.toFixed(0)}%</div><div class="us">${u||'-'} µs${st?' · no data':''}</div></div>`;
    else grid+=`<div class="card ${st?'stale':''}"><div class="lbl">${name}</div><b>${p.toFixed(0)}%</b> <span class="us">${u||'-'}</span><div class="hbar"><div class="hfill" style="width:${p}%;background:${col(p)}"></div></div></div>`;
  }
  document.getElementById('big').innerHTML=big||'<div class="card lbl">waiting for the nose-lift motor list…</div>';
  document.getElementById('grid').innerHTML=grid;
  document.getElementById('texts').textContent=s.texts.join('\n');
}
function setMode(n){fetch('/mode?name='+n,{method:'POST'})}
const es=new EventSource('/events');es.onmessage=e=>render(JSON.parse(e.data));
const esc=t=>String(t).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));
const hm=t=>new Date(t*1000).toLocaleTimeString();
const open=new Set();let runsKey='';
async function loadRun(name,el){
  const r=await (await fetch('/runs/'+name)).json(), t0=r.start;
  el.innerHTML='<table>'+r.events.map(e=>`<tr class="${e.before?'pre':''}"><td>${(e.t-t0>=0?'+':'')+(e.t-t0).toFixed(2)} s</td><td class="k">${esc(e.kind)}</td><td>${esc(e.text)}</td></tr>`).join('')+'</table>';
}
async function refreshRuns(){
  let runs;try{runs=await (await fetch('/runs')).json()}catch(e){return}
  const key=JSON.stringify(runs);if(key==runsKey)return;runsKey=key;
  const box=document.getElementById('runs');
  box.innerHTML=runs.length?runs.map(r=>{
    const dur=r.end?((r.end-r.start).toFixed(1)+' s'):'recording…';
    const pk=Object.entries(r.peak_pwm).filter(([k,v])=>v>1000).map(([k,v])=>`out ${k} ${Math.round((v-1000)/10)}%`).join(', ')||'no throttle';
    const bad=r.outcome&&r.outcome.startsWith('stopped');
    return `<details class="run" data-name="${r.name}" ${open.has(r.name)?'open':''}><summary><b>${new Date(r.start*1000).toLocaleString()}</b><span>${dur}</span><span class="${bad?'bad':'ok'}">${esc(r.outcome||'')}</span><span class="us">peak ${pk} · ${r.messages} messages</span></summary><div class="body"></div></details>`}).join(''):'<div class="lbl">no runs recorded yet: arm to start one</div>';
  box.querySelectorAll('details').forEach(d=>{
    const body=d.querySelector('.body');
    if(d.open)loadRun(d.dataset.name,body);
    d.addEventListener('toggle',()=>{if(d.open){open.add(d.dataset.name);loadRun(d.dataset.name,body)}else open.delete(d.dataset.name)});
  });
}
refreshRuns();setInterval(refreshRuns,2000);
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_POST(self):
        name = self.path.partition("name=")[2]
        ok = self.path.startswith("/mode?") and name in SET_MODES and "m" in link
        if ok:
            set_mode(name)
        self.send_response(204 if ok else 400)
        self.end_headers()

    def send_json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/runs":
            with lock:
                return self.send_json(rec.summaries())
        if self.path.startswith("/runs/"):
            name = self.path[len("/runs/"):]
            with lock:
                r = rec.load(name) if name.replace("_", "").isalnum() else None
                return self.send_json(r) if r else self.send_json({"error": "no such run"}, 404)
        if self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    with lock:
                        s = dict(state, now=time.time(), mode_name=mode_name(state["mode"]) if state["link"] else "-")
                    self.wfile.write(f"data: {json.dumps(s)}\n\n".encode())
                    self.wfile.flush()
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                return
        body = PAGE.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="COM6", help="ground radio serial port")
    p.add_argument("--baud", type=int, default=57600)
    p.add_argument("--dev", default="/dev/ttyS1", help="the radio's port on the Pixhawk (TELEM3 on the 6X)")
    p.add_argument("--http", type=int, default=8095)
    p.add_argument("--qgc-port", type=int, default=14550, help="forward to QGC over UDP (0: off)")
    p.add_argument("--runs-dir", default=str(Path(__file__).resolve().parents[1] / "results" / "telemetry_runs"),
                   help="where each run's messages are saved")
    args = p.parse_args()
    global rec
    rec = Recorder(Path(args.runs_dir))
    print(f"runs saved to {rec.dir}")
    threading.Thread(target=reader, args=(args,), daemon=True).start()
    print(f"throttle dashboard: http://127.0.0.1:{args.http}  (radio {args.port}, QGC udp {args.qgc_port or 'off'})")
    ThreadingHTTPServer(("127.0.0.1", args.http), Handler).serve_forever()


if __name__ == "__main__":
    main()
