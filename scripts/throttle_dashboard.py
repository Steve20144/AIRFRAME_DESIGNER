"""Live throttle dashboard for the aircraft over the ground telemetry radio.

Run on the machine the ground radio is plugged into (Windows: COM6), with QGC closed or left to connect over UDP:
  python scripts/throttle_dashboard.py --port COM6
then open http://127.0.0.1:8095. Needs pymavlink and pyserial.

- Every PWM output (1-16) as a bar, labelled with its motor from PWM_MAIN/AUX_FUNC; the nose-lift (front) fans large,
  every other motor (the rear foil fans) as a row of tall bars below them.
- The nose-lift module's own command, state, abort reason and pitch (DEBUG_VECT "NLIFT").
- Armed, battery, radio RSSI, the last status texts; flight mode buttons (Stabilized, Altitude, Position, Hold, Land).
- Set hover pitch: the angle PX4 treats as level (disarmed only). Moves everything that depends on it together and
  consistently: SENS_BOARD_Y_OFF, the nose-lift target (NL_TGT, NL_HOV_PITCH), every rotor's position and thrust axis
  in PX4's frame (CA_ROTOR*_PX/PZ/AX/AZ, rotated by the change), the H-FLOW's range tilt and lever arms
  (EKF2_RNG_PITCH, EKF2_OF_POS / EKF2_RNG_POS); then saves and reboots the board so the estimator re-aligns.
- Every run (arm to disarm) saved to results/telemetry_runs/run_<time>.json with its messages, nose-lift states,
  mode changes and peak outputs, and listed under Runs on the page.
- Every flight's live log, arm to disarm: the live values 10 times a second (mode, link, battery, RSSI, attitude,
  nose lift, every output) and every event, each row timestamped (local time, seconds since arming, the board's
  uptime to line up with its ULog), in results/telemetry_runs/log_<time>.csv; the Logs tab lists, shows and
  downloads them.
- Sends a GCS heartbeat (PX4 sends no status texts otherwise). With NAV_DLL_ACT other than 0, closing the dashboard
  would count as losing the ground station.
- PX4 only streams outputs 9-16 (SERVO_OUTPUT_RAW_1) when asked, and forgets at every reboot: the dashboard asks
  again through the board's shell whenever the heartbeat comes back after a gap.
- Everything received is forwarded to udp 127.0.0.1:14550, so QGC connects at the same time (and can command back).
"""
import argparse
import csv
import json
import math
import os
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

os.environ.setdefault("MAVLINK20", "1")  # PX4 speaks MAVLink 2; STATUSTEXT's chunk id exists only there
from pymavlink import mavutil  # noqa: E402

STATES =["disabled", "disarmed", "parked", "raising the nose", "holding", "handing over", "flying",
          "lowering the nose", "aborted", "holding the nose"]
ABORTS = ["none", "kill switch", "switch off", "radio lost", "attitude lost", "roll limit", "overshoot",
          "left the ground", "lift timeout", "motors cannot hold the nose", "hold timeout", "lowering timeout"]
FUNC_PARAMS = [f"PWM_MAIN_FUNC{i}" for i in range(1, 9)] + [f"PWM_AUX_FUNC{i}" for i in range(1, 9)]

lock = threading.Lock()
state = {
    "link": False, "last_hb": 0.0, "armed": False, "mode": 0,
    "pwm": [0] * 16, "pwm_t": [0.0] * 16, "funcs": [0] * 16, "lift": [],
    "nl": None, "nl_t": 0.0, "volt": None, "rssi": None, "remrssi": None, "texts": [], "rate": 0.0, "ack": None,
    "att": None, "att_t": 0.0, "boot_ms": None, "boot_t": 0.0,
    "params": {}, "hover": {"deg": None, "busy": False, "msg": "", "ok": None, "t": 0.0, "readback": None},
}
INT_TYPES = {mavutil.mavlink.MAV_PARAM_TYPE_INT32, mavutil.mavlink.MAV_PARAM_TYPE_UINT32, mavutil.mavlink.MAV_PARAM_TYPE_INT16,
             mavutil.mavlink.MAV_PARAM_TYPE_UINT16, mavutil.mavlink.MAV_PARAM_TYPE_INT8, mavutil.mavlink.MAV_PARAM_TYPE_UINT8}
link = {}  # the aircraft connection, for commands sent from the page

# PX4 custom modes: main mode in bits 16-23, auto sub-mode in bits 24-31
MAIN_MODES = {1: "Manual", 2: "Altitude", 3: "Position", 4: "Auto", 5: "Acro", 6: "Offboard", 7: "Stabilized"}
AUTO_MODES = {1: "Ready", 2: "Takeoff", 3: "Hold", 4: "Mission", 5: "RTL", 6: "Land", 8: "Follow", 9: "Precland"}
# buttons on the page: name -> (main mode, auto sub-mode)
SET_MODES = {"stabilized": (7, 0), "altitude": (2, 0), "position": (3, 0), "hold": (4, 3), "land": (4, 6)}
RESULTS = ["accepted", "temporarily rejected", "denied", "unsupported", "failed", "in progress", "cancelled"]


def mode_name(custom):
    main, sub = custom >> 16 & 0xFF, custom >> 24 & 0xFF
    if main == 4:
        return "Auto " + AUTO_MODES.get(sub, str(sub))
    return MAIN_MODES.get(main, f"mode {main}")


def set_mode(name):
    main, sub = SET_MODES[name]
    m = link["m"]
    m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub, 0, 0, 0, 0)
    with lock:
        state["ack"] = {"text": f"{mode_name(main << 16 | sub << 24)} requested…", "ok": None, "t": time.time()}


def hover_msg(text, ok=None, busy=True):
    with lock:
        state["hover"].update(msg=text, ok=ok, busy=busy, t=time.time())
    print("[hover]", text)


def read_params(m, names, timeout=20.0):
    """Ask the board for ``names`` and wait for every answer (resending the missing ones); {name: (value, type)}."""
    t_ask = time.time()
    for n in names:
        m.mav.param_request_read_send(1, 1, n.encode(), -1); time.sleep(0.02)
    t0 = time.time()
    while time.time() - t0 < timeout:
        with lock:
            got = {n: state["params"][n] for n in names if n in state["params"] and state["params"][n][2] >= t_ask}
        if len(got) == len(names):
            return {n: (v, t) for n, (v, t, _) in got.items()}
        if time.time() - t_ask > 1.5:
            t_ask = time.time()
            for n in names:
                if n not in got:
                    m.mav.param_request_read_send(1, 1, n.encode(), -1); time.sleep(0.02)
        time.sleep(0.1)
    missing = [n for n in names if n not in got]
    raise RuntimeError(f"no answer for {', '.join(missing[:6])}{'…' if len(missing) > 6 else ''}")


def write_param(m, name, value, ptype, tries=6):
    want = float(value)
    raw = struct.unpack("<f", struct.pack("<i", int(round(want))))[0] if ptype in INT_TYPES else want
    for _ in range(tries):
        t_ask = time.time()
        m.mav.param_set_send(1, 1, name.encode(), raw, ptype)
        while time.time() - t_ask < 1.5:
            with lock:
                e = state["params"].get(name)
            if e and e[2] >= t_ask and abs(float(e[0]) - want) <= 1e-4 * max(1.0, abs(want)):
                return True
            time.sleep(0.05)
    return False


def readback(m, want=None):
    """Read the hover pitch and the nose-lift target back from the board (plus every value in ``want``, compared) and
    put the result on the page: what the board itself reports now, not what was sent."""
    keys = ["SENS_BOARD_Y_OFF", "NL_TGT", "NL_HOV_PITCH"]
    names = sorted(set(keys) | set(want or {}))
    got = read_params(m, names)
    bad = {k: (got[k][0], v) for k, (v, _) in (want or {}).items()
           if abs(float(got[k][0]) - float(v)) > 1e-3 * max(1.0, abs(float(v)))}
    hov, tgt, hp = (float(got[k][0]) for k in keys)
    consistent = abs(hov - tgt) < 0.01 and abs(hov - hp) < 0.01
    rb = {"t": time.time(), "hover": round(hov, 3), "nl_tgt": round(tgt, 3), "nl_hov": round(hp, 3),
          "checked": len(want or {}), "bad": {k: [round(a, 4), round(b, 4)] for k, (a, b) in bad.items()},
          "ok": consistent and not bad}
    with lock:
        state["hover"]["readback"] = rb
        state["hover"]["deg"] = round(hov, 2)
    return rb


def set_hover(deg):
    """Make ``deg`` the hover pitch: rotate PX4's frame-dependent parameters by the change, save, reboot."""
    try:
        m = link.get("m")
        if m is None:
            raise RuntimeError("no link to the aircraft")
        with lock:
            armed, nl = state["armed"], state["nl"]
        if armed:
            raise RuntimeError("disarm first")
        hover_msg("reading the current geometry from the board…")
        base = read_params(m, ["SENS_BOARD_Y_OFF", "CA_ROTOR_COUNT", "FD_FAIL_P", "NL_TGT", "NL_HOV_PITCH"])
        old, n = float(base["SENS_BOARD_Y_OFF"][0]), int(base["CA_ROTOR_COUNT"][0])
        if nl is not None and deg - float(nl["pitch"]) > float(base["FD_FAIL_P"][0]) - 2:
            raise RuntimeError(f"parked at {nl['pitch']:.1f}° the board would be {deg - nl['pitch']:.0f}° off its new level, "
                               f"past FD_FAIL_P {base['FD_FAIL_P'][0]:.0f}°: it would refuse to arm")
        names = [f"CA_ROTOR{i}_{a}" for i in range(n) for a in ("PX", "PZ", "AX", "AZ")]
        names += ["EKF2_RNG_PITCH", "EKF2_OF_POS_X", "EKF2_OF_POS_Z", "EKF2_RNG_POS_X", "EKF2_RNG_POS_Z"]
        cur = read_params(m, names)
        d = math.radians(deg - old)
        c, s_ = math.cos(d), math.sin(d)
        new = {"SENS_BOARD_Y_OFF": (round(deg, 3), base["SENS_BOARD_Y_OFF"][1]),
               "NL_TGT": (round(deg, 3), base["NL_TGT"][1]), "NL_HOV_PITCH": (round(deg, 3), base["NL_HOV_PITCH"][1]),
               "EKF2_RNG_PITCH": (round(cur["EKF2_RNG_PITCH"][0] + d, 4), cur["EKF2_RNG_PITCH"][1])}
        # a vector fixed to the airframe, seen in a frame pitched d further nose-up: x' = c x + s z, z' = -s x + c z
        for xk, zk in [(f"CA_ROTOR{i}_PX", f"CA_ROTOR{i}_PZ") for i in range(n)] + [(f"CA_ROTOR{i}_AX", f"CA_ROTOR{i}_AZ") for i in range(n)] \
                + [("EKF2_OF_POS_X", "EKF2_OF_POS_Z"), ("EKF2_RNG_POS_X", "EKF2_RNG_POS_Z")]:
            x, z = cur[xk][0], cur[zk][0]
            new[xk] = (round(c * x + s_ * z, 4), cur[xk][1])
            new[zk] = (round(-s_ * x + c * z, 4), cur[zk][1])
        hover_msg(f"writing {len(new)} parameters: hover {old:g}° → {deg:g}°…")
        bad = [k for k, (v, t) in new.items() if not write_param(m, k, v, t)]
        if bad:
            raise RuntimeError(f"not confirmed: {', '.join(bad[:6])} (nothing rebooted; check and retry)")
        m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(1.5)
        with lock:
            still_disarmed = not state["armed"]
        if rec is not None:
            rec.event(time.time(), "hover pitch", f"{old:g} -> {deg:g} deg ({len(new)} params)")
        if not still_disarmed:
            readback(m, new)
            hover_msg(f"hover pitch {deg:g}° saved; armed meanwhile, so not rebooted: reboot before flying", True, False)
            return
        hover_msg(f"hover pitch {deg:g}° saved ({len(new)} parameters); board rebooting, then reading it back…")
        t_reboot = time.time()
        m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
        # wait for the board to go quiet and come back (the reader reopens the port by itself)
        time.sleep(6)
        rb = None
        while time.time() - t_reboot < 90:
            m = link.get("m")
            with lock:
                alive = m is not None and state["link"] and time.time() - state["last_hb"] < 2
            if alive:
                try:
                    rb = readback(m, new)
                    break
                except Exception:
                    pass
            time.sleep(2)
        if rb is None:
            hover_msg(f"hover pitch {deg:g}° saved, but the board did not answer after the reboot: press Read back", False, False)
        elif rb["ok"]:
            hover_msg(f"verified after reboot: hover pitch {rb['hover']:g}°, nose-lift target {rb['nl_tgt']:g}°, "
                      f"{rb['checked']}/{rb['checked']} parameters match", True, False)
        else:
            hover_msg(f"read back after reboot: {len(rb['bad'])} parameter(s) differ ({', '.join(list(rb['bad'])[:4])}): set it again",
                      False, False)
    except Exception as e:
        hover_msg(f"hover pitch not changed: {e}", False, False)


FN_NAMES = {**{101 + i: f"M{i + 1}" for i in range(12)}, **{201 + i: f"S{i + 1}" for i in range(8)}}


class LiveLog:
    """One CSV per flight, from arming to disarming: the dashboard's live values ``hz`` times a second and every
    event as its own row, each timestamped three ways (local time, seconds since arming, the board's uptime, which
    lines up with its ULog). Appended and flushed row by row, so a crash or a closed window keeps what was recorded.
    Values older than ``STALE`` seconds are left blank rather than repeated."""

    STALE = 1.5
    HEAD = ["time", "epoch", "since_arm_s", "board_uptime_s", "armed", "mode", "link", "msg_rate", "battery_v", "rssi",
            "remote_rssi", "px4_roll_deg", "px4_pitch_deg", "px4_yaw_deg", "nl_state", "nl_abort", "nl_pitch_deg", "nl_cmd"]
    # px4_*: ATTITUDE, in PX4's frame (level = the hover pitch on ATLAS); nl_pitch_deg: the nose lift's nose angle

    def __init__(self, folder, hz=10.0):
        self.dir = folder
        self.dt = 1.0 / hz
        self.f = self.w = None
        self.name = None
        self.start = None
        self.last = 0.0
        self._cache = {}          # name -> (mtime, summary)

    @staticmethod
    def stamp(t):
        return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t)) + f".{int(t * 1000) % 1000:03d}"

    def open(self, now, stamp, st):
        self.close()
        self.name, self.start, self.last = f"log_{stamp}", now, 0.0
        self.f = open(self.dir / f"{self.name}.csv", "w", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        outs = [f"out{i + 1}" + (f" {FN_NAMES[f]}" if f in FN_NAMES else "") for i, f in enumerate(st["funcs"])]
        self.w.writerow(self.HEAD + outs + ["event", "text"])
        self.f.flush()

    def close(self):
        if self.f is not None:
            self.f.close()
        self.f = self.w = None
        self.name = None

    def _row(self, now, st, kind="", text=""):
        fresh = lambda t: now - t < self.STALE
        up = st["boot_ms"] is not None and fresh(st["boot_t"])
        att = st["att"] if st["att"] is not None and fresh(st["att_t"]) else None
        nl = st["nl"] if st["nl"] is not None and fresh(st["nl_t"]) else None
        r = lambda v, d: "" if v is None else round(v, d)
        # the arm / disarm rows are written as the heartbeat changes it, before state["armed"] follows
        armed = (text == "armed") if kind == "armed" else st["armed"]
        row = [self.stamp(now), f"{now:.3f}", f"{now - self.start:.3f}",
               f"{st['boot_ms'] / 1000 + (now - st['boot_t']):.3f}" if up else "",
               int(armed), mode_name(st["mode"]) if st["link"] else "", int(st["link"]), r(st["rate"], 1),
               r(st["volt"], 2), "" if st["rssi"] is None else st["rssi"], "" if st["remrssi"] is None else st["remrssi"],
               *((r(a, 2) for a in att) if att else ("", "", "")),
               *((nl["state"], nl["abort"], r(nl["pitch"], 2), r(nl["cmd"], 3)) if nl else ("", "", "", "")),
               *(st["pwm"][i] if fresh(st["pwm_t"][i]) else "" for i in range(16)), kind, text]
        self.w.writerow(row)
        self.f.flush()

    def sample(self, now, st):
        if self.f is not None and now - self.last >= self.dt:
            self.last = now
            self._row(now, st)

    def event(self, now, st, kind, text):
        if self.f is not None:
            self._row(now, st, kind, text)

    # ---------------------------------------------------------------- reading
    def _read(self, name):
        f = self.dir / f"{name}.csv"
        if f.parent != self.dir or not f.exists():
            return None
        with open(f, newline="", encoding="utf-8") as fh:
            rows = list(csv.reader(fh))
        return rows[0], rows[1:]

    def summaries(self):
        out = []
        for f in sorted(self.dir.glob("log_*.csv"), reverse=True)[:50]:
            name, mt = f.stem, f.stat().st_mtime
            hit = self._cache.get(name)
            if hit is None or hit[0] != mt:
                got = self._read(name)
                if got is None:
                    continue
                head, rows = got
                ev = head.index("event")
                s = {"name": name, "start": float(rows[0][1]) if rows else None, "end": float(rows[-1][1]) if rows else None,
                     "samples": sum(1 for x in rows if not x[ev]), "events": sum(1 for x in rows if x[ev]),
                     "bytes": f.stat().st_size}
                self._cache[name] = hit = (mt, s)
            out.append(dict(hit[1], recording=name == self.name))
        return out

    def rows(self, name):
        got = self._read(name)
        if got is None:
            return None
        head, rows = got
        return {"name": name, "columns": head, "rows": rows, "recording": name == self.name}

    def csv_bytes(self, name):
        f = self.dir / f"{name}.csv"
        return f.read_bytes() if f.parent == self.dir and f.exists() else None


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
        self.log = LiveLog(folder)  # the flight's live values, arm to disarm (the Logs tab)

    def event(self, now, kind, text):
        self.log.event(now, state, kind, text)
        if self.run is not None:
            self.run["events"].append({"t": now, "kind": kind, "text": text})
        else:
            self.before = [e for e in self.before if now - e["t"] < 15] + [{"t": now, "kind": kind, "text": text}]

    def sample(self, now):
        self.log.sample(now, state)

    def arm(self, now):
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(now))
        self.log.open(now, stamp, state)
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
        elif self.log.f is not None:
            self.log.event(now, state, "armed", "disarmed")
        self.log.close()

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
    m.mav.param_request_read_send(1, 1, b"SENS_BOARD_Y_OFF", -1)


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
            rec.sample(now)
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
                new_mode = msg.custom_mode != state["mode"] and not gap
                state["last_hb"], state["armed"], state["mode"] = now, armed, msg.custom_mode
                if new_mode:  # after the update, so the mode change's log row shows the new mode
                    rec.event(now, "mode", mode_name(msg.custom_mode))
                if gap:  # first heartbeat, or the board rebooted: ask for the streams again
                    t_conf = 0.0
                    reconfigure("new heartbeat")
            elif ty == "ATTITUDE":
                if msg.time_boot_ms + 1000 < boot_ms:  # uptime went backwards: the board rebooted
                    rec.event(now, "link", "board rebooted")
                    t_conf = 0.0
                    reconfigure("board rebooted")
                boot_ms = msg.time_boot_ms
                state["boot_ms"], state["boot_t"] = msg.time_boot_ms, now
                state["att"] = [msg.roll * 57.29578, msg.pitch * 57.29578, msg.yaw * 57.29578]
                state["att_t"] = now
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
                state["nl"], state["nl_t"] = nl, now      # before the event, so its log row shows the new state
                if old is None or (old["state"], old["abort"]) != (nl["state"], nl["abort"]):
                    text = nl["state"] + (f" ({nl['abort']})" if nl["abort"] != "none" else "")
                    rec.event(now, "nose lift", f"{text}, pitch {nl['pitch']:.1f}, cmd {nl['cmd']:.2f}")
                    if nl["abort"] != "none":
                        rec.outcome = nl["abort"]
                rec.peak("cmd", None, nl["cmd"])
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
                v = as_int(msg.param_value) if msg.param_type in INT_TYPES else float(msg.param_value)
                state["params"][msg.param_id] = (v, msg.param_type, now)
                if msg.param_id == "SENS_BOARD_Y_OFF":
                    state["hover"]["deg"] = round(v, 2)
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
.big{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px;margin-bottom:14px}.big.front{grid-template-columns:repeat(3,minmax(0,1fr))}
.modes{display:inline-flex;gap:6px;flex-wrap:wrap}
.hov{display:inline-flex;align-items:center;gap:6px}.hov input{width:64px;background:var(--bg);color:var(--fg);border:1px solid var(--track);border-radius:6px;padding:4px 6px;font:inherit}
.hovbtn{background:var(--card);border-radius:6px;padding:4px 10px}.rbline{margin:-6px 0 12px;font-size:13px;color:var(--dim)}.rbline b{color:var(--fg)}.hovbtn:disabled{opacity:.4;cursor:not-allowed}
.card{background:var(--card);border-radius:10px;padding:12px}.lbl{color:var(--dim);font-size:12px}
.vbar{height:220px;background:var(--track);border-radius:8px;position:relative;overflow:hidden;margin:8px 0}
.vfill{position:absolute;bottom:0;left:0;right:0;background:var(--bar)}
.rear{grid-template-columns:repeat(auto-fit,minmax(96px,1fr))}.rear .vbar{height:160px}.rear .pct{font-size:24px}
.row{margin:0 0 6px;font-size:13px}
.pct{font-size:34px;font-weight:700;font-variant-numeric:tabular-nums}.us{color:var(--dim);font-variant-numeric:tabular-nums}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(88px,1fr));gap:8px}
.hbar{height:10px;background:var(--track);border-radius:5px;overflow:hidden;margin-top:6px}.hfill{height:100%;background:var(--bar)}
.runs .run{background:var(--card);border-radius:8px;margin-bottom:6px}.runs summary{padding:9px 12px;cursor:pointer;display:flex;gap:14px;flex-wrap:wrap}
.runs table{width:100%;border-collapse:collapse;font-size:12px;font-family:ui-monospace,monospace}.runs td{padding:3px 12px;border-top:1px solid var(--track);vertical-align:top}
.runs td:first-child{color:var(--dim);white-space:nowrap;width:1%}.runs .k{color:var(--dim);white-space:nowrap;width:1%}.runs .pre td{opacity:.55}
.stale{opacity:.35}.texts{font-family:ui-monospace,monospace;font-size:12px;color:var(--dim);white-space:pre-wrap;margin-top:14px}
.tabs{display:flex;gap:4px;margin-bottom:14px;border-bottom:1px solid var(--track)}
.tab{background:none;border:0;border-bottom:2px solid transparent;color:var(--dim);font:inherit;font-weight:600;padding:8px 14px;cursor:pointer}
.tab.on{color:var(--fg);border-bottom-color:var(--bar)}.tab .rec{color:var(--hot);margin-left:6px}
.logbar{display:flex;gap:10px;align-items:center;flex-wrap:wrap;padding:8px 12px;border-top:1px solid var(--track)}
.logbar select,.logbar a{background:var(--bg);color:var(--fg);border:1px solid var(--track);border-radius:6px;padding:4px 8px;font:inherit;font-size:12px;text-decoration:none}
.logbar a:hover{border-color:var(--bar)}.tblwrap{overflow:auto;max-height:70vh}
.logs table{border-collapse:collapse;font-size:12px;font-family:ui-monospace,monospace;white-space:nowrap}
.logs th{position:sticky;top:0;background:var(--card);color:var(--dim);font-weight:500;text-align:right;padding:4px 8px}
.logs td{padding:2px 8px;border-top:1px solid var(--track);text-align:right;font-variant-numeric:tabular-nums}
.logs td.l,.logs th.l{text-align:left}.logs tr.ev td{background:#1f2430}.logs tr.ev td.txt{color:var(--warn);white-space:normal;min-width:260px}
</style></head><body>
<nav class="tabs"><button class="tab on" data-tab="live">Live</button><button class="tab" data-tab="logs">Logs<span class="rec" id="recdot"></span></button></nav>
<section id="tab-live">
<div class="top"><div class="top" id="top" style="margin:0"></div>
<span class="modes"><button class="pill btn" data-mode="Stabilized" onclick="setMode('stabilized')">Stabilized</button><button class="pill btn" data-mode="Altitude" onclick="setMode('altitude')">Altitude</button><button class="pill btn" data-mode="Position" onclick="setMode('position')">Position</button><button class="pill btn" data-mode="Auto Hold" onclick="setMode('hold')">Hold</button><button class="pill btn" data-mode="Auto Land" onclick="setMode('land')">Land</button></span><span class="pill hov">hover pitch <b id="hovnow">-</b>° → <input id="hovdeg" type="number" step="0.5" min="-10" max="60"> <button class="btn hovbtn" onclick="setHover()">Set hover pitch</button><button class="btn hovbtn rb" onclick="fetch('/readback',{method:'POST'})">Read back</button></span><div class="pill" id="ack" style="display:none"></div><div class="pill" id="hovmsg" style="display:none"></div></div><div class="rbline" id="rbline"></div><h3 class="lbl row">Front · nose lift (left · centre · right, seen from behind)</h3><div class="big front" id="big"></div><h3 class="lbl row">Rear</h3><div class="big rear" id="rear"></div><div class="grid" id="grid"></div><div class="texts" id="texts"></div>
<h3 class="lbl" style="margin:20px 0 8px;font-size:13px">Runs</h3><div id="runs" class="runs"></div>
</section>
<section id="tab-logs" hidden>
<div class="lbl" style="margin-bottom:10px">Each flight from arm to disarm: the live values 10 times a second and every event, timestamped with the local time, the seconds since arming and the board's uptime (to line up with its ULog). Saved as CSV in the runs folder while it records.</div>
<div id="logs" class="runs logs"></div>
</section>
<script>
const SIDE={107:'left',109:'centre',108:'right'}, ORDER={107:0,109:1,108:2};  // V3 nose fans, seen from behind
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
  document.querySelectorAll('.btn[data-mode]').forEach(b=>b.classList.toggle('on',s.mode_name==b.dataset.mode));
  if(s.hover){document.getElementById('hovnow').textContent=s.hover.deg??'-';
    const hb=document.querySelector('.hovbtn');hb.disabled=!!(s.armed||s.hover.busy);
    const hm=document.getElementById('hovmsg');if(s.hover.msg&&(s.hover.busy||now-s.hover.t<20)){hm.style.display='';hm.innerHTML=`<b class="${s.hover.ok===false?'bad':s.hover.ok?'ok':''}">${s.hover.msg}</b>`}else hm.style.display='none';
    const inp=document.getElementById('hovdeg');if(inp.value===''&&s.hover.deg!=null)inp.value=s.hover.deg;
    const rb=s.hover.readback, rl=document.getElementById('rbline');
    rl.innerHTML=rb?`<span class="${rb.ok?'ok':'bad'}">${rb.ok?'✓':'✗'}</span> read back from the board at ${new Date(rb.t*1000).toLocaleTimeString()}: `+
      `hover pitch (SENS_BOARD_Y_OFF) <b>${rb.hover}°</b> · nose-lift target (NL_TGT) <b>${rb.nl_tgt}°</b> · NL_HOV_PITCH <b>${rb.nl_hov}°</b>`+
      (rb.checked?` · ${rb.checked-Object.keys(rb.bad).length}/${rb.checked} written parameters match`:'')+
      (Object.keys(rb.bad).length?` · <span class="bad">differ: ${Object.keys(rb.bad).join(', ')}</span>`:''):'';
    document.querySelector('.hovbtn.rb').disabled=!!s.hover.busy}
  const ack=document.getElementById('ack');
  if(s.ack&&now-s.ack.t<8){ack.style.display='';ack.innerHTML=`<b class="${s.ack.ok===false?'bad':s.ack.ok?'ok':''}">${s.ack.text}</b>`}else ack.style.display='none';
  let big='',rear='',grid='';const front={};
  const vcard=(name,u,p,st)=>`<div class="card ${st?'stale':''}"><div class="lbl">${name}</div><div class="vbar"><div class="vfill" style="height:${p}%;background:${col(p)}"></div></div><div class="pct">${p.toFixed(0)}%</div><div class="us">${u||'-'} µs${st?' · no data':''}</div></div>`;
  for(let i=0;i<16;i++){
    const u=s.pwm[i],p=pct(u),st=age(i)>1.5,f=s.funcs[i],name=`out ${i+1} · ${FN(f)}`;
    if(!f && s.funcs.some(x=>x)) continue;  // unassigned output (once the functions are known)
    if(lift.has(f)) front[f]=vcard(name+(SIDE[f]?' · '+SIDE[f]:''),u,p,st);
    else if(f>=101&&f<=112&&s.lift.length) rear+=vcard(name,u,p,st);  // every other motor: the rear foil fans
    else grid+=`<div class="card ${st?'stale':''}"><div class="lbl">${name}</div><b>${p.toFixed(0)}%</b> <span class="us">${u||'-'}</span><div class="hbar"><div class="hfill" style="width:${p}%;background:${col(p)}"></div></div></div>`;
  }
  big=[...lift].sort((a,b)=>(ORDER[a]??a)-(ORDER[b]??b)).map(f=>front[f]||'').join('');
  const bg=document.getElementById('big');bg.style.gridTemplateColumns=`repeat(${Math.max(lift.size,1)},minmax(0,1fr))`;
  bg.innerHTML=big||'<div class="card lbl">waiting for the nose-lift motor list…</div>';
  document.getElementById('rear').innerHTML=rear||'<div class="card lbl">waiting for the motor list…</div>';
  document.getElementById('grid').innerHTML=grid;
  document.getElementById('texts').textContent=s.texts.join('\n');
}
function setMode(n){fetch('/mode?name='+n,{method:'POST'})}
function setHover(){const v=parseFloat(document.getElementById('hovdeg').value);const cur=document.getElementById('hovnow').textContent;
  if(!isFinite(v)||v<-10||v>60){alert('Enter a hover pitch between -10 and 60 deg');return}
  if(!confirm(`Set the hover pitch from ${cur}° to ${v}°?\n\nWrites PX4's level (SENS_BOARD_Y_OFF), the nose-lift target and every rotor's geometry, saves them and reboots the board. Disarmed only.`))return;
  fetch('/hover?deg='+v,{method:'POST'}).then(r=>{if(!r.ok)alert('Not started: disarm first, and wait for the link and any change in progress.')})}
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

// ---- tabs
function showTab(t){
  document.querySelectorAll('.tab').forEach(b=>b.classList.toggle('on',b.dataset.tab==t));
  document.getElementById('tab-live').hidden=t!='live';document.getElementById('tab-logs').hidden=t!='logs';
  try{localStorage.setItem('dash-tab',t)}catch(e){}
  if(t=='logs')refreshLogs(true);
}
document.querySelectorAll('.tab').forEach(b=>b.addEventListener('click',()=>showTab(b.dataset.tab)));
try{const t=localStorage.getItem('dash-tab');if(t)showTab(t)}catch(e){}

// ---- logs: one per flight, arm to disarm
const openLogs=new Set(),logView={};let logsKey='';
const dur=s=>s>=60?Math.floor(s/60)+' min '+(s%60).toFixed(0)+' s':s.toFixed(1)+' s';
const opct=u=>u===''?'':Math.max(0,Math.round((+u-1000)/10))+'%';
async function loadLog(name,el){
  const mode=logView[name]||'all';
  let d;try{d=await (await fetch('/logs/'+name)).json()}catch(e){return}
  const c=d.columns,ix=k=>c.indexOf(k),ev=ix('event'),tx=ix('text');
  // outputs with a motor/servo function; before the functions were known, those that ever carried a signal
  let outs=c.map((h,i)=>[h,i]).filter(([h])=>/^out\d+/.test(h));
  outs=outs.some(([h])=>h.includes(' '))?outs.filter(([h])=>h.includes(' ')):outs.filter(([h,i])=>d.rows.some(r=>+r[i]>0));
  let rows=d.rows;
  if(mode=='events')rows=rows.filter(r=>r[ev]);
  else if(mode=='1s'){let last=-1;rows=rows.filter(r=>{if(r[ev])return true;const s=Math.floor(+r[ix('since_arm_s')]);if(s==last)return false;last=s;return true})}
  const num=(v,d=1)=>v===''?'':(+v).toFixed(d);
  const cols=[['time','l'],['+s'],['uptime'],['mode','l'],['link'],['batt V'],['roll'],['pitch'],['yaw'],['nose lift','l'],['NL pitch'],['NL cmd'],...outs.map(([h])=>[h.replace(/^out\d+ /,'')||h]),['event','l'],['text','l']];
  const head='<tr>'+cols.map(([h,cl])=>`<th class="${cl||''}">${esc(h)}</th>`).join('')+'</tr>';
  const body=rows.map(r=>{
    const nl=r[ix('nl_state')]+(r[ix('nl_abort')]&&r[ix('nl_abort')]!='none'?' ('+r[ix('nl_abort')]+')':'');
    return `<tr class="${r[ev]?'ev':''}"><td class="l">${esc(r[0].slice(11))}</td><td>${num(r[ix('since_arm_s')],2)}</td><td>${num(r[ix('board_uptime_s')],2)}</td><td class="l">${esc(r[ix('mode')])}</td><td>${r[ix('link')]=='1'?'':'<span class="bad">lost</span>'}</td><td>${num(r[ix('battery_v')],2)}</td><td>${num(r[ix('px4_roll_deg')])}</td><td>${num(r[ix('px4_pitch_deg')])}</td><td>${num(r[ix('px4_yaw_deg')])}</td><td class="l">${esc(nl)}</td><td>${num(r[ix('nl_pitch_deg')])}</td><td>${r[ix('nl_cmd')]===''?'':Math.round(+r[ix('nl_cmd')]*100)+'%'}</td>${outs.map(([h,i])=>`<td>${opct(r[i])}</td>`).join('')}<td class="l">${esc(r[ev])}</td><td class="l txt">${esc(r[tx])}</td></tr>`}).join('');
  const bar=`<div class="logbar"><select data-view="${name}"><option value="all">every sample (10 per second)</option><option value="1s">one per second</option><option value="events">events only</option></select><a href="/logs/${name}.csv" download>Download CSV</a><span class="us">${d.rows.length} rows${d.recording?' · recording…':''} · pitch/roll/yaw: PX4's frame · outputs: % of 1000-2000 µs</span></div>`;
  const wrap=el.querySelector('.tblwrap'),top=wrap?wrap.scrollTop:0;
  el.innerHTML=bar+`<div class="tblwrap"><table>${head}${body}</table></div>`;
  el.querySelector('.tblwrap').scrollTop=top;
  const sel=el.querySelector('select');sel.value=mode;sel.addEventListener('change',()=>{logView[name]=sel.value;loadLog(name,el)});
}
let logNames=null;
async function refreshLogs(force){
  let logs;try{logs=await (await fetch('/logs')).json()}catch(e){return}
  document.getElementById('recdot').textContent=logs.some(l=>l.recording)?'●':'';
  if(document.getElementById('tab-logs').hidden)return;
  const key=JSON.stringify(logs);
  if(key==logsKey&&!force)return;
  logsKey=key;
  const box=document.getElementById('logs'),names=logs.map(l=>l.name).join();
  if(force||names!==logNames){  // a new flight (or the first look): rebuild the list
    logNames=names;
    box.innerHTML=logs.length?logs.map(l=>`<details class="run" data-name="${l.name}" ${openLogs.has(l.name)?'open':''}><summary><b>${l.start?new Date(l.start*1000).toLocaleString():l.name}</b><span class="dur"></span><span class="${l.recording?'bad':'ok'} st"></span><span class="us cnt"></span></summary><div class="body"></div></details>`).join(''):'<div class="lbl">no flight logs yet: arm to start one</div>';
    box.querySelectorAll('details').forEach(d=>{
      const body=d.querySelector('.body');
      if(d.open)loadLog(d.dataset.name,body);
      d.addEventListener('toggle',()=>{if(d.open){openLogs.add(d.dataset.name);loadLog(d.dataset.name,body)}else openLogs.delete(d.dataset.name)});
    });
  }
  logs.forEach(l=>{  // update the headers in place; reload an open log while it records
    const d=box.querySelector(`details[data-name="${l.name}"]`);if(!d)return;
    d.querySelector('.dur').textContent=l.start&&l.end?dur(l.end-l.start):'';
    d.querySelector('.st').textContent=l.recording?'recording…':'arm to disarm';
    d.querySelector('.cnt').textContent=`${l.samples} samples · ${l.events} events · ${(l.bytes/1024).toFixed(0)} kB`;
    if(d.open&&l.recording)loadLog(l.name,d.querySelector('.body'));
  });
}
setInterval(refreshLogs,2000);refreshLogs();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_POST(self):
        if self.path == "/readback":
            m = link.get("m")
            if m is None:
                self.send_response(400); self.end_headers(); return
            def go():
                try:
                    rb = readback(m)
                    hover_msg(f"board reports hover pitch {rb['hover']:g}°, nose-lift target {rb['nl_tgt']:g}°"
                              + ("" if rb["ok"] else " (they differ: set the hover pitch again)"), rb["ok"], False)
                except Exception as e:
                    hover_msg(f"read back failed: {e}", False, False)
            threading.Thread(target=go, daemon=True).start()
            self.send_response(204); self.end_headers(); return
        if self.path.startswith("/hover?"):
            try:
                deg = float(self.path.partition("deg=")[2])
            except ValueError:
                deg = None
            with lock:
                busy = state["hover"]["busy"]
            ok = deg is not None and -10.0 <= deg <= 60.0 and not busy and "m" in link
            if ok:
                hover_msg("starting…")
                threading.Thread(target=set_hover, args=(deg,), daemon=True).start()
            self.send_response(204 if ok else 400)
            self.end_headers()
            return
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
        if self.path == "/logs":
            with lock:
                return self.send_json(rec.log.summaries())
        if self.path.startswith("/logs/"):
            name = self.path[len("/logs/"):]
            want_csv = name.endswith(".csv")
            name = name[:-4] if want_csv else name
            if not name.replace("_", "").isalnum():
                return self.send_json({"error": "no such log"}, 404)
            with lock:
                got = rec.log.csv_bytes(name) if want_csv else rec.log.rows(name)
            if got is None:
                return self.send_json({"error": "no such log"}, 404)
            if not want_csv:
                return self.send_json(got)
            self.send_response(200)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Disposition", f'attachment; filename="{name}.csv"')
            self.send_header("Content-Length", str(len(got)))
            self.end_headers()
            self.wfile.write(got)
            return
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
