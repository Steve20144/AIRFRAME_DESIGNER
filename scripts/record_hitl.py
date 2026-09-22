"""Record a HITL pilot attempt: the board's MAVLink (proxied by the app to udp 14550) and the simulator's truth
(websocket /ws). One JSON line per record; --out results/hitl/<name>.jsonl. Stop with Ctrl-C or --seconds."""
import argparse, asyncio, json, sys, time, threading
from pymavlink import mavutil
import websockets

ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); ap.add_argument("--seconds", type=float, default=900)
ap.add_argument("--mav", default="udpin:127.0.0.1:14550"); ap.add_argument("--ws", default="ws://127.0.0.1:8080/ws")
a = ap.parse_args()
out = open(a.out, "w"); lock = threading.Lock(); t0 = time.time(); n = {"mav": 0, "sim": 0}
def emit(rec):
    rec["t"] = round(time.time() - t0, 3)
    with lock: out.write(json.dumps(rec) + "\n")

def mav_loop():
    m = mavutil.mavlink_connection(a.mav, dialect="common", source_system=254)
    keep = {"ATTITUDE": ["roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed"],
            "ATTITUDE_TARGET": ["q", "body_roll_rate", "body_pitch_rate", "body_yaw_rate", "thrust"],
            "RC_CHANNELS": [f"chan{i}_raw" for i in range(1, 9)],
            "HIL_ACTUATOR_CONTROLS": ["controls"], "HEARTBEAT": ["base_mode", "custom_mode"],
            "MANUAL_CONTROL": ["x", "y", "z", "r"], "LOCAL_POSITION_NED": ["x", "y", "z", "vx", "vy", "vz"],
            "STATUSTEXT": ["text", "severity"]}
    while time.time() - t0 < a.seconds:
        msg = m.recv_match(blocking=True, timeout=1.0)
        if msg is None: continue
        t = msg.get_type()
        if t in keep:
            d = {"src": "mav", "msg": t}
            for f in keep[t]:
                v = getattr(msg, f, None)
                d[f] = list(v) if isinstance(v, (list, tuple)) else v
            emit(d); n["mav"] += 1

async def sim_loop():
    async with websockets.connect(a.ws, max_size=None) as ws:
        last = 0.0
        while time.time() - t0 < a.seconds:
            raw = await ws.recv()
            try: d = json.loads(raw)
            except Exception: continue
            if d.get("type") == "state" and time.time() - last > 0.05:
                s = d["state"]; last = time.time()
                emit({"src": "sim", "pos": s.get("pos"), "vel": s.get("vel"), "euler": s.get("euler"), "rates": s.get("rates"),
                      "on_ground": s.get("on_ground"), "cmd": s.get("cmd"), "thrust": s.get("thrust"), "nose_lift": s.get("nose_lift")})
                n["sim"] += 1

threading.Thread(target=mav_loop, daemon=True).start()
try:
    asyncio.run(sim_loop())
except KeyboardInterrupt:
    pass
finally:
    out.close(); print("records", n, file=sys.stderr)
