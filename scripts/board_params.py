"""Back up, compare and set the flight controller's parameters over USB (Windows: the board's COM port).

  python scripts/board_params.py backup --port COM3                 all parameters -> results/board_params/<time>.json
  python scripts/board_params.py diff   --port COM3 --against F.json what differs from a backup (e.g. after a flash)
  python scripts/board_params.py get    --port COM3 NL_KQ NL_K_ANG     read only
  python scripts/board_params.py set    --port COM3 NL_KQ=0.03 NL_K_ANG=0.4   each set is read back and checked

Over the telemetry radio while scripts/throttle_dashboard.py runs: --port udpin:127.0.0.1:14550 (the dashboard relays a
ground station there, as it does for QGC; slower, ~2 KB/s).

Needs pymavlink and pyserial. Close QGC first (it holds the port). The telemetry dashboard on the radio can stay.
"""
import argparse
import json
import os
import struct
import sys
import time
from pathlib import Path

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "results" / "board_params"
INT_TYPES = {mavutil.mavlink.MAV_PARAM_TYPE_INT8, mavutil.mavlink.MAV_PARAM_TYPE_INT16, mavutil.mavlink.MAV_PARAM_TYPE_INT32,
             mavutil.mavlink.MAV_PARAM_TYPE_UINT8, mavutil.mavlink.MAV_PARAM_TYPE_UINT16, mavutil.mavlink.MAV_PARAM_TYPE_UINT32}


def value(msg):
    if msg.param_type in INT_TYPES:        # PX4 sends integers bit-cast into the float field
        return struct.unpack("<i", struct.pack("<f", msg.param_value))[0]
    return round(float(msg.param_value), 7)


def connect(port):
    m = mavutil.mavlink_connection(port, baud=115200, source_system=254)
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        sys.exit(f"no heartbeat on {port}")
    print(f"{port}: system {m.target_system}, autopilot {hb.autopilot}")
    return m


def read_all(m, timeout=60):
    params, types, count = {}, {}, None
    m.mav.param_request_list_send(m.target_system, m.target_component)
    t_end = time.time() + timeout
    last = time.time()
    while time.time() < t_end:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is not None:
            params[msg.param_id], types[msg.param_id] = value(msg), msg.param_type
            count = msg.param_count
            last = time.time()
        if count and len(params) >= count:
            break
        if count and time.time() - last > 2:   # ask again for what is missing, one by one
            for i in range(count):
                m.mav.param_request_read_send(m.target_system, m.target_component, b"", i)
            last = time.time()
    if count is None or len(params) < count:
        print(f"warning: {len(params)} of {count} parameters")
    return params, types


def cmd_backup(a):
    m = connect(a.port)
    params, types = read_all(m)
    OUT.mkdir(parents=True, exist_ok=True)
    f = OUT / f"params_{time.strftime('%Y%m%d_%H%M%S')}{('_' + a.tag) if a.tag else ''}.json"
    f.write_text(json.dumps({"port": a.port, "time": time.time(), "count": len(params), "params": params,
                             "types": types}, indent=1, sort_keys=True))
    print(f"{len(params)} parameters -> {f}")


def cmd_diff(a):
    old = json.loads(Path(a.against).read_text())["params"]
    m = connect(a.port)
    new, _ = read_all(m)
    changed = {k: (old[k], new[k]) for k in old if k in new and old[k] != new[k]}
    gone = sorted(set(old) - set(new))
    added = sorted(set(new) - set(old))
    print(f"{len(new)} now, {len(old)} in the backup; changed {len(changed)}, removed {len(gone)}, new {len(added)}")
    for k, (o, n) in sorted(changed.items()):
        print(f"  {k}: {o} -> {n}")
    if gone:
        print("  removed:", ", ".join(gone))
    if added:
        print("  new:", ", ".join(f"{k}={new[k]}" for k in added))


def read_one(m, k, timeout=3.0, tries=3):
    for _ in range(tries):
        m.mav.param_request_read_send(m.target_system, m.target_component, k.encode(), -1)
        t_end = time.time() + timeout
        while time.time() < t_end:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is not None and msg.param_id == k:
                return msg
    return None


def cmd_get(a):
    m = connect(a.port)
    for k in a.names:
        msg = read_one(m, k)
        print(f"  {k}: {value(msg) if msg is not None else 'not on this firmware (or no reply)'}")


def cmd_set(a):
    want = {}
    for it in a.values:
        k, v = it.split("=", 1)
        want[k.strip()] = float(v)
    m = connect(a.port)
    ok = True
    for k, v in want.items():
        m.mav.param_request_read_send(m.target_system, m.target_component, k.encode(), -1)
        cur = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
        while cur is not None and cur.param_id != k:
            cur = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
        if cur is None:
            print(f"  {k}: unknown to this firmware"); ok = False; continue
        before = value(cur)
        wire = struct.unpack("<f", struct.pack("<i", int(v)))[0] if cur.param_type in INT_TYPES else float(v)
        for _ in range(3):
            m.mav.param_set_send(m.target_system, m.target_component, k.encode(), wire, cur.param_type)
            back = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
            while back is not None and back.param_id != k:
                back = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
            if back is not None and abs(value(back) - v) < 1e-5:
                break
        good = back is not None and abs(value(back) - v) < 1e-5
        ok &= good
        print(f"  {k}: {before} -> {value(back) if back is not None else '?'} {'ok' if good else 'NOT SET'}")
    sys.exit(0 if ok else 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("backup"); p.add_argument("--port", required=True); p.add_argument("--tag", default="")
    p = sub.add_parser("diff"); p.add_argument("--port", required=True); p.add_argument("--against", required=True)
    p = sub.add_parser("get"); p.add_argument("--port", required=True); p.add_argument("names", nargs="+")
    p = sub.add_parser("set"); p.add_argument("--port", required=True); p.add_argument("values", nargs="+")
    a = ap.parse_args()
    {"backup": cmd_backup, "diff": cmd_diff, "get": cmd_get, "set": cmd_set}[a.cmd](a)


if __name__ == "__main__":
    main()
