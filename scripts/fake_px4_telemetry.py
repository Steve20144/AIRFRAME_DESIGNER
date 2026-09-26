"""A scripted fake Pixhawk for testing scripts/throttle_dashboard.py without the radio or the aircraft.

Start the dashboard on a UDP port (the `throttle-fake` entry in .claude/launch.json does this, port 8097, runs in
/tmp/throttle_fake_runs):
  python scripts/throttle_dashboard.py --port udpin:127.0.0.1:14660 --http 8097 --qgc-port 0 --runs-dir /tmp/runs
then fly:
  python scripts/fake_px4_telemetry.py [--armed 12]

The flight: 4 s disarmed, armed in
Stabilized for ``--armed`` seconds (nose lift parked -> raising -> holding, outputs 9/10 spinning, Altitude mode
8 s after arming), then disarmed
for 5 s. Sends what the dashboard reads: HEARTBEAT, ATTITUDE, SERVO_OUTPUT_RAW 0/1, DEBUG_VECT NLIFT, SYS_STATUS,
STATUSTEXT (one long text in 50-character chunks), PARAM_VALUE for the output functions and NL_MOT_MSK."""
import argparse
import os
os.environ.setdefault("MAVLINK20", "1")
import math
import struct
import time

from pymavlink import mavutil

p = argparse.ArgumentParser()
p.add_argument("--to", default="udpout:127.0.0.1:14660")
p.add_argument("--armed", type=float, default=12.0)
a = p.parse_args()
m = mavutil.mavlink_connection(a.to, source_system=1, source_component=1)
f = lambda i: struct.unpack("<f", struct.pack("<i", i))[0]
STAB = 7 << 16
t0 = time.time()
boot0 = 123_000                     # the board has been up 123 s
t_arm, t_disarm, t_end = 4.0, 4.0 + a.armed, 4.0 + a.armed + 5.0
last = {"hb": -9, "par": -9, "att": -9, "srv": -9, "nl": -9, "sys": -9}
said = set()
while True:
    t = time.time() - t0
    if t > t_end:
        break
    armed = t_arm <= t < t_disarm
    ta = t - t_arm
    if t - last["hb"] >= 1:
        last["hb"] = t
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_AUTOPILOT_PX4,
                             (128 if armed else 0) | 1, (2 << 16) if armed and ta > 8 else STAB, 4 if armed else 3)
    if t - last["par"] >= 2:
        last["par"] = t
        for name, val in [("PWM_AUX_FUNC1", 109), ("PWM_AUX_FUNC2", 110), ("PWM_MAIN_FUNC1", 101), ("PWM_MAIN_FUNC2", 102),
                          ("NL_MOT_MSK", (1 << 8) | (1 << 9))]:
            m.mav.param_value_send(name.encode(), f(val), mavutil.mavlink.MAV_PARAM_TYPE_INT32, 20, 0)
    # nose lift: parked, raising 3 deg/s from +2 to 24 after 1 s armed, holding
    if not armed:
        st, pitch, cmd = 1, 2.0, 0.0
    elif ta < 1:
        st, pitch, cmd = 2, 2.0, 0.0
    elif 2.0 + 3 * (ta - 1) < 24:
        st, pitch, cmd = 3, 2.0 + 3 * (ta - 1), 0.85 + 0.1 * math.sin(6 * ta)
    else:
        st, pitch, cmd = 4, 24.0, 0.62
    if t - last["att"] >= 0.05:
        last["att"] = t
        m.mav.attitude_send(boot0 + int(t * 1000), 0.01 * math.sin(t), math.radians(pitch - 24), 0.3, 0, 0, 0)
    if t - last["srv"] >= 0.1:
        last["srv"] = t
        pw = 1000 + int(1000 * cmd) if armed else 900
        m.mav.servo_output_raw_send(int(t * 1e6), 0, *([1000 if armed else 900] * 8))
        m.mav.servo_output_raw_send(int(t * 1e6), 1, pw, int(pw * 0.9), *([0] * 6))
    if t - last["nl"] >= 0.1:
        last["nl"] = t
        m.mav.debug_vect_send(b"NLIFT", int(t * 1e6), st + 0.0, pitch, cmd)
    if t - last["sys"] >= 0.5:
        last["sys"] = t
        m.mav.sys_status_send(0, 0, 0, 500, int(24800 - 30 * max(0, ta) * armed), -1, -1, 0, 0, 0, 0, 0, 0)
    if armed and ta > 2 and "long" not in said:
        said.add("long")
        text = "Nose lift: raising the nose on motors 9 and 10 at 3 deg/s, target 24 deg, hover handover armed"
        for k in range(0, len(text), 50):
            m.mav.statustext_send(6, text[k:k + 50].encode(), 7, k // 50)
    if armed and ta > 6 and "short" not in said:
        said.add("short")
        m.mav.statustext_send(4, b"Accel 0 clipping, not safe to fly!", 0, 0)
    time.sleep(0.01)
print("fake flight done")
