"""The aircraft's nose-lift fan bursts in SITL, and candidate fixes.

On the aircraft (2026-09-23, dashboard logs log_20260923_1639*.csv, video IMG_1508) the firmware nose lift slams
fans 9 and 10 between idle and full at about 4 Hz: the frame rocks on its legs at 5-10 Hz (+-20 deg/s of pitch
rate, log 184), the loop's rate gain (NL_KQ 0.10 on the board) turns that into full-range commands, and the slow
fans turn every cut into a lag and every restart into a surge.

  python scripts/nose_lift_smoothing.py --what repro     # the committed model vs the aircraft-like one
  python scripts/nose_lift_smoothing.py --what fixes     # candidate firmware settings on the aircraft-like one

Each flight is fw_nose_lift_takeoff or fw_nose_lift_cancel (needs the ~/PX4-nl build), park +2 deg.
Results and time series go to results/nose_lift_smoothing/<what>/.
"""
import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from airframe_designer.batch.runner import run_many            # noqa: E402
from airframe_designer.geometry.airframe import Airframe        # noqa: E402
from airframe_designer.sim.nose_lift import firmware_params     # noqa: E402
from airframe_designer.sim.scenario import load_scenario        # noqa: E402

AIRFRAME = ROOT / "airframes" / "atlas_09b.json"
PARK, HOVER = 2.0, 24.0          # park overridable with --park
G4 = {"NL_KQ": 0.10, "NL_KQI": 0.05, "NL_LOW_KQ": 0.06, "NL_LOW_KQI": 0.03}   # on the board since 2026-09-23
FANS = {"rotors[8,9].tau": 0.4, "rotors[8,9].tau_down": 1.0}                   # the nose fans' slow response
ROCKING = [0.0, 0.35, 0.0]                                                     # rad/s of pitch-rate rocking at full fan speed


def scenario(name):
    sc = json.loads((ROOT / "scenarios" / f"fw_nose_lift_{name}.json").read_text())
    sc["attitude"] = {"park_pitch_deg": PARK, "hover_pitch_deg": HOVER}
    for ph in sc["phases"]:
        u = ph.get("until") or {}
        if "pitch_deg" in u and ph.get("name") == "at_park":   # other phases' angles (a cancel at 15 deg) stay
            u["pitch_deg"] = PARK
    return sc


def underrated_a(sc, factor=0.85):
    """NL_A0/NL_A1 scaled by ``factor``: below 1 the firmware's balance feed-forward asks for more thrust than the
    fans need (an earlier session reproduced the aircraft with 0.85)."""
    af = load_scenario(sc).apply_attitude(Airframe.load(AIRFRAME))
    p = firmware_params(af)
    return {k: round(float(p[k]) * factor, 6) for k in ("NL_A0", "NL_A1") if k in p}


def conditions(what):
    if what == "sweep":
        # the rate gain on the shaking gyro, its low-pass, and how early the rise slows down (NL_K_ANG: q_des =
        # K_ANG x (target - pitch), capped at NL_RATE); the integral and lowering gains keep G4's ratios
        out = []
        for lpf in (0.0, 2.0, 3.5):
            for kq in (0.03, 0.06, 0.10):
                for k_ang in (1.0, 0.4):
                    p = {"NL_KQ": kq, "NL_KQI": kq / 2, "NL_LOW_KQ": 0.6 * kq, "NL_LOW_KQI": 0.3 * kq,
                         "NL_Q_LPF": lpf, "NL_K_ANG": k_ang}
                    out.append((f"LPF {lpf:g} KQ {kq:g} K_ANG {k_ang:g}", p, dict(FANS), ROCKING))
        return out
    if what == "ceiling":
        # balance at 12 deg, never above 14, and a mid-rise cancel, on the board's firmware (no hold fix); every
        # candidate keeps G4's stronger lowering gains, and flies both fan-estimate variants (the firmware's NL_A
        # right, or 15 % low as fitted to flight 2)
        g4low = {"NL_LOW_KQ": 0.06, "NL_LOW_KQI": 0.03}
        cands = [("G4", dict(G4) | {"NL_K_ANG": 1.0, "NL_RATE": 3.0}),
                 ("KQ .03 K .4 R 3", {"NL_KQ": 0.03, "NL_KQI": 0.015, **g4low, "NL_K_ANG": 0.4, "NL_RATE": 3.0}),
                 ("KQ .03 K .4 R 1.5", {"NL_KQ": 0.03, "NL_KQI": 0.015, **g4low, "NL_K_ANG": 0.4, "NL_RATE": 1.5}),
                 ("KQ .04 K .3 R 1.5", {"NL_KQ": 0.04, "NL_KQI": 0.02, **g4low, "NL_K_ANG": 0.3, "NL_RATE": 1.5}),
                 ("KQ .05 K .5 R 2", {"NL_KQ": 0.05, "NL_KQI": 0.025, **g4low, "NL_K_ANG": 0.5, "NL_RATE": 2.0}),
                 ("KQ .05 K .5 R 1.5", {"NL_KQ": 0.05, "NL_KQI": 0.025, **g4low, "NL_K_ANG": 0.5, "NL_RATE": 1.5})]
        return [(f"{lab} | A x{u:g}", {**p, "_UNDERRATE": u}, dict(FANS), ROCKING) for lab, p in cands for u in (0.85, 1.0)]
    if what == "hover24":
        # NL_TGT 24 (the hover pitch) on the NL_CEIL firmware, balance and a cancel at 15 deg on the way up. The first
        # three were flown toward 24 on the aircraft and check the model: the HITL set (22 Sep: 15 deg/s, coasted to
        # 43), the 17:49 set (cancel at 14-15.5 carried on to 41-48 deg) and G4 (cancels peaked 21-29, bursting)
        g4low = {"NL_LOW_KQ": 0.06, "NL_LOW_KQI": 0.03}
        cands = [("V1 HITL 22 Sep", {"NL_KQ": 0.02, "NL_KQI": 0.012, "NL_LOW_KQ": 0.3, "NL_LOW_KQI": 0.1,
                                     "NL_K_ANG": 1.0, "NL_RATE": 3.0, "NL_CEIL": 0.0}),
                 ("V2 aircraft 17:49", {"NL_KQ": 0.03, "NL_KQI": 0.015, "NL_LOW_KQ": 0.018, "NL_LOW_KQI": 0.009,
                                        "NL_K_ANG": 0.4, "NL_RATE": 3.0, "NL_CEIL": 0.0}),
                 ("V3 G4", dict(G4) | {"NL_K_ANG": 1.0, "NL_RATE": 3.0, "NL_CEIL": 0.0}),
                 ("C1 board now", {"NL_KQ": 0.03, "NL_KQI": 0.015, **g4low, "NL_K_ANG": 0.4, "NL_RATE": 3.0, "NL_CEIL": 1.0}),
                 ("C2 board now ceil 2", {"NL_KQ": 0.03, "NL_KQI": 0.015, **g4low, "NL_K_ANG": 0.4, "NL_RATE": 3.0, "NL_CEIL": 2.0}),
                 ("C3 G4 ceil 1", dict(G4) | {"NL_K_ANG": 1.0, "NL_RATE": 3.0, "NL_CEIL": 1.0}),
                 ("C4 KQ .03 K .4 R 1.5", {"NL_KQ": 0.03, "NL_KQI": 0.015, **g4low, "NL_K_ANG": 0.4, "NL_RATE": 1.5, "NL_CEIL": 1.0}),
                 ("C5 KQ .04 K .3 R 1.5", {"NL_KQ": 0.04, "NL_KQI": 0.02, **g4low, "NL_K_ANG": 0.3, "NL_RATE": 1.5, "NL_CEIL": 1.0}),
                 ("C6 KQ .05 K .5 R 2", {"NL_KQ": 0.05, "NL_KQI": 0.025, **g4low, "NL_K_ANG": 0.5, "NL_RATE": 2.0, "NL_CEIL": 1.0}),
                 ("C7 KQ .03 LOW .10", {"NL_KQ": 0.03, "NL_KQI": 0.015, "NL_LOW_KQ": 0.10, "NL_LOW_KQI": 0.05,
                                        "NL_K_ANG": 0.4, "NL_RATE": 3.0, "NL_CEIL": 1.0})]
        return [(f"{lab} | A x{u:g}", {**p, "_UNDERRATE": u}, dict(FANS), ROCKING) for lab, p in cands for u in (0.85, 1.0)]
    if what == "fit24":
        # the fan model that reproduces the aircraft's 24-deg runs (23 Sep, park ~5, pre-NL_CEIL firmware, so NL_CEIL
        # 0): G4 rose 8->14 deg at 4.2-5.7 deg/s and peaked 27-29 on the way to 24, a cancel at 17-18 peaked 20-23;
        # the 17:49 set rose at 7-8 deg/s and a cancel at 11-14 carried on to 41-48 (onto the tail). The hover24 model
        # (quadratic fans, tau_down 1 s) peaks 25-26 and 16-23. Candidates: a flatter thrust curve than the
        # firmware's NL_EXPO 2 (more thrust at part command, so cutting the command brakes less) and a slower spin-down
        v2 = {"NL_KQ": 0.03, "NL_KQI": 0.015, "NL_LOW_KQ": 0.018, "NL_LOW_KQI": 0.009, "NL_K_ANG": 0.4}
        out = []
        for lab, gains in (("V2 17:49", v2), ("V3 G4", dict(G4) | {"NL_K_ANG": 1.0})):
            for expo in (2.0, 1.5, 1.2):
                for down in (1.0, 2.0):
                    for under in (1.0, 0.85):
                        p = {**gains, "NL_RATE": 3.0, "NL_CEIL": 0.0, "NL_EXPO": 2.0, "_UNDERRATE": under}
                        fans = {"rotors[8,9].tau": FANS["rotors[8,9].tau"], "rotors[8,9].tau_down": down,
                                "rotors[8,9].thrust_exponent": expo}
                        out.append((f"{lab} | expo {expo:g} down {down:g} A x{under:g}", p, fans, ROCKING))
        return out
    if what == "robust24":
        # gains for NL_TGT 24 that survive the whole bracket fit24 found: fans spinning down in 1.0 s (the hover24
        # model: too optimistic, the 17:49 cancel stops at 16-22 instead of 41-49) to 2.0 s (too pessimistic: G4
        # tips on its way up, which the aircraft never did), with 1.5 s between, and NL_A right or 15 % low
        g4low = {"NL_LOW_KQ": 0.06, "NL_LOW_KQI": 0.03}
        strong = {"NL_LOW_KQ": 0.10, "NL_LOW_KQI": 0.05}
        cands = [("V2 17:49", {"NL_KQ": 0.03, "NL_KQI": 0.015, "NL_LOW_KQ": 0.018, "NL_LOW_KQI": 0.009,
                               "NL_K_ANG": 0.4, "NL_RATE": 3.0, "NL_CEIL": 0.0}),
                 ("V3 G4", dict(G4) | {"NL_K_ANG": 1.0, "NL_RATE": 3.0, "NL_CEIL": 0.0}),
                 ("C1 board now", {"NL_KQ": 0.03, "NL_KQI": 0.015, **g4low, "NL_K_ANG": 0.4, "NL_RATE": 3.0, "NL_CEIL": 1.0}),
                 ("C3 G4 ceil 1", dict(G4) | {"NL_K_ANG": 1.0, "NL_RATE": 3.0, "NL_CEIL": 1.0}),
                 ("C4 KQ .03 R 1.5", {"NL_KQ": 0.03, "NL_KQI": 0.015, **g4low, "NL_K_ANG": 0.4, "NL_RATE": 1.5, "NL_CEIL": 1.0}),
                 ("C5 KQ .04 K .3 R 1.5", {"NL_KQ": 0.04, "NL_KQI": 0.02, **g4low, "NL_K_ANG": 0.3, "NL_RATE": 1.5, "NL_CEIL": 1.0}),
                 ("C6 KQ .05 K .5 R 2", {"NL_KQ": 0.05, "NL_KQI": 0.025, **g4low, "NL_K_ANG": 0.5, "NL_RATE": 2.0, "NL_CEIL": 1.0}),
                 ("C7 KQ .03 LOW .10", {"NL_KQ": 0.03, "NL_KQI": 0.015, **strong, "NL_K_ANG": 0.4, "NL_RATE": 3.0, "NL_CEIL": 1.0}),
                 ("C8 KQ .03 R 1.5 LOW .10", {"NL_KQ": 0.03, "NL_KQI": 0.015, **strong, "NL_K_ANG": 0.4, "NL_RATE": 1.5, "NL_CEIL": 1.0}),
                 ("C9 KQ .05 R 1 LOW .10 ceil .5", {"NL_KQ": 0.05, "NL_KQI": 0.025, **strong, "NL_K_ANG": 0.5, "NL_RATE": 1.0,
                                                    "NL_CEIL": 0.5})]
        out = []
        for lab, p in cands:
            for down in (1.0, 1.5, 2.0):
                for under in (1.0, 0.85):
                    fans = {"rotors[8,9].tau": FANS["rotors[8,9].tau"], "rotors[8,9].tau_down": down}
                    out.append((f"{lab} | down {down:g} A x{under:g}", {**p, "_UNDERRATE": under}, fans, ROCKING))
        return out
    if what == "confirm24":
        # the robust24 survivors over more seeds, with the board's set for reference
        keep = ("C1 ", "C5 ", "C6 ")
        return [c for c in conditions("robust24") if c[0].startswith(keep)]
    if what == "ceiling4":
        # the smoothest raise with the ceiling triggering earlier so the fans' spin-down coast ends below 14 deg
        cands = []
        for ceil in (1.0, 1.5):
            cands.append((f"KQ .03 K .4 R 3 ceil {ceil:g}", {"NL_KQ": 0.03, "NL_KQI": 0.015, "NL_LOW_KQ": 0.06, "NL_LOW_KQI": 0.03,
                                                              "NL_K_ANG": 0.4, "NL_RATE": 3.0, "NL_CEIL": ceil}))
        return [(f"{lab} | A x{u:g}", {**p, "_UNDERRATE": u}, dict(FANS), ROCKING) for lab, p in cands for u in (0.85, 1.0)]
    if what == "ceiling3":
        # the firmware ceiling (NL_CEIL 2: above 14 deg the lowering brings it back to 12), board source + hold fix
        g4low = {"NL_LOW_KQ": 0.06, "NL_LOW_KQI": 0.03, "NL_CEIL": 2.0}
        cands = [("G4 ceil 2", dict(G4) | {"NL_K_ANG": 1.0, "NL_RATE": 3.0, "NL_CEIL": 2.0}),
                 ("KQ .03 K .4 R 3 ceil 2", {"NL_KQ": 0.03, "NL_KQI": 0.015, **g4low, "NL_K_ANG": 0.4, "NL_RATE": 3.0}),
                 ("KQ .03 K .4 R 1.5 ceil 2", {"NL_KQ": 0.03, "NL_KQI": 0.015, **g4low, "NL_K_ANG": 0.4, "NL_RATE": 1.5}),
                 ("KQ .04 K .3 R 1.5 ceil 2", {"NL_KQ": 0.04, "NL_KQI": 0.02, **g4low, "NL_K_ANG": 0.3, "NL_RATE": 1.5})]
        return [(f"{lab} | A x{u:g}", {**p, "_UNDERRATE": u}, dict(FANS), ROCKING) for lab, p in cands for u in (0.85, 1.0)]
    if what == "ceiling2":
        # the smooth gain sets with the lifting thrust capped just above what the lift-off needs (NL_MAX_CMD is a
        # thrust fraction): less surplus to overshoot with once the nose breaks free
        g4low = {"NL_LOW_KQ": 0.06, "NL_LOW_KQI": 0.03}
        cands = []
        for cap in (0.90, 0.92):
            cands += [(f"KQ .03 K .4 R 3 cap {cap:g}", {"NL_KQ": 0.03, "NL_KQI": 0.015, **g4low, "NL_K_ANG": 0.4, "NL_RATE": 3.0, "NL_MAX_CMD": cap}),
                      (f"KQ .04 K .3 R 1.5 cap {cap:g}", {"NL_KQ": 0.04, "NL_KQI": 0.02, **g4low, "NL_K_ANG": 0.3, "NL_RATE": 1.5, "NL_MAX_CMD": cap})]
        return [(f"{lab} | A x{u:g}", {**p, "_UNDERRATE": u}, dict(FANS), ROCKING) for lab, p in cands for u in (0.85, 1.0)]
    if what == "balance":
        # the nose held at 12 deg on the two nose fans only (scenarios/fw_nose_lift_balance.json)
        new = {"NL_KQ": 0.03, "NL_KQI": 0.015, "NL_LOW_KQ": 0.018, "NL_LOW_KQI": 0.009, "NL_K_ANG": 0.4}
        return [("committed model, G4", dict(G4), {}, None), ("committed model, KQ 0.03", new, {}, None),
                ("aircraft-like, G4", dict(G4), dict(FANS), ROCKING), ("aircraft-like, KQ 0.03", new, dict(FANS), ROCKING)]
    if what == "confirm":
        # the candidates over seeds, with the firmware's settled-rate hold check (so a lift can reach Holding)
        return [("G4 (board today)", dict(G4), dict(FANS), ROCKING),
                ("KQ 0.03 K_ANG 0.4", {"NL_KQ": 0.03, "NL_KQI": 0.015, "NL_LOW_KQ": 0.018, "NL_LOW_KQI": 0.009,
                                       "NL_K_ANG": 0.4}, dict(FANS), ROCKING),
                ("KQ 0.02 K_ANG 0.25", {"NL_KQ": 0.02, "NL_KQI": 0.01, "NL_LOW_KQ": 0.012, "NL_LOW_KQI": 0.006,
                                        "NL_K_ANG": 0.25}, dict(FANS), ROCKING)]
    if what == "sweep2":
        # around the park-2 winner (KQ 0.03, K_ANG 0.4, no filter), on the model fitted to flight 2
        out = [("G4 (board today)", dict(G4), dict(FANS), ROCKING)]
        for kq in (0.02, 0.03, 0.04, 0.06):
            for k_ang in (0.25, 0.4, 0.6):
                p = {"NL_KQ": kq, "NL_KQI": kq / 2, "NL_LOW_KQ": 0.6 * kq, "NL_LOW_KQI": 0.3 * kq, "NL_K_ANG": k_ang}
                out.append((f"KQ {kq:g} K_ANG {k_ang:g}", p, dict(FANS), ROCKING))
        return out
    if what == "fit2":
        # the size of the legs' rocking that reproduces flight 2 (the aircraft lifts through its bursts)
        out = []
        for amp in (0.05, 0.1, 0.2):
            for up in (0.25, 0.4):
                out.append((f"rocking {amp:g} tau {up:g}/1", {**G4, "_UNDERRATE": 1.0},
                            {"rotors[8,9].tau": up, "rotors[8,9].tau_down": 1.0}, [0.0, amp, 0.0]))
        return out
    if what == "fit":
        # the fan response and the firmware's thrust estimate that reproduce the aircraft's flight 2 (park 5.3: rose
        # about 3 deg/s while bursting ~7 flips/s); _UNDERRATE scales NL_A0/1 (1.0: the firmware knows the fans)
        out = []
        for up in (0.15, 0.25, 0.4):
            for down in (0.5, 1.0):
                for under in (1.0, 0.85):
                    out.append((f"tau {up:g}/{down:g} A x{under:g}", {**G4, "_UNDERRATE": under},
                                {"rotors[8,9].tau": up, "rotors[8,9].tau_down": down}, ROCKING))
        return out
    if what == "repro":
        return [("C0 committed model, G4", dict(G4), {}, None),
                ("C1 aircraft-like, G4", dict(G4), dict(FANS), ROCKING)]
    base = ("aircraft-like", dict(FANS), ROCKING)
    return [("G4 (board today)", dict(G4), *base[1:]),
            ("G4 + rate LPF 5 Hz", {**G4, "NL_Q_LPF": 5.0}, *base[1:]),
            ("G4 + slew 2/s + band 0.25", {**G4, "NL_SLEW": 2.0, "NL_FB_BAND": 0.25}, *base[1:]),
            ("G4 + LPF 5 + slew 2 + band 0.25", {**G4, "NL_Q_LPF": 5.0, "NL_SLEW": 2.0, "NL_FB_BAND": 0.25}, *base[1:])]


def analyse(ts, n_rotors):
    """Per phase of interest: the nose fans' command (cmd_mean x rotors / 2: only the two nose fans run on the
    ground), how often it flips between off (<0.15) and full (>0.9), its step-to-step activity, the pitch rate."""
    ts = ts.get("timeseries", ts)          # run_once writes {"result", "timeseries"}
    c = {k: i for i, k in enumerate(ts["columns"])}
    rows = np.array(ts["rows"], float)
    ph = np.array(ts["phase"])
    out = {}
    for name in ("lift", "hold", "lift_1", "held", "raise", "balance", "switch_off", "lowered", "cut"):
        m = ph == name
        if m.sum() < 5:
            continue
        x = rows[m]
        cmd = np.clip(x[:, c["cmd_mean"]] * n_rotors / 2.0, 0, 1)
        t = x[:, c["t"]]
        lvl = np.where(cmd > 0.9, 1, np.where(cmd < 0.15, -1, 0))
        seen = lvl[lvl != 0]
        flips = int((np.diff(seen) != 0).sum()) if len(seen) > 1 else 0
        dur = float(t[-1] - t[0]) or 1.0
        out[name] = {"s": round(dur, 1), "flips_per_s": round(flips / dur, 2),
                     "off_or_full": round(float((lvl != 0).mean()), 2),
                     "cmd_step_mean": round(float(np.abs(np.diff(cmd)).mean()), 3),
                     "q_max": round(float(np.degrees(np.abs(x[:, c["q"]])).max()), 1),
                     "pitch_end": round(float(np.degrees(x[-1, c["pitch"]])) + HOVER, 1)}   # the nose angle (ts: hover frame)
        # what the nose did (the phase can end after a timeout lowered it again, so the end angle says little):
        # its peak, when it first reached 20 deg and how fast it got there, and the time near the target
        nose = np.degrees(x[:, c["pitch"]]) + HOVER
        d = out[name]
        d["nose_max"] = round(float(nose.max()), 1)
        d["nose_mean"], d["nose_std"], d["nose_min"] = (round(float(nose.mean()), 2), round(float(nose.std()), 2),
                                                        round(float(nose.min()), 1))
        hit = np.flatnonzero(nose >= 20.0)
        if hit.size:                       # the aircraft's flight 2 rose 5.3 -> 19.4 deg in 4.5 s
            d["t_to_20"] = round(float(t[hit[0]] - t[0]), 1)
            d["rise_deg_s"] = round(float((nose[hit[0]] - nose[0]) / max(t[hit[0]] - t[0], 1e-3)), 2)
            d["near_target_s"] = round(float((np.abs(nose[hit[0]:] - HOVER) < 2.0).sum() * np.median(np.diff(t))), 1)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--what", choices=["repro", "fixes", "sweep", "sweep2", "fit", "fit2", "confirm", "balance", "ceiling", "ceiling2", "ceiling3", "ceiling4", "hover24", "fit24", "robust24", "confirm24"], default="repro")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--instances", default="5,6,7,8")
    ap.add_argument("--px4-dir", default=os.path.expanduser("~/PX4-nl"))
    ap.add_argument("--analyse", action="store_true", help="only re-read the time series of flights already flown")
    ap.add_argument("--park", type=float, default=PARK, help="parked pitch, deg (the aircraft: 1.9 and 5.3 on 2026-09-23)")
    ap.add_argument("--tag", default="", help="suffix for the output folder")
    ap.add_argument("--rocking", type=float, default=None, help="pitch-rate rocking, rad/s at full fan speed (default 0.35)")
    ap.add_argument("--fans", default="", help="nose fans' tau,tau_down in s (default 0.4,1.0)")
    ap.add_argument("--seeds", default="1", help="comma-separated seeds; every condition flies each")
    a = ap.parse_args()
    ap_rock = a.rocking
    globals()["PARK"] = a.park
    if ap_rock is not None:                # the fitted rocking for fixes / sweep
        globals()["ROCKING"] = [0.0, ap_rock, 0.0]
    if a.fans:
        up, down = (float(v) for v in a.fans.split(","))
        FANS.update({"rotors[8,9].tau": up, "rotors[8,9].tau_down": down})
    out = ROOT / "results" / "nose_lift_smoothing" / (a.what + (f"_{a.tag}" if a.tag else ""))
    out.mkdir(parents=True, exist_ok=True)
    tasks = []
    for label, params, variables, rocking in conditions(a.what):
        for seed in (int(v) for v in a.seeds.split(",")):
            for sname in (("balance24", "midcancel24") if a.what in ("hover24", "fit24", "robust24", "confirm24") else ("cancel",) if a.what.startswith("fit") else ("balance",) if a.what == "balance" else ("balance12", "midcancel12") if a.what.startswith("ceiling") else ("takeoff", "cancel")):
                sc = scenario(sname)
                p = dict(params)
                under = p.pop("_UNDERRATE", 0.85 if variables else None)
                if under is not None and under != 1.0:   # aircraft-like: the firmware also misjudges the fans
                    p.update(underrated_a(sc, under))
                tid = f"{label} | {sname}" + (f" | s{seed}" if a.seeds != "1" else "")
                slug = "".join(ch if ch.isalnum() else "_" for ch in tid)
                opts = {"extra_params": p, "timeseries_path": str(out / f"{slug}_ts.json"), "seed": seed}
                if rocking:
                    opts["vibration"] = rocking
                tasks.append({"id": tid, "airframe": str(AIRFRAME), "scenario": sc, "variables": variables, "options": opts})
    if a.analyse:                          # re-read the flights already flown
        rs = [json.loads(Path(t["options"]["timeseries_path"]).read_text())["result"] for t in tasks]
    else:
        rs = run_many(tasks, workers=a.workers, instances=[int(i) for i in a.instances.split(",")], px4_dir=a.px4_dir)
    n = len(Airframe.load(AIRFRAME).active_rotors())
    table = []
    for t, r in zip(tasks, rs):
        ts_path = Path(t["options"]["timeseries_path"])
        res = analyse(json.loads(ts_path.read_text()), n) if ts_path.exists() else {}
        table.append({"id": t["id"], "ok": r.get("ok"), "status": r.get("status"), "failures": r.get("failures"),
                      "events": [e["text"] for e in (r.get("metrics") or {}).get("events", [])][:8], "phases": res})
    (out / "summary.json").write_text(json.dumps(table, indent=1))
    for row in table:
        print(f"\n{row['id']}: {'ok' if row['ok'] else 'FAILED ' + str(row['status'])} {'; '.join(row['failures'] or [])[:120]}")
        for name, d in row["phases"].items():
            print(f"   {name:9s} {d['s']:5.1f} s  flips {d['flips_per_s']:5.2f}/s  off-or-full {d['off_or_full']:4.2f}  "
                  f"|q| max {d['q_max']:5.1f}  nose max {d['nose_max']:5.1f}  to 20 deg {d.get('t_to_20', '-')} s "
                  f"({d.get('rise_deg_s', '-')} deg/s)  near target {d.get('near_target_s', '-')} s")


if __name__ == "__main__":
    main()
