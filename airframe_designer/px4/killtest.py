"""Kill-switch test recorder (Controller tab): proof that the emergency stop cuts every motor, in whatever phase.

While running it samples, at a few hundred Hz, what the flight controller itself reports: the kill channel from
RC_CHANNELS (streamed at 50 Hz for the test), the motor outputs from HIL_ACTUATOR_CONTROLS (HITL: the board's
actual output commands, at the simulator rate) with their armed flag, and the nose_lift module's state. A trial
starts when the kill channel enters the kill position as PX4 computes it (RC_MAP_KILL_SW, RC_KILLSWITCH_TH,
RCn_MIN/TRIM/MAX/REV, as rc_update.cpp does) and records how long until every motor output is off, how long until
the outputs report disarmed, the nose-lift phase at that moment, and whether any motor came back in the seconds after
the switch was released.
"""
from __future__ import annotations

import threading
import time

RC_CHANNELS_ID = 65
OFF = 1e-3                # a motor output at or below this is off (disarmed outputs arrive as 0 or negative)
WATCH_AFTER_RELEASE_S = 4.0


def _param(link, name, default=None):
    v = (getattr(link, "params", {}) or {}).get(name, {}).get("value")
    return default if v is None else v


def kill_engaged(link, raw: float) -> bool | None:
    """rc_update.cpp getRCSwitchOnOffPosition for the kill function, from the board's own parameters."""
    ch = int(_param(link, "RC_MAP_KILL_SW", 0) or 0)
    if ch <= 0 or raw is None or raw <= 0:
        return None
    lo, trim, hi = (float(_param(link, f"RC{ch}_{k}", d)) for k, d in (("MIN", 1000), ("TRIM", 1500), ("MAX", 2000)))
    rev = -1.0 if float(_param(link, f"RC{ch}_REV", 1)) < 0 else 1.0
    v = (raw - trim) / max(hi - trim, 1.0) if raw > trim else (raw - trim) / max(trim - lo, 1.0)
    v = max(-1.0, min(1.0, v * rev)) * 0.5 + 0.5
    th = float(_param(link, "RC_KILLSWITCH_TH", 0.75))
    return (-v > th) if th < 0 else (v > th)


class KillTest:
    def __init__(self, get_link, log):
        self.get_link = get_link
        self.log = log
        self.running = False
        self.trials: list[dict] = []
        self.live: dict = {}
        self.started = 0.0
        self._thread: threading.Thread | None = None

    def start(self) -> dict:
        link = self.get_link()
        if link is None or not getattr(link, "ctl_connected", False):
            return {"ok": False, "error": "not connected to the board"}
        if int(_param(link, "RC_MAP_KILL_SW", 0) or 0) <= 0:
            return {"ok": False, "error": "no kill switch mapped (RC_MAP_KILL_SW is 0): set one in this tab first"}
        if self.running:
            return {"ok": True}
        try:
            link.request_message(RC_CHANNELS_ID, 50.0)
        except Exception:
            pass
        self.running, self.trials, self.started = True, [], time.time()
        self._thread = threading.Thread(target=self._run, name="kill-test", daemon=True)
        self._thread.start()
        self.log("[kill test] recording: arm, start the rotation, then engage the kill switch")
        return {"ok": True}

    def stop(self) -> dict:
        self.running = False
        return {"ok": True}

    def status(self) -> dict:
        return {"running": self.running, "trials": self.trials, "live": self.live,
                "elapsed": round(time.time() - self.started, 1) if self.started else 0.0}

    def _run(self) -> None:
        was_engaged = None
        trial: dict | None = None
        n_motors = None
        while self.running:
            link = self.get_link()
            if link is None:
                time.sleep(0.1)
                continue
            if n_motors is None:
                n_motors = int(_param(link, "CA_ROTOR_COUNT", 0) or 0) or 16
            now = time.time()
            ch = int(_param(link, "RC_MAP_KILL_SW", 0) or 0)
            rc = getattr(link, "rc", {}) or {}
            vals = rc.get("channels") or []
            raw = vals[ch - 1] if 0 < ch <= len(vals) and now - rc.get("t", 0) < 1.0 else None
            engaged = kill_engaged(link, raw) if raw is not None else None
            acts = [float(a) for a in (getattr(link, "actuators", []) or [])[:n_motors]]
            peak = max(acts) if acts else 0.0
            out_armed = bool(getattr(link, "actuator_armed", False))
            nl = getattr(link, "nose_lift_fw", {}) or {}
            nl_state = nl.get("state") if now - nl.get("t", 0) < 1.0 else None
            self.live = {"kill_raw": raw, "kill_engaged": engaged, "motor_peak": round(peak, 3), "outputs_armed": out_armed,
                         "armed": bool(getattr(link, "armed", False)), "nose_lift": nl_state}
            if engaged and was_engaged is False:
                if trial is not None and trial["released_t"] is not None:
                    # engaged again before the watch window ended: judge the previous one on what it saw so far
                    trial["restarted"] = trial["max_after_release"] > OFF
                    trial.pop("_t0", None)
                trial = {"t": round(now - self.started, 3), "phase": nl_state or ("armed" if out_armed else "disarmed"),
                         "motor_peak_before": round(self._peak_before, 3), "armed_before": self._armed_before,
                         "motors_off_ms": None, "disarmed_ms": None, "released_t": None, "restarted": None,
                         "max_after_release": 0.0, "_t0": now}
                self.trials.append(trial)
                self.log(f"[kill test] kill engaged during '{trial['phase']}' (motors at {trial['motor_peak_before']:.2f})")
            if trial is not None:
                if trial["motors_off_ms"] is None and peak <= OFF:
                    trial["motors_off_ms"] = round((now - trial["_t0"]) * 1000, 1)
                if trial["disarmed_ms"] is None and not out_armed:
                    trial["disarmed_ms"] = round((now - trial["_t0"]) * 1000, 1)
                if engaged is False and trial["released_t"] is None:
                    trial["released_t"] = now
                if trial["released_t"] is not None:
                    trial["max_after_release"] = round(max(trial["max_after_release"], peak), 3)
                    if now - trial["released_t"] > WATCH_AFTER_RELEASE_S:
                        trial["restarted"] = trial["max_after_release"] > OFF
                        trial.pop("_t0", None)
                        self.log(f"[kill test] motors off after {trial['motors_off_ms']} ms, outputs disarmed after "
                                 f"{trial['disarmed_ms']} ms; after the release: "
                                 f"{'MOTORS CAME BACK' if trial['restarted'] else 'nothing restarted'}")
                        trial = None
            if engaged is not None:
                was_engaged = engaged
            if not engaged:
                self._peak_before, self._armed_before = peak, out_armed
            time.sleep(0.002)

    _peak_before = 0.0
    _armed_before = False
