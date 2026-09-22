"""Scripted flights: a scenario is a list of phases that drive PX4 through MAVLink while the simulation runs.

A scenario is JSON:
{
  "name": "takeoff_hover_land",
  "max_time": 90,                          # simulation seconds before the run is declared a timeout
  "params": {"MIS_TAKEOFF_ALT": 3},        # PX4 parameters for this scenario (seeded before boot in batch)
  "wind": [0, 0, 0],                       # NED m/s at the start
  "abort": {"max_tilt_deg": 100, "max_alt": 200, "crash_speed": 3.0},
  "phases": [
    {"type": "wait_ready", "timeout": 40},
    {"type": "takeoff", "alt": 3, "settle": 2, "timeout": 30},
    {"type": "hold", "duration": 10},
    {"type": "offboard_velocity", "vel": [12, 0, 0], "duration": 12, "yaw": 0},
    {"type": "offboard_position", "pos": [0, 0, -5], "tolerance": 1.0, "timeout": 30},
    {"type": "manual", "roll": 0, "pitch": 0.6, "throttle": 0.5, "yaw": 0, "duration": 5, "mode": "position"},
    {"type": "wind", "ned": [5, 0, 0]},
    {"type": "motor_failure", "rotor": 2, "scale": 0.0},
    {"type": "param", "name": "MC_PITCH_P", "value": 5.0},
    {"type": "wait", "duration": 2},
    {"type": "land", "timeout": 40}
  ]
}
Every phase may carry "name" (metrics are grouped by it; default = type). All MAVLink traffic is fire-and-forget
and checked on later steps, so the runner never blocks the lockstep loop.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from ..px4.link import DEBUG_VECT_ID
from .metrics import MetricsRecorder

SCENARIO_DIR = Path(__file__).resolve().parents[2] / "scenarios"


@dataclass
class Scenario:
    name: str = "scenario"
    phases: list[dict] = field(default_factory=list)
    max_time: float = 120.0
    params: dict = field(default_factory=dict)
    wind: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    abort: dict = field(default_factory=dict)
    description: str = ""
    attitude: dict = field(default_factory=dict)     # {"park_pitch_deg": -10, "hover_pitch_deg": 25}: the flight starts
                                                     # parked at the first, PX4 levels at the second (legs re-solved)
    design: dict = field(default_factory=dict)       # merged into airframe.design, e.g. {"nose_lift": {"executor": "firmware"}}

    @classmethod
    def from_dict(cls, d: dict) -> "Scenario":
        return cls(name=d.get("name", "scenario"), phases=list(d.get("phases", [])), max_time=float(d.get("max_time", 120.0)),
                   params=dict(d.get("params", {}) or {}), wind=list(d.get("wind", [0, 0, 0]) or [0, 0, 0]),
                   abort=dict(d.get("abort", {}) or {}), description=str(d.get("description", "")),
                   attitude=dict(d.get("attitude", {}) or {}), design=dict(d.get("design", {}) or {}))

    def to_dict(self) -> dict:
        return {"name": self.name, "description": self.description, "max_time": self.max_time, "params": self.params,
                "wind": self.wind, "abort": self.abort, "phases": self.phases, "attitude": self.attitude,
                "design": self.design}

    def apply_attitude(self, airframe):
        """The airframe as this scenario wants it parked and hovering (unchanged when the scenario says nothing)."""
        a = self.attitude or {}
        if a.get("park_pitch_deg") is not None or a.get("hover_pitch_deg") is not None:
            airframe = airframe.with_attitude(park_pitch_deg=a.get("park_pitch_deg"), hover_pitch_deg=a.get("hover_pitch_deg"))
        if self.design:
            import copy
            airframe = copy.deepcopy(airframe)
            for k, v in self.design.items():
                cur = airframe.design.get(k)
                airframe.design[k] = {**cur, **v} if isinstance(cur, dict) and isinstance(v, dict) else v
        return airframe


def load_scenario(spec: str | Path | dict) -> Scenario:
    if isinstance(spec, dict):
        return Scenario.from_dict(spec)
    p = Path(spec)
    if not p.is_file():
        alt = SCENARIO_DIR / (p.name if p.name.endswith(".json") else p.name + ".json")
        if alt.is_file():
            p = alt
        else:
            raise FileNotFoundError(f"scenario not found: {spec}")
    with open(p) as f:
        return Scenario.from_dict(json.load(f))


class ScenarioRunner:
    """A simulator hook that executes the phases and fills a MetricsRecorder."""

    def __init__(self, scenario: Scenario, link, log=None, metrics: MetricsRecorder | None = None):
        self.sc = scenario
        self.link = link
        self.log = log or (lambda s: None)
        self.metrics = metrics or MetricsRecorder()
        self.index = -1
        self.phase: dict | None = None
        self.phase_t0 = 0.0
        self.done = False
        self.ok = True
        self.status = "running"
        self.failures: list[str] = []
        self.targets: dict[str, dict] = {}
        self._state: dict[str, Any] = {}
        self._rest_alt = None
        self._airborne_once = False
        self._started = False
        self._wind_applied = False
        self.max_time = scenario.max_time
        self._rc: list[int] | None = None     # scripted transmitter (rc phases), streamed at 50 Hz once set
        self._rc_on = True

    # ------------------------------------------------------------ helpers
    def _name(self) -> str:
        return self.phase.get("name") or self.phase.get("type", "phase")

    def _elapsed(self, simr) -> float:
        return simr.t - self.phase_t0

    def _next(self, simr) -> None:
        self.index += 1
        if self.index >= len(self.sc.phases):
            self.finish(simr, "done", self.ok)
            return
        self.phase = dict(self.sc.phases[self.index])
        self.phase_t0 = simr.t
        self._state = {}
        self.metrics.begin_phase(self._name(), simr.t)
        self.log(f"[scenario] t={simr.t:.1f}s phase {self.index + 1}/{len(self.sc.phases)}: {self._name()}")

    def fail(self, simr, why: str, fatal: bool = False) -> None:
        self.failures.append(f"{self._name() if self.phase else 'init'}: {why}")
        self.ok = False
        self.log(f"[scenario] t={simr.t:.1f}s FAIL {why}")
        if fatal:
            self.finish(simr, "aborted", False)
        else:
            self._next(simr)

    def finish(self, simr, status: str, ok: bool) -> None:
        if self.done:
            return
        self.metrics.end_phase(simr.t)
        self.done = True
        self.ok = ok and self.ok
        self.status = status
        self.log(f"[scenario] t={simr.t:.1f}s finished: {status} ({'ok' if self.ok else 'failed'})")

    # ---------------------------------------------------------------- hook
    def __call__(self, simr) -> None:
        if self.done:
            return
        s = simr.sim
        if not self._started:
            self._started = True
            self._t_start = simr.t          # a live simulator may have been running for a long time already
            self._rest_alt = -float(s.pos[2])
            if self.link is not None and hasattr(self.link, "request_message"):
                try:
                    self.link.request_message(83, 50.0)          # ATTITUDE_TARGET for setpoint-vs-actual metrics
                except Exception:
                    pass
            if any(abs(v) > 0 for v in self.sc.wind):
                simr.sim.wind_ned = np.array(self.sc.wind, float)
            self._next(simr)
            if self.done:
                return
        self.metrics(simr)
        # global abort checks
        ab = self.sc.abort
        airborne = not s.on_ground
        if airborne:
            self._airborne_once = True
        if simr.t - getattr(self, "_t_start", 0.0) > self.max_time:
            self.fail(simr, f"scenario exceeded max_time {self.max_time}s", fatal=True); return
        if self._airborne_once and s.tilt_deg > float(ab.get("max_tilt_deg", 110.0)):
            self.metrics.crashed = True; self.metrics.crash_reason = f"tilt {s.tilt_deg:.0f} deg"
            self.fail(simr, "crashed: tilt " + f"{s.tilt_deg:.0f} deg", fatal=True); return
        if -float(s.pos[2]) > float(ab.get("max_alt", 500.0)):
            self.fail(simr, "flew above max_alt", fatal=True); return
        if self._airborne_once and s.on_ground and float(np.linalg.norm(s.vel)) > float(ab.get("crash_speed", 3.0)):
            self.metrics.crashed = True; self.metrics.crash_reason = f"hit the ground at {np.linalg.norm(s.vel):.1f} m/s"
            self.fail(simr, self.metrics.crash_reason, fatal=True); return
        link = self.link
        if link is not None and link.main_mode == 10 and self._airborne_once:
            self.metrics.crashed = True; self.metrics.crash_reason = "PX4 flight termination"
            self.fail(simr, "PX4 flight termination", fatal=True); return
        if self._rc is not None and self._rc_on and link is not None:
            if simr.step_count % max(1, int(round(simr.sensor_rate / 50.0))) == 0:
                link.send_rc_override(self._rc)
        handler = getattr(self, "_p_" + self.phase.get("type", ""), None)
        if handler is None:
            self.fail(simr, f"unknown phase type '{self.phase.get('type')}'"); return
        handler(simr)

    # ---------------------------------------------------------------- phases
    def _p_wait(self, simr) -> None:
        if self._elapsed(simr) >= float(self.phase.get("duration", 1.0)):
            self._next(simr)

    def _p_wait_ready(self, simr) -> None:
        link = self.link
        st = self._state
        st.setdefault("wall_t0", time.time())
        # only a summary PX4 issued after this phase began counts: the vehicle may just have been put back on
        # its legs, and a HITL estimator needs a few seconds to accept that
        can = link.can_arm_modes(since=st["wall_t0"]) if link is not None else ""
        if self._elapsed(simr) < float(self.phase.get("min_wait", 0.0)):
            return
        want = [str(m) for m in (self.phase.get("modes") or ["takeoff", "loiter", "posctl"])]
        ready = link is not None and link.ctl_connected and any(m in can for m in want)
        if not can and link is not None and link.ctl_connected and self._elapsed(simr) > float(self.phase.get("summary_wait", 12.0)):
            # a HITL board whose event metadata is not loaded never yields a decoded arming summary: go on and let
            # the arm/takeoff phase report what PX4 says
            self.log(f"[scenario] no arming-check summary from PX4 after {self._elapsed(simr):.0f}s (event metadata missing?), proceeding")
            self._next(simr)
        elif ready:
            self.log(f"[scenario] PX4 ready after {simr.t:.1f}s sim time (can arm: {can.replace('|', ', ')})")
            self._next(simr)
        elif self._elapsed(simr) > float(self.phase.get("timeout", 60.0)):
            self.fail(simr, f"PX4 never became ready to arm (last summary: '{can}')", fatal=True)

    def _p_takeoff(self, simr) -> None:
        st, link = self._state, self.link
        el = self._elapsed(simr)
        alt = -float(simr.sim.pos[2]) - (self._rest_alt or 0.0)
        target = float(self.phase.get("alt", 2.5))
        self.targets[self._name()] = {"alt": target + (self._rest_alt or 0.0)}
        if "sent" not in st:
            if not link.armed and link.main_mode in (1, 2, 7) and "premode" not in st:
                # PX4 refuses to arm in a manual mode without stick input: ask for Hold first, then arm
                link.set_mode("hold"); st["premode"] = simr.t
                return
            if "premode" in st and simr.t - st["premode"] < 0.7:
                return
            if not link.armed:
                link.arm()
            st["sent"] = simr.t
            st["mode_at"] = None
            return
        if st["mode_at"] is None and link.armed and simr.t - st["sent"] > 0.2:
            link.set_mode("takeoff"); st["mode_at"] = simr.t
        elif st["mode_at"] is None and simr.t - st["sent"] > 3.0 and not link.armed:
            # a HITL estimator can take a while to accept the attitude change of a nose lift: keep asking
            link.arm(); st["sent"] = simr.t; st["arm_retries"] = st.get("arm_retries", 0) + 1
            if el > float(self.phase.get("arm_timeout", 40.0)):
                self.fail(simr, f"PX4 refused to arm for {el:.0f}s (can arm: '{link.can_arm_modes()}')", fatal=True)
            return
        if st["mode_at"] is not None and not link.mode_is("takeoff") and not link.mode_is("hold") and simr.t - st["mode_at"] > 2.0 and "retry" not in st:
            link.set_mode("takeoff"); st["retry"] = True
        reached = alt >= 0.9 * target and abs(float(simr.sim.vel[2])) < 0.3
        if reached or (link.mode_is("hold") and alt > 0.5 * target):
            if "reached" not in st:
                st["reached"] = simr.t
                self.metrics.note(simr.t, f"takeoff altitude reached after {simr.t - self.phase_t0:.1f}s")
                self.metrics.phases[self._name()]["time_to_alt"] = round(simr.t - self.phase_t0, 2)
            if simr.t - st["reached"] >= float(self.phase.get("settle", 2.0)):
                self._next(simr)
        elif el > float(self.phase.get("timeout", 30.0)):
            self.fail(simr, f"takeoff did not reach {target} m in {el:.0f}s (alt {alt:.2f} m, armed={link.armed}, mode={link.custom_mode >> 16 & 0xFF})", fatal=bool(self.phase.get("fatal", True)))

    def _p_hold(self, simr) -> None:
        st, link = self._state, self.link
        if "sent" not in st:
            link.set_mode("hold"); st["sent"] = simr.t
            st["pos0"] = simr.sim.pos.copy()
            self.targets[self._name()] = {"pos": st["pos0"].tolist()}
        if self._elapsed(simr) >= float(self.phase.get("duration", 10.0)):
            self._next(simr)

    def _p_land(self, simr) -> None:
        st, link = self._state, self.link
        if "sent" not in st:
            link.set_mode("land"); st["sent"] = simr.t
        landed = simr.sim.on_ground and not link.armed
        if landed:
            self.metrics.note(simr.t, "landed and disarmed")
            self._next(simr)
        elif self._elapsed(simr) > float(self.phase.get("timeout", 60.0)):
            self.fail(simr, "did not land in time")

    def _offboard_stream(self, simr, pos=None, vel=None, yaw=0.0) -> None:
        st = self._state
        every = max(1, int(round(simr.sensor_rate / 20.0)))     # 20 Hz setpoints
        if simr.step_count % every == 0:
            self.link.send_setpoint_local(int(simr.t * 1000), pos=pos, vel=vel, yaw=yaw)
        if "mode_at" not in st and self._elapsed(simr) > 0.5:
            self.link.set_mode("offboard"); st["mode_at"] = simr.t
        elif "mode_at" in st and not self.link.mode_is("offboard") and simr.t - st["mode_at"] > 1.5:
            self.link.set_mode("offboard"); st["mode_at"] = simr.t
            st["mode_retries"] = st.get("mode_retries", 0) + 1
            if st["mode_retries"] > 6:
                self.fail(simr, "PX4 did not accept offboard mode")

    def _p_offboard_velocity(self, simr) -> None:
        vel = [float(v) for v in self.phase.get("vel", [5, 0, 0])]
        yaw = self.phase.get("yaw", 0.0)
        self.targets[self._name()] = {"vel": vel}
        self._offboard_stream(simr, vel=vel, yaw=None if yaw is None else math.radians(float(yaw)))
        if self._elapsed(simr) >= float(self.phase.get("duration", 10.0)):
            self._next(simr)

    def _p_offboard_position(self, simr) -> None:
        pos = [float(v) for v in self.phase.get("pos", [0, 0, -5])]
        yaw = self.phase.get("yaw", 0.0)
        self.targets[self._name()] = {"pos": pos}
        self._offboard_stream(simr, pos=pos, yaw=None if yaw is None else math.radians(float(yaw)))
        err = float(np.linalg.norm(simr.sim.pos - np.array(pos)))
        st = self._state
        if err < float(self.phase.get("tolerance", 1.0)) and float(np.linalg.norm(simr.sim.vel)) < float(self.phase.get("speed_tolerance", 0.5)):
            st.setdefault("reached", simr.t)
            if simr.t - st["reached"] >= float(self.phase.get("settle", 1.0)):
                self.metrics.phases[self._name()]["time_to_target"] = round(simr.t - self.phase_t0, 2)
                self._next(simr)
        elif self._elapsed(simr) > float(self.phase.get("timeout", 30.0)):
            self.fail(simr, f"did not reach {pos} (error {err:.1f} m)")

    def _p_manual(self, simr) -> None:
        st, link = self._state, self.link
        every = max(1, int(round(simr.sensor_rate / 50.0)))
        if simr.step_count % every == 0:
            link.send_manual_control(float(self.phase.get("roll", 0)), float(self.phase.get("pitch", 0)),
                                     float(self.phase.get("throttle", 0.5)), float(self.phase.get("yaw", 0)))
        if "mode_at" not in st and self._elapsed(simr) > 0.3:
            link.set_mode(self.phase.get("mode", "position")); st["mode_at"] = simr.t
        if self._elapsed(simr) >= float(self.phase.get("duration", 5.0)):
            self._next(simr)

    def _p_stick(self, simr) -> None:
        """A pilot's sticks, streamed at 50 Hz: the mode is selected once the stream is up, PX4 is armed if asked,
        and the throttle follows a smooth (cosine) ramp from ``throttle_from`` to ``throttle`` over ``ramp_s``
        (default: the whole phase). ``alt_hold`` {target, kp, kv, max_corr, min, max} makes the scripted pilot work
        the throttle around that value to hold a height (Stabilized has no altitude loop). Ends after ``duration`` s,
        or as soon as the hover-frame pitch has been within
        ``until_pitch_tol_deg`` of ``until_pitch_deg`` for ``settle`` s. Roll/pitch/yaw sticks are held constant."""
        st, link = self._state, self.link
        el = self._elapsed(simr)
        thr1 = float(self.phase.get("throttle", 0.0))
        thr0 = float(self.phase.get("throttle_from", st.get("thr_prev", thr1)))
        ramp = float(self.phase.get("ramp_s", self.phase.get("duration", 5.0)))
        if ramp > 1e-6 and el < ramp:
            thr = thr0 + (thr1 - thr0) * 0.5 * (1.0 - math.cos(math.pi * el / ramp))
        else:
            thr = thr1
        thr = self._alt_hold(simr, thr, el)
        every = max(1, int(round(simr.sensor_rate / 50.0)))
        if simr.step_count % every == 0:
            link.send_manual_control(float(self.phase.get("roll", 0)), float(self.phase.get("pitch", 0)), thr,
                                     float(self.phase.get("yaw", 0)))
        self.last_throttle = thr
        mode = self.phase.get("mode", "stabilized")
        if "mode_at" not in st and el > 0.3:
            link.set_mode(mode); st["mode_at"] = simr.t
        elif "mode_at" in st and not link.mode_is(mode) and simr.t - st["mode_at"] > 1.5:
            link.set_mode(mode); st["mode_at"] = simr.t          # PX4 refused or was not listening yet: ask again
        if self.phase.get("arm") and not link.armed and link.mode_is(mode):
            if "arm_at" not in st or simr.t - st["arm_at"] > 1.5:
                link.arm(force=bool(self.phase.get("force", False))); st["arm_at"] = simr.t
                st["arm_tries"] = st.get("arm_tries", 0) + 1
            if st.get("arm_tries", 0) > int(self.phase.get("arm_tries", 6)):
                self.fail(simr, f"PX4 did not arm (can arm: '{link.can_arm_modes()}')", fatal=True); return
        if self.phase.get("arm") and link.armed and "armed_at" not in st:
            st["armed_at"] = simr.t
            self.metrics.note(simr.t, f"armed after {el:.1f}s")
            self.metrics.phases[self._name()]["time_to_arm"] = round(el, 2)
        if self.phase.get("disarm") and link.armed and el > 0.5 and ("disarm_at" not in st or simr.t - st["disarm_at"] > 1.5):
            link.disarm(force=bool(self.phase.get("force", False))); st["disarm_at"] = simr.t
        target = self.phase.get("until_pitch_deg")
        if target is not None:
            _, p, _ = simr.sim.hover_frame_euler()
            pitch_deg = math.degrees(p)
            if abs(pitch_deg - float(target)) <= float(self.phase.get("until_pitch_tol_deg", 2.0)):
                st.setdefault("in_tol_since", simr.t)
                if simr.t - st["in_tol_since"] >= float(self.phase.get("settle", 0.5)):
                    self.metrics.note(simr.t, f"pitch at {pitch_deg:.1f} deg (hover frame) after {el:.1f}s")
                    self.metrics.phases[self._name()]["time_to_pitch"] = round(el, 2)
                    self._finish_stick(simr, thr); return
            else:
                st.pop("in_tol_since", None)
        if self.phase.get("until_ground"):
            # a landing under the sticks: end once the vehicle has been airborne and is back on its legs for settle s
            if not simr.sim.on_ground:
                st["ug_air"] = True
            if st.get("ug_air") and simr.sim.on_ground:
                st.setdefault("ug_since", simr.t)
                if simr.t - st["ug_since"] >= float(self.phase.get("settle", 0.5)):
                    self.metrics.note(simr.t, f"on the ground after {el:.1f}s of the stick landing")
                    self.metrics.phases[self._name()]["time_to_ground"] = round(el, 2)
                    self._finish_stick(simr, thr); return
            else:
                st.pop("ug_since", None)
        if el >= float(self.phase.get("duration", 5.0)):
            if target is not None and self.phase.get("require", True):
                self.fail(simr, f"pitch did not settle at {target} deg within {el:.0f}s", fatal=bool(self.phase.get("fatal", True))); return
            if self.phase.get("until_ground") and self.phase.get("require", True):
                self.fail(simr, f"did not touch down within {el:.0f}s", fatal=bool(self.phase.get("fatal", True))); return
            self._finish_stick(simr, thr)

    def _alt_hold(self, simr, thr: float, el: float) -> float:
        """The phase's ``alt_hold``: a pilot's throttle hand in a mode without altitude control, nudging the stick
        around the phase throttle by the height error and the climb rate (alt above the rest altitude, m; vz up,
        m/s). Attitude stays entirely with PX4; this only keeps the aircraft in the band a pilot would."""
        ah = self.phase.get("alt_hold")
        if not ah:
            return thr
        st = self._state
        alt = -float(simr.sim.pos[2]) - (self._rest_alt or 0.0)
        vz_up = -float(simr.sim.vel[2])
        target = float(ah.get("target", 3.0))
        rate = ah.get("rate")
        if rate is not None:                 # slew the height target from where the phase started, like a pilot
            if "ah_from" not in st:
                st["ah_from"] = alt
            tgt0 = st["ah_from"]
            target = tgt0 + max(-abs(float(rate)) * el, min(abs(float(rate)) * el, target - tgt0))
        st["ah_target"] = target
        corr = float(ah.get("kp", 0.05)) * (target - alt) - float(ah.get("kv", 0.08)) * vz_up
        lim = float(ah.get("max_corr", 0.15))
        return min(float(ah.get("max", 0.85)), max(float(ah.get("min", 0.2)), thr + max(-lim, min(lim, corr))))

    def _p_rc(self, simr) -> None:
        """A transmitter as the flight controller's receiver sees it (RC_CHANNELS_OVERRIDE -> input_rc), for
        firmware that reads switches itself: the kill switch, the nose-lift module's switch. Channels persist across
        phases and stream at 50 Hz from the first rc phase on (all 1500 us, throttle channel 3 at 1000 us).
          channels   {"7": 1000, ...} pulse widths to set on entry (1-based channel numbers)
          throttle   0..1 on channel 3, with throttle_from / ramp_s for a cosine ramp
          radio      "off" stops the stream (a transmitter lost), "on" resumes it
          arm        arm PX4 with a MAVLink command (retried); mode: select that mode once
          until      {"nl_state": "holding" | [...], "armed": bool, "motors_off": true, "pitch_deg": x, "tol": 2}
                     ends the phase when met (timeout s, default 30; require, default true, fails the run if not);
                     without it the phase lasts duration s (default 2)
          expect     {"nl_abort": "kill switch", "nl_state": "..."}: checked when the phase ends"""
        st, link = self._state, self.link
        el = self._elapsed(simr)
        if "entered" not in st:
            st["entered"] = True
            if self._rc is None:
                self._rc = [1500] * 18
                self._rc[2] = 1000
                try:
                    link.request_message(DEBUG_VECT_ID, 20.0)     # the nose_lift module's state
                except Exception:
                    pass
            for k, v in (self.phase.get("channels") or {}).items():
                if 1 <= int(k) <= 18:
                    self._rc[int(k) - 1] = int(v)
            radio = self.phase.get("radio")
            if radio in ("off", "on"):
                self._rc_on = radio == "on"
                self.metrics.note(simr.t, f"transmitter {radio}")
            st["thr_from"] = (self._rc[2] - 1000) / 1000.0
        if "throttle" in self.phase:
            thr1 = float(self.phase["throttle"])
            thr0 = float(self.phase.get("throttle_from", st["thr_from"]))
            ramp = float(self.phase.get("ramp_s", 0.0))
            thr = thr1 if ramp <= 1e-6 or el >= ramp else thr0 + (thr1 - thr0) * 0.5 * (1.0 - math.cos(math.pi * el / ramp))
            thr = self._alt_hold(simr, thr, el)
            self._rc[2] = int(round(1000 + 1000 * min(1.0, max(0.0, thr))))
            self.last_throttle = thr
        mode = self.phase.get("mode")
        if mode and "mode_at" not in st:
            link.set_mode(mode); st["mode_at"] = simr.t
        if self.phase.get("arm") and not link.armed and el > 0.3 and ("arm_at" not in st or simr.t - st["arm_at"] > 1.5):
            link.arm(force=bool(self.phase.get("force", False))); st["arm_at"] = simr.t
            st["arm_tries"] = st.get("arm_tries", 0) + 1
            if st["arm_tries"] > int(self.phase.get("arm_tries", 6)):
                self.fail(simr, f"PX4 did not arm (can arm: '{link.can_arm_modes()}')", fatal=True); return
        nl = getattr(link, "nose_lift_fw", {}) or {}
        until = self.phase.get("until")
        if until:
            met = True
            if "nl_state" in until:
                want = until["nl_state"] if isinstance(until["nl_state"], list) else [until["nl_state"]]
                met &= nl.get("state") in want and time.time() - nl.get("t", 0) < 1.0
            if "armed" in until:
                met &= bool(link.armed) == bool(until["armed"])
            if until.get("motors_off"):
                n = simr.sim.rotors.n
                met &= all(not (float(c) > 1e-3) for c in list(link.actuators)[:n])
            if "pitch_deg" in until:
                met &= abs(math.degrees(simr.sim.euler[1]) - float(until["pitch_deg"])) <= float(until.get("tol", 2.0))
            if met:
                self.metrics.note(simr.t, f"{self._name()}: {until} after {el:.2f}s")
                self.metrics.phases[self._name()]["time_to_until"] = round(el, 3)
                self._end_rc(simr, nl); return
            if el > float(self.phase.get("timeout", 30.0)):
                if self.phase.get("require", True):
                    self.fail(simr, f"{until} not reached in {el:.0f}s (nose lift {nl.get('state')}, abort "
                                    f"{nl.get('abort')}, armed {link.armed})", fatal=bool(self.phase.get("fatal", True))); return
                self._end_rc(simr, nl); return
        elif el >= float(self.phase.get("duration", 2.0)):
            self._end_rc(simr, nl)

    def _end_rc(self, simr, nl: dict) -> None:
        exp = self.phase.get("expect") or {}
        for key, field_ in (("nl_abort", "abort"), ("nl_state", "state")):
            if key in exp and nl.get(field_) != exp[key]:
                self.fail(simr, f"expected nose lift {field_} '{exp[key]}', got '{nl.get(field_)}'",
                          fatal=bool(self.phase.get("fatal", True)))
                return
        self._next(simr)

    def _finish_stick(self, simr, thr: float) -> None:
        self._next(simr)
        if not self.done:
            self._state["thr_prev"] = thr        # the next stick phase ramps from where this one left the throttle

    def _p_wind(self, simr) -> None:
        simr.sim.wind_ned = np.array(self.phase.get("ned", [0, 0, 0]), float)
        self.metrics.note(simr.t, f"wind set to {simr.sim.wind_ned.tolist()}")
        self._next(simr)

    def _p_motor_failure(self, simr) -> None:
        scales = simr.sim.rotors.scale.copy()
        i = int(self.phase.get("rotor", 0))
        if 0 <= i < len(scales):
            scales[i] = float(self.phase.get("scale", 0.0))
        simr.sim.set_rotor_health(scales)
        self.metrics.note(simr.t, f"rotor {i + 1} thrust scaled to {self.phase.get('scale', 0.0)}")
        self._next(simr)

    def _p_param(self, simr) -> None:
        st, link = self._state, self.link
        name, value = str(self.phase.get("name")), self.phase.get("value")
        if "sent" not in st:
            if not link.param_set_nowait(name, value):
                self.fail(simr, f"parameter {name} unknown to the link (not downloaded)"); return
            st["sent"] = simr.t
            return
        got = link.params.get(name, {}).get("value")
        if got is not None and abs(float(got) - float(value)) <= 1e-4 * max(1.0, abs(float(value))):
            self.metrics.note(simr.t, f"{name} = {value}")
            self._next(simr)
        elif self._elapsed(simr) > 3.0:
            self.fail(simr, f"no echo for parameter {name}")

    def _p_nose_lift(self, simr) -> None:
        """Raise the nose with the given motors (0-based) to target_pitch_deg before arming; the following takeoff
        phase arms PX4 and the sequence hands over by itself."""
        st = self._state
        if "nl" not in st:
            # a live simulator was just put back on its legs: let it settle on the ground first, or the sequence
            # sees a vehicle "in the air" at its first step and gives up
            if not simr.sim.on_ground:
                st.pop("settled_since", None)
                if self._elapsed(simr) > 10.0:
                    self.fail(simr, "vehicle never settled on the ground before the nose lift", fatal=True)
                return
            st.setdefault("settled_since", simr.t)
            if simr.t - st["settled_since"] < 0.5:
                return
            motors = self.phase.get("motors") or []
            kw = {k: self.phase[k] for k in ("rate_deg_s", "kp", "ki", "kd", "max_cmd", "tolerance_deg", "hold_s", "fade_s", "k_ang", "kq", "kqi", "assist_motors", "assist_cmd", "timeout_s", "handover_timeout_s") if k in self.phase}
            design_nl = ((getattr(simr.airframe, "design", None) or {}).get("nose_lift") or {})
            target = self.phase.get("target_pitch_deg", design_nl.get("target_pitch_deg", simr.airframe.hover_pitch_deg))
            st["nl"] = simr.start_nose_lift(motors, float(target), **kw)
            self.targets[self._name()] = {}
            return
        nl = st["nl"]
        if nl.state == "holding":
            self.metrics.note(simr.t, f"nose at {nl.pitch:.1f} deg after {simr.t - self.phase_t0:.1f}s")
            self.metrics.phases[self._name()]["time_to_pitch"] = round(simr.t - self.phase_t0, 2)
            self.metrics.phases[self._name()]["max_cmd"] = round(max((c for _, _, c in nl.history), default=0.0), 3)
            self.metrics.phases[self._name()]["overshoot_deg"] = round(max((p for _, p, _ in nl.history), default=nl.target) - nl.target, 2)
            self.metrics.phases[self._name()]["start_pitch_deg"] = round(nl.history[0][1], 2) if nl.history else None
            self._next(simr)
        elif nl.state == "failed":
            self.fail(simr, f"nose lift failed: {nl.reason}", fatal=bool(self.phase.get("fatal", True)))
        elif self._elapsed(simr) > float(self.phase.get("timeout", 30.0)):
            self.fail(simr, f"nose lift did not reach the target (at {nl.pitch:.1f} deg)", fatal=True)

    def _p_nose_lower(self, simr) -> None:
        """Land the way the aircraft took off: PX4 lands at the hover attitude (unless px4_land is false), the
        simulator takes over at touchdown, cuts the other motors and lowers the nose with ``motors`` to
        target_pitch_deg (default: the airframe's landed pitch) at rate_deg_s. Ends when the nose is down."""
        st = self._state
        if "nl" not in st:
            motors = self.phase.get("motors") or []
            kw = {k: self.phase[k] for k in ("rate_deg_s", "min_airborne_alt", "fade_s", "tolerance_deg", "timeout_s", "wait_timeout_s", "wait_touchdown", "k_ang", "kq", "kqi", "max_cmd", "takeover_boost", "rear_fade_s") if k in self.phase}
            cur = getattr(simr, "nose_lift", None)
            if cur is not None and getattr(cur, "kind", "") == "lower" and cur.state in ("waiting", "lowering", "settling", "done"):
                st["nl"] = cur          # design.nose_lower.enabled already armed one in the air (it may already be
                                        # lowering after a stick landing): adopt it
            else:
                design_lo = ((getattr(simr.airframe, "design", None) or {}).get("nose_lower") or {})
                target = self.phase.get("target_pitch_deg", design_lo.get("target_pitch_deg", simr.airframe.landed_pitch_deg))
                st["nl"] = simr.start_nose_lower(motors, float(target), **kw)
            if self.phase.get("px4_land", True):
                self.link.set_mode("land")
            self.targets[self._name()] = {}
            return
        nl = st["nl"]
        if nl.state in ("lowering", "settling", "done") and "td" not in st and nl.touchdown_speed is not None:
            st["td"] = simr.t
            self.metrics.note(simr.t, f"touchdown at {nl.touchdown_speed:.2f} m/s, pitch {nl.touchdown_pitch:.1f} deg; nose lower started")
            self.metrics.phases[self._name()]["touchdown_speed"] = round(nl.touchdown_speed, 3)
            self.metrics.phases[self._name()]["touchdown_pitch_deg"] = round(nl.touchdown_pitch, 2)
        if nl.state == "done":
            self.metrics.note(simr.t, f"nose down: {nl.reason}")
            ph = self.metrics.phases[self._name()]
            ph["final_pitch_deg"] = round(nl.pitch, 2)
            ph["lower_rate_max_deg_s"] = round(nl.rate_max, 2)
            ph["lower_duration"] = round(simr.t - st.get("td", simr.t), 2)
            self._next(simr)
        elif nl.state == "failed":
            self.fail(simr, f"nose lower failed: {nl.reason}", fatal=bool(self.phase.get("fatal", True)))
        elif self._elapsed(simr) > float(self.phase.get("timeout", 120.0)):
            self.fail(simr, f"nose lower did not finish (state {nl.state}, pitch {nl.pitch:.1f} deg)", fatal=True)

    def _p_arm(self, simr) -> None:
        st, link = self._state, self.link
        if "sent" not in st:
            link.arm(force=bool(self.phase.get("force", False))); st["sent"] = simr.t
        if link.armed:
            self._next(simr)
        elif self._elapsed(simr) > float(self.phase.get("timeout", 5.0)):
            self.fail(simr, "did not arm")

    def _p_mode(self, simr) -> None:
        st, link = self._state, self.link
        mode = self.phase.get("mode", "hold")
        if "sent" not in st:
            link.set_mode(mode); st["sent"] = simr.t
        if link.mode_is(mode):
            self._next(simr)
        elif self._elapsed(simr) > float(self.phase.get("timeout", 3.0)):
            self.fail(simr, f"mode {mode} not accepted")

    # ---------------------------------------------------------------- result
    def result(self, mass: float) -> dict:
        m = self.metrics.summary(mass, self.targets)
        return {"ok": self.ok and not self.metrics.crashed, "status": self.status, "failures": self.failures,
                "sim_time": round(self.metrics.rows[-1][0], 2) if self.metrics.rows else 0.0, "metrics": m}
