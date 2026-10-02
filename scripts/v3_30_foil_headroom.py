"""Static jetfoil sweep on the V3_30 (STEP masses, 14.4 kg) for thrust headroom, with the foil fans' thrust taken from
a jet model calibrated to the OpenFOAM point (foils 75 / 65 / 50 deg inner / middle / outer -> per group
(-56.12, 46.48) N at 3 x 36 N in, see scripts/atlas_v3_30_cfd.py).

One CFD point fixes two numbers per model. Per foil fan with geometric turn d (deg, foil angle):
    jet angle above the duct axis  e = k * d                (the jet under-turns the foil by a fixed ratio)
    thrust out / thrust in         n = 1 - c * f(d)         A: f = d / 90      (loss linear in turning)
                                                            B: f = (d / 90)^2  (loss grows with turning)
                                                            C: f = 0           (flat loss, not tied to turning)
k and c solved so the three fans of the CFD foils sum to the CFD group force. Every inner / middle / outer triple on
RANGE is trimmed (PX4 pseudo-inverse hover mix) and scored on headroom at hover: the busiest fan's share of its
full thrust (u_max; 1 / u_max is the thrust-to-weight left with the moments balanced) and the spare roll / pitch /
yaw moment before a fan hits 0 or 100 %. Force points stay on the V3_30 trailing edges.

  python scripts/v3_30_foil_headroom.py [--out results/v3_30/foil_headroom] [--step 5]
Writes <out>/<YYYYmmdd_HHMMSS>/static.json (every row timestamped), calib.json, airframes for the picks.
"""
from __future__ import annotations
import argparse, copy, itertools, json, math, time
from datetime import datetime
from pathlib import Path
import numpy as np
from scipy.optimize import fsolve, linprog
from airframe_designer.geometry.airframe import Airframe, G

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "airframes" / "atlas_v3_30_cfd.json"
CFD_FOILS = {"inner": 75.0, "middle": 65.0, "outer": 50.0}
CFD_GROUP = (56.12, 46.48)          # forward (against the fan axis), up; N per group of three
FAN_N = 36.0
STATION = {"outer": ("M1", "M2"), "middle": ("M3", "M4"), "inner": ("M5", "M6")}
MODELS = {"A": lambda d: d / 90.0, "B": lambda d: (d / 90.0) ** 2, "C": lambda d: 0.0}


def now() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def calibrate(model: str) -> tuple[float, float]:
    f = MODELS[model]
    ds = list(CFD_FOILS.values())

    def res(x):
        k, c = x
        fx = sum(FAN_N * (1 - c * f(d)) * math.cos(math.radians(k * d)) for d in ds)
        fz = sum(FAN_N * (1 - c * f(d)) * math.sin(math.radians(k * d)) for d in ds)
        return [fx - CFD_GROUP[0], fz - CFD_GROUP[1]]

    if model == "C":                 # c is the flat loss itself
        def res(x):
            k, c = x
            fx = sum(FAN_N * (1 - c) * math.cos(math.radians(k * d)) for d in ds)
            fz = sum(FAN_N * (1 - c) * math.sin(math.radians(k * d)) for d in ds)
            return [fx - CFD_GROUP[0], fz - CFD_GROUP[1]]
    k, c = fsolve(res, [0.6, 0.4])
    return float(k), float(c)


def jet(model: str, k: float, c: float, d: float) -> tuple[float, float]:
    """(jet angle above the duct axis deg, efficiency) for a foil turned d deg."""
    eff = 1 - c if model == "C" else 1 - c * MODELS[model](d)
    return k * d, max(0.0, eff)


def apply(af: Airframe, foils: dict, model: str, k: float, c: float) -> Airframe:
    for st, d in foils.items():
        e, eff = jet(model, k, c, d)
        for r in af.rotors:
            if r.name in STATION[st]:
                er = math.radians(e)
                r.axis = [math.cos(er), 0.0, -math.sin(er)]
                r.duct_axis = [1.0, 0.0, 0.0]
                r.max_thrust = FAN_N
                r.turn_loss = (1 - eff) * 90.0 / r.deflection_deg()
    af.resolve_mass()
    return af


def trim(af: Airframe) -> float | None:
    t = af.trim_hover_pitch(step=0.5)
    return None if t is None else af.trim_hover_pitch(lo=t - 0.6, hi=t + 0.6, step=0.05)


def hover(af: Airframe) -> dict | None:
    """Trimmed hover: pitch, fan shares and spare moments (N m) before a fan hits 0 or 100 %."""
    th = trim(af)
    if th is None:
        return None
    af.hover_pitch_deg = th
    ct = np.array([r.effective_max_thrust() for r in af.active_rotors()])
    W = af.mass.mass * G
    P = np.linalg.pinv(af.effectiveness() * ct)
    u = P @ np.array([0, 0, 0, 0, 0, -W])
    # spare moment per axis, the worse of the two directions, with the other five axes held at hover and every fan
    # within 0..100 % (linear programme: the most any allocator can get; 0 when the axis cannot be moved alone)
    B, sp = af.effectiveness() * ct, np.array([0, 0, 0, 0, 0, -W])
    spare = []
    for ax in (0, 1, 2):
        Aeq, beq = np.delete(B, ax, 0), np.delete(sp, ax)
        lim = []
        for sgn in (1, -1):
            lp = linprog(-sgn * B[ax], A_eq=Aeq, b_eq=beq, bounds=[(0, 1)] * len(ct), method="highs")
            lim.append(max(0.0, -lp.fun) if lp.status == 0 else 0.0)
        spare.append(min(lim))
    sv = np.linalg.svd(B, compute_uv=False)
    names = [r.name for r in af.active_rotors()]
    return {"hover_pitch_deg": round(th, 2), "u_max": round(float(u.max()), 3), "u_min": round(float(u.min()), 3),
            "busiest_fan": names[int(u.argmax())], "tw_avail": round(float(1 / u.max()), 3),
            "thr_hover": float(af.hover_thrust_fraction()),
            "roll_Nm": round(spare[0], 2), "pitch_Nm": round(spare[1], 2), "yaw_Nm": round(spare[2], 3),
            "foil_fan_u": round(float(u[:6].mean()), 3), "nose_fan_u": round(float(u[6:].mean()), 3),
            "max_thrust_N": round(float(ct.sum()), 1), "sv_min": round(float(sv[-1]), 3)}


def build(foils: dict, model: str, out: Path, log=print) -> Airframe:
    """A flyable airframe for one triple: jet model applied, hover re-trimmed, PX4 hover thrust, nose-lift target and
    the arming tilt limit set as in scripts/atlas_v3_30_cfd.py."""
    k, c = calibrate(model)
    af = apply(Airframe.load(SRC), foils, model, k, c)
    af.hover_pitch_deg = round(trim(af), 2)
    af.px4_overrides["MPC_THR_HOVER"] = round(float(af.hover_thrust_fraction()), 3)
    af.design["nose_lift"]["target_pitch_deg"] = af.hover_pitch_deg
    tilt = af.hover_pitch_deg - af.landed_pitch_deg
    if tilt > 55.0:
        af.px4_overrides["FD_FAIL_P"] = int(math.ceil(tilt / 5.0) * 5 + 5)
    tag = "_".join(f"{int(foils[s])}" for s in ("inner", "middle", "outer"))
    af.name = f"ATLAS_V3_30_FOILS_{tag}"
    af.notes += (f"\n\nFoil headroom sweep ({now()}, scripts/v3_30_foil_headroom.py): foils inner / middle / outer "
                 f"{tag.replace('_', ' / ')} deg, jet model {model} calibrated to the CFD point; hover {af.hover_pitch_deg} deg.")
    af.save(out)
    log(now(), out.name, "hover", af.hover_pitch_deg, "MPC_THR_HOVER", af.px4_overrides["MPC_THR_HOVER"])
    return af


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "results" / "v3_30" / "foil_headroom"))
    ap.add_argument("--lo", type=float, default=30.0); ap.add_argument("--hi", type=float, default=90.0)
    ap.add_argument("--step", type=float, default=5.0)
    ap.add_argument("--build", nargs="*", help="inner,middle,outer triples: write flyable airframes into --out")
    ap.add_argument("--model", default="A")
    a = ap.parse_args()
    if a.build:
        for t in a.build:
            di, dm, do = map(float, t.split(","))
            build({"inner": di, "middle": dm, "outer": do}, a.model, Path(a.out) / f"atlas_v3_30_foils_{int(di)}_{int(dm)}_{int(do)}.json")
        return
    run = Path(a.out) / datetime.now().strftime("%Y%m%d_%H%M%S"); run.mkdir(parents=True, exist_ok=True)
    base = Airframe.load(SRC)
    calib = {m: dict(zip(("k", "c"), calibrate(m))) for m in MODELS}
    for m, kc in calib.items():           # check: the CFD foils give the CFD force
        fx = fz = 0.0
        for d in CFD_FOILS.values():
            e, eff = jet(m, kc["k"], kc["c"], d)
            fx += FAN_N * eff * math.cos(math.radians(e)); fz += FAN_N * eff * math.sin(math.radians(e))
        kc["check_N"] = [round(fx, 2), round(fz, 2)]
        kc["per_foil"] = {st: dict(zip(("jet_deg", "eff"), map(lambda v: round(v, 3), jet(m, kc["k"], kc["c"], d))))
                          for st, d in CFD_FOILS.items()}
    (run / "calib.json").write_text(json.dumps({"ts": now(), "source": str(SRC.relative_to(ROOT)), "cfd_foils": CFD_FOILS,
                                                "cfd_group_N": CFD_GROUP, "models": calib}, indent=1))
    print(now(), "calibration", json.dumps(calib))
    grid = np.arange(a.lo, a.hi + 1e-9, a.step)
    rows, t0 = [], time.time()
    for m in MODELS:
        k, c = calib[m]["k"], calib[m]["c"]
        for di, dm, do in itertools.product(grid, grid, grid):
            foils = {"inner": float(di), "middle": float(dm), "outer": float(do)}
            h = hover(apply(copy.deepcopy(base), foils, m, k, c))
            rows.append({"ts": now(), "model": m, **foils, **(h or {"status": "no trim"})})
        print(now(), f"model {m}: {len(grid) ** 3} triples, {time.time() - t0:.0f} s")
    (run / "static.json").write_text(json.dumps(rows, indent=0))
    print(run)


if __name__ == "__main__":
    main()
