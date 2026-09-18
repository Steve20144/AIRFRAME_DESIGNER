#!/usr/bin/env python
"""Search the design space for a configuration that tolerates the most reaction torque.

Atlas's ten fans all turn the same way, so their reaction torques add instead of cancelling, and the coefficient
that sets how much torque that is has never been measured. Rather than wait for the measurement, this asks the
opposite question: what would the aircraft have to look like for the answer not to depend on it?

The search is over the things that can still be changed, which is deliberately a short list. Hover pitch is free
(it is one PX4 parameter). The centre of gravity moves if the battery moves. The front fans' sideways tilt is a
printed bracket. The foil exit angles are not here, because changing them means a new foil.

The score is the largest reaction-torque coefficient at which PX4's pseudo-inverse allocation still asks every
rotor for positive thrust, found by bisection. That static check is optimistic compared with a closed-loop
flight, which also needs authority left over for attitude control, so treat the numbers as an ordering rather
than a promise and fly the winners.
"""
from __future__ import annotations

import argparse
import copy
import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from airframe_designer.geometry.airframe import Airframe  # noqa: E402


def max_km(af: Airframe, hi: float = 0.10, iters: int = 18) -> float:
    """Largest |km| whose hover allocation is still feasible, by bisection."""
    lo = 0.0
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        probe = copy.deepcopy(af)
        for r in probe.rotors:
            r.km = -mid
        lo, hi = (mid, hi) if probe.hover_check()["ok"] else (lo, mid)
    return lo


def utilisation(af: Airframe, km: float) -> float:
    probe = copy.deepcopy(af)
    for r in probe.rotors:
        r.km = -km
    hc = probe.hover_check()
    if not hc["ok"]:
        return float("nan")
    return max(u for u in hc["hover_utilisation"] if u is not None)


def variant(base: Airframe, pitch: float, dx: float, dz: float, front_tilt_deg: float) -> Airframe:
    af = copy.deepcopy(base)
    af.hover_pitch_deg = float(pitch)
    af.mass.cg = [base.mass.cg[0] + dx, base.mass.cg[1], base.mass.cg[2] + dz]
    # The two nose fans are the ones with no jetfoil: re-aim them at the requested sideways tilt, in opposition.
    front = [r for r in af.rotors if not r.duct_axis]
    for k, r in enumerate(front):
        s = math.radians(front_tilt_deg) * (1.0 if k % 2 == 0 else -1.0)
        r.axis = [0.0, math.sin(s), -math.cos(s)]
    return af


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--airframe", default="airframes/atlas_phase01_legs.json")
    ap.add_argument("--max-util", type=float, default=0.80,
                    help="reject a design whose worst rotor exceeds this fraction of its thrust in hover")
    ap.add_argument("--assess-km", type=float, default=0.005, help="reaction torque at which to report utilisation")
    ap.add_argument("--top", type=int, default=12)
    ap.add_argument("--out", default="results/km_margin.json")
    a = ap.parse_args(argv)

    base = Airframe.load(a.airframe)
    baseline = max_km(base)
    print(f"{base.name}: as built, hover pitch {base.hover_pitch_deg:g} deg -> tolerates |km| <= {baseline:.4f}\n")

    pitches = np.arange(22.0, 40.01, 0.5)
    dxs = np.arange(-0.12, 0.041, 0.02)
    dzs = np.arange(-0.14, 0.061, 0.02)
    tilts = (0.0, 10.0, 20.0, 30.0, 40.0)
    total = len(pitches) * len(dxs) * len(dzs) * len(tilts)
    print(f"searching {total} configurations (pitch x CG x CG x front tilt)...")

    rows = []
    for pitch, dx, dz, tilt in itertools.product(pitches, dxs, dzs, tilts):
        af = variant(base, pitch, dx, dz, tilt)
        k = max_km(af)
        if k <= baseline:
            continue
        u = utilisation(af, min(a.assess_km, k * 0.95))
        if not math.isnan(u) and u <= a.max_util:
            rows.append({"max_km": round(k, 5), "hover_pitch_deg": round(float(pitch), 2),
                         "cg_aft_mm": round(-float(dx) * 1000, 1), "cg_up_mm": round(-float(dz) * 1000, 1),
                         "front_tilt_deg": float(tilt), "util": round(float(u), 3)})

    rows.sort(key=lambda r: -r["max_km"])
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps({"airframe": a.airframe, "baseline_max_km": round(baseline, 5),
                                       "max_util": a.max_util, "candidates": rows}, indent=2))

    if not rows:
        print("nothing beat the baseline within the utilisation limit.")
        return 0
    print(f"\n{len(rows)} configurations beat it. Best {min(a.top, len(rows))}:\n")
    print(f"{'max |km|':>9}{'pitch':>8}{'CG aft':>9}{'CG up':>8}{'front tilt':>12}{'util':>7}")
    for r in rows[:a.top]:
        print(f"{r['max_km']:>9.4f}{r['hover_pitch_deg']:>8.1f}{r['cg_aft_mm']:>8.0f}mm{r['cg_up_mm']:>7.0f}mm"
              f"{r['front_tilt_deg']:>11.0f}d{r['util']:>7.3f}")
    best = rows[0]
    print(f"\nbest is {best['max_km'] / baseline:.1f}x the baseline.")
    print("The battery is 3.23 kg of 11.19, so it carries the CG 0.289 of its own travel:")
    print(f"  that CG needs the pack {abs(best['cg_aft_mm']) / 0.289:.0f} mm aft "
          f"and {abs(best['cg_up_mm']) / 0.289:.0f} mm up.")
    print(f"\n-> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
