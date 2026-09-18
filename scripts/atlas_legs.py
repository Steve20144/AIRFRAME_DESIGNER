#!/usr/bin/env python
"""Place Atlas's landing legs on real CAD hardpoints and write the airframe that stands on them.

Atlas cannot simply be stood on the ground: its hull has no statically stable resting attitude, and standing it at
the hover attitude means the nose points up, so everything forward of the centre of gravity is also *above* it.
Front legs are therefore long, and the only structure behind the centre of gravity is the pair of outboard hub
clamps, 20 mm aft. Legs dropped straight down from those would tip backwards at about 2 degrees.

So the rear legs are raked aft: the attachment is structure, the foot is wherever the geometry needs it. The feet
are placed in the *landed* frame (the aircraft pitched nose-up, gravity straight down), which is the frame the
tip-over margins live in; the conversion to the structural frame happens once, here.

Nothing about the attachment points is verified to carry the load. They are the nodes the CAD calls clamps, which
is where a leg would sensibly go, and two of them have never been weighed. Print nothing on the strength of this
script alone.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from airframe_designer.geometry.airframe import Airframe  # noqa: E402
from airframe_designer.geometry.gear import Leg  # noqa: E402

#: CAD bodies used as leg attachments, by the role each leg plays.
HARDPOINTS = {
    "RL": "CLAMP_HUB_TIP_L:1", "RR": "CLAMP_HUB_TIP_R:1",
    "FL": "CLAMP_X_B45-B39_L:1", "FR": "CLAMP_X_B38-B39_R:1",
}


def landed_basis(pitch_deg: float):
    """Unit vectors of the landed frame in structural coordinates: horizontal-forward, and gravity."""
    p = math.radians(pitch_deg)
    return (math.cos(p), 0.0, math.sin(p)), (-math.sin(p), 0.0, math.cos(p))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--airframe", default="airframes/atlas_phase01_sym.json")
    ap.add_argument("--cad", default="../mujoco-vibe/vehicles/atlas-phase01/source/cad_geometry.json")
    ap.add_argument("--out", default="airframes/atlas_phase01_legs.json")
    ap.add_argument("--clearance", type=float, default=0.08, help="ground clearance under the lowest structure, m")
    ap.add_argument("--rear-foot", type=float, default=-0.20, help="rear foot, m behind the CG in the landed frame")
    ap.add_argument("--front-foot", type=float, default=0.20, help="front foot, m ahead of the CG")
    ap.add_argument("--rear-track", type=float, default=0.41, help="rear foot half-track, m")
    ap.add_argument("--front-track", type=float, default=0.28, help="front foot half-track, m")
    ap.add_argument("--sink", type=float, default=0.02, help="static compression of a leg under its share, m")
    ap.add_argument("--zeta", type=float, default=0.8, help="damping ratio (below 1 does not bounce)")
    ap.add_argument("--foot-radius", type=float, default=0.02)
    a = ap.parse_args(argv)

    af = Airframe.load(a.airframe)
    cad = json.loads(Path(a.cad).read_text())
    body = {b["name"]: b["centroid_frd_m"] for b in cad["bodies"]}
    bb = cad["bounding_box_m"]
    corners = [[x, y, z] for x in (bb["min"][0], bb["max"][0])
               for y in (bb["min"][1], bb["max"][1]) for z in (bb["min"][2], bb["max"][2])]

    cg = list(af.mass.cg)
    pitch = af.landed_pitch_deg
    fwd, down = landed_basis(pitch)
    proj = lambda P, v: sum((P[i] - cg[i]) * v[i] for i in range(3))

    # The ground plane has to clear the lowest structure at this attitude, which is the foil, not the fuselage.
    drop = max(proj(c, down) for c in corners)
    height = drop + a.clearance

    plan = {"RL": (a.rear_foot, -a.rear_track), "RR": (a.rear_foot, a.rear_track),
            "FL": (a.front_foot, -a.front_track), "FR": (a.front_foot, a.front_track)}
    share = af.mass.mass * 9.80665 / 4.0
    stiffness = round(share / a.sink, 1)
    damping = round(2.0 * a.zeta * math.sqrt(stiffness * af.mass.mass / 4.0), 1)

    legs, report = [], []
    for name, (fore, lat) in plan.items():
        attach = body[HARDPOINTS[name]]
        # foot = cg + fore * forward + lat * sideways + height * down, in structural coordinates
        foot = [cg[i] + fore * fwd[i] + height * down[i] + (lat if i == 1 else 0.0) for i in range(3)]
        leg = Leg.from_points(attach, foot, name=name, stiffness=stiffness, damping=damping,
                              friction=0.8, foot_radius=a.foot_radius)
        legs.append(leg)
        rake = math.degrees(math.atan2(fore - proj(attach, fwd), height - proj(attach, down)))
        report.append((name, HARDPOINTS[name], leg.length, -rake, proj(attach, down)))

    af.legs = legs
    af.save(a.out)

    print(f"{af.name}: standing at {pitch:.2f} deg nose-up, ground {height:.3f} m below the CG "
          f"({a.clearance*1000:.0f} mm under the lowest structure)")
    print(f"springs {stiffness:.0f} N/m, damping {damping:.0f} Ns/m ({a.sink*1000:.0f} mm sink, zeta {a.zeta})\n")
    print(f"{'leg':<5}{'attaches to':<26}{'length m':>10}{'rake aft deg':>14}{'hardpoint drop m':>18}")
    for name, hp, length, rake, hdrop in report:
        print(f"{name:<5}{hp:<26}{length:>10.3f}{rake:>+14.1f}{hdrop:>18.3f}")

    print(f"\nfootprint: {a.front_foot - a.rear_foot:.2f} m fore-aft, "
          f"{2*a.rear_track:.2f} m rear track, {2*a.front_track:.2f} m front track")
    for label, margin in (("aft", -a.rear_foot), ("forward", a.front_foot),
                          ("sideways (rear)", a.rear_track), ("sideways (front)", a.front_track)):
        print(f"  tips {label:<17} beyond {math.degrees(math.atan2(margin, height)):.1f} deg")
    for w in af.validate():
        print(f"  ! {w}")
    print(f"\n-> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
