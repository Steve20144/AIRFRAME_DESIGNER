"""Aristotelis bundle for the POWERED Atlas v1: the clean skin plus the nine fans as actuator discs (momentum sources
on the air, OpenFOAM fvOptions vectorSemiImplicitSource in cylinder cell sets). Fan positions and axes come from
the airframe JSON (rotors, FRD metres); the user clicked the fans on the lower skin, so each disc is placed just
below the skin in the air (--offset). Wing fans blow aft along the Coanda wall and the jet is taken as already
turned by --jet-deg below the aft direction (V3 CFD, 1 Oct: the jet follows about 63 % of the 75 deg foil, 47 deg);
the nose trio blows straight down. Thrust per fan is a parameter (cruise guess: wing fans carry about two thirds of
the weight plus the drag, nose fans off).

  .venv/bin/python scripts/atlas_v1_powered_bundle.py --batch b05-powered-60kmh --speed-kmh 60 \
      --wing-thrust 60 --nose-thrust 0 --alphas=-3,0,3,6,9 --betas=0,5,10,15 --partition rome --cores 64
"""
import argparse, json, math
from pathlib import Path
import numpy as np
import airframe_designer.cfd.hpc as H

ROOT = Path(__file__).resolve().parents[1]
ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--airframe", default=str(ROOT / "airframes/atlas_v1_m001.json"))
ap.add_argument("--batch", required=True); ap.add_argument("--speed-kmh", type=float, default=60.0)
ap.add_argument("--quality", default="standard"); ap.add_argument("--alphas", default="-3,0,3,6,9"); ap.add_argument("--betas", default="0,5,10,15")
ap.add_argument("--wing-thrust", type=float, default=60.0, help="N per wing fan (6 fans)")
ap.add_argument("--nose-thrust", type=float, default=0.0, help="N per nose fan (3 fans)")
ap.add_argument("--jet-deg", type=float, default=47.0, help="wing jet angle below the aft direction after the Coanda wall")
ap.add_argument("--offset", type=float, default=0.08, help="m, disc centre below the clicked skin point (into the air)")
ap.add_argument("--partition", default="rome"); ap.add_argument("--cores", type=int, default=64); ap.add_argument("--parallel", type=int, default=8)
ap.add_argument("--dry", action="store_true", help="print the fans and stop")
a = ap.parse_args()
af = json.loads(Path(a.airframe).read_text())
fans = []
for r in af["rotors"]:
    p = np.array(r["pos"], float)
    if r.get("duct_axis"):                      # wing fan: duct +x, jet aft and turned down
        j = math.radians(a.jet_deg); jet = [-math.cos(j), 0.0, math.sin(j)]; T = a.wing_thrust
    else:                                       # nose fan: vertical, blows down (+z in FRD)
        jet = [0.0, 0.0, 1.0]; T = a.nose_thrust
    if T <= 0:
        continue
    c = p + np.array([0.0, 0.0, a.offset])      # the clicked point is on the lower skin: the air is below (+z down)
    fans.append({"name": r["name"], "centre": [round(float(x), 4) for x in c], "jet_dir": [round(x, 4) for x in jet],
                 "thrust_N": T, "diameter_m": float(r.get("diameter", 0.195)), "thickness_m": 0.06})
print(json.dumps(fans, indent=1))
tot = np.sum([np.array(f["jet_dir"]) * f["thrust_N"] for f in fans], axis=0) if fans else np.zeros(3)
print(f"{len(fans)} fans, total thrust on the aircraft FRD {(-tot).round(1).tolist()} N (x forward, z down: negative z = up)")
if a.dry:
    raise SystemExit
H.CLUSTER["partition"] = a.partition
m = H.export_bundle("atlas_v1", a.batch, [float(x) for x in a.alphas.split(",")], [float(x) for x in a.betas.split(",")], a.speed_kmh, a.quality,
                    cores_per_node=a.cores, max_parallel_jobs=a.parallel, walltime_case="02:00:00", walltime_mesh="01:30:00", log=print, fans=fans)
print(m["tar"], m["tar_mb"], "MB")
