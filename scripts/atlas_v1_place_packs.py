"""ATLAS v1 M002: add the 18 flight packs and the 9 BMS boards to the user's clicked masses (M001) so that the CG lands
a chosen distance ahead of the M001 CG (default 0.25 m: about 19 % static margin at 60 km/h by the strip-theory
Cruise Test, where the M001 CG gave 8 % and +0.6 m gave 34 % with a 21 deg elevon). The packs are a block inside
the centre body ahead of the wing (CAD |x| <= 0.17, 0.32 m long, 0.18 m tall; the skin there is 0.96 m wide and
0.6 m tall). The user's masses.json is not touched; the result is masses_m002.json next to it.

  .venv/bin/python scripts/atlas_v1_place_packs.py [--cg-fwd 0.25] [--out .../masses_m002.json]
"""
import argparse, json
from datetime import datetime, timezone
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CAND = ROOT / "results/handoff/T20261005-atlas-v1-foil-references/from_A/v002"
R = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], float); Q = np.array([0.0, 0.0, 5.0])
PACK_G, PACK_DIMS = 2356.0, (0.170, 0.108, 0.060)     # Profuse 14S 15 Ah, BOM row 32: 170 x 60 x 108 mm (x, y, z here)
BMS_G = 403.0                                          # AYAA EF-001, BOM row 29
ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--masses", default=str(CAND / "masses.json"))
ap.add_argument("--out", default=str(CAND / "masses_m002.json"))
ap.add_argument("--cg-fwd", type=float, default=0.25, help="m, how far ahead of the M001 CG the new CG must land")
a = ap.parse_args()
doc = json.loads(Path(a.masses).read_text())
items = list(doc["items"])
m0 = sum(it["grams"] for it in items) / 1000
x0 = sum(it["grams"] * it["pos_cad_m"][1] for it in items) / 1000 / m0      # CAD y (aft) of the M001 CG
mp, mb = 18 * PACK_G / 1000, 9 * BMS_G / 1000
M = m0 + mp + mb
y_t = x0 - a.cg_fwd                                                           # nose is CAD -Y: forward = smaller y
y_p = (y_t * M - m0 * x0 - mb * 0.25) / (mp + mb)                             # BMS sit 0.25 m aft of the pack block
stamp = datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")
new = []
k = 0
for iz in range(3):                 # 3 layers of 60 mm
    for iy in range(3):             # 3 along the body (108 mm)
        for ix in range(2):         # 2 across (170 mm)
            k += 1
            p = [(-0.5 + ix) * PACK_DIMS[0], y_p + (iy - 1) * PACK_DIMS[1], 5.20 + (iz - 1) * PACK_DIMS[2]]
            new.append({"name": f"Flight battery Profuse 14S 15Ah #{k}", "grams": PACK_G, "pos_cad_m": [round(v, 4) for v in p],
                        "picked_on_body": "placed by scripts/atlas_v1_place_packs.py (inside the centre body, not on the skin)",
                        "timestamp": stamp, "bom_row": 32, "bom_name": "Flight battery: Profuse 14S 15 Ah 15C semi-solid LiPo, 2 per 28S assembly",
                        "subsystem": "Energy Storage & Charging", "dims_m": list(PACK_DIMS)})
for k in range(9):
    p = [(k % 3 - 1) * 0.13, y_p + 0.25, 5.14 + (k // 3) * 0.04]
    new.append({"name": f"28S BMS AYAA EF-001 #{k + 1}", "grams": BMS_G, "pos_cad_m": [round(v, 4) for v in p],
                "picked_on_body": "placed by scripts/atlas_v1_place_packs.py (inside the centre body)", "timestamp": stamp, "bom_row": 29,
                "bom_name": "28S BMS, isolated CAN / DroneCAN, AYAA EF-001 24-32S 350 A, one per assembly", "subsystem": "Energy Storage & Charging"})
for it in new:
    it["pos_frd_m"] = [round(float(v), 4) for v in (R @ (np.array(it["pos_cad_m"]) - Q))]
items += new
mt = sum(it["grams"] for it in items) / 1000
cg = np.array([sum(it["grams"] * it["pos_cad_m"][i] for it in items) / 1000 / mt for i in range(3)])
out = {**doc, "items": items, "updated": stamp, "derived_from": str(Path(a.masses).relative_to(ROOT)),
       "note": f"M001 clicks + 18 flight packs + 9 BMS placed by scripts/atlas_v1_place_packs.py for a CG {a.cg_fwd:.2f} m ahead of the M001 CG"}
Path(a.out).write_text(json.dumps(out, indent=1))
print(f"M001 {m0:.3f} kg at CAD y {x0:.3f}; packs block centre CAD y {y_p:.3f} (FRD x {-y_p:.3f}); total {mt:.3f} kg, CG CAD {np.round(cg, 3).tolist()} "
      f"= FRD {np.round(R @ (cg - Q), 3).tolist()}; saved {a.out}")
