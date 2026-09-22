"""Build airframes/atlas_og.json: the ATLAS as it stands in the lab, from the Fusion design PHASE_0_V4.

What comes straight from the CAD (read over the Fusion MCP server on 2026-09-18 and pasted below):
  * every top-level part's centre of mass and inertia (CAD densities), and its placement,
  * the ten XFLY 80 mm EDF units: position and duct axis (the eight foil fans point fore-aft inside JET_FOIL_V1,
    the two nose fans sit in FRONT_JET_TILT30, canted +-30 deg),
  * the leg hard points (CLAMP_HUB_TIP_L/R, CLAMP_X_B45-B39_L, CLAMP_X_B38-B39_R), which are the same as the
    landing stand designed on PHASE_0.1, so that stand is reused,
  * the visual mesh, merged from the per-part STL exports in airframes/meshes/_og_parts (component frame, mm).

What is NOT in the CAD and is an assumption, kept identical to the earlier ATLAS airframes so results compare:
  * part masses: the CAD densities give 52 kg; the per-part weighing of 2026-09-14 is used where a part was
    weighed, the CAD mass scaled by the weighed/CAD ratio of the weighed parts otherwise. The PID RIG MOUNT is a
    test fixture and is left out.
  * jet deflection: the foil ducts are straight in the CAD (axis fore-aft). The jetfoil is assumed to bend each
    jet down-and-forward by 25/30/35/40 deg from vertical (outer to inner station) with a 10% turning loss, the
    jet leaving at the foil's aft-lower edge (measured on the mesh, same as PHASE_0.1). Unvalidated; the same assumption as atlas_phase01.
  * thrust curve (33.3 N, quadratic), spool-up 0.12 s, reaction torque km -0.002 (the +-30 deg bracket aircraft
    was only shown to hover up to |km| 0.002).

Frames: Fusion is x right, y DOWN, z forward, centimetres; body FRD = (z, x, y) / 100. Same origin.
"""
from __future__ import annotations

import glob
import json
import math
import struct
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MESH_DIR = ROOT / "airframes" / "meshes"

# ---- Fusion readout (PHASE_0_V4), centimetres, kg, kg cm^2 about the design origin ------------------------------
PARTS = {  # name: (cad mass kg, com cm, [Ixx Iyy Izz Ixy Iyz Ixz] kg cm^2 about origin)
    "XFLY 80mm EDF:3": (0.8308, [12.8, 12.201, -9.468], [208.57, 221.19, 264.69, -129.76, 95.98, 100.69]),
    "BATTERY GRID:1": (1.1487, [-0.102, 2.168, -26.752], [913.33, 940.58, 39.32, 0.25, 66.65, -3.13]),
    "BATTER PACK:1": (12.4742, [-0.101, 5.157, -26.683], [9859.81, 9703.22, 552.78, 6.52, 1716.41, -33.76]),
    "CLAMP_X_B37-B41_R:1": (0.3015, [6.877, 0.327, 1.264], [1.37, 15.52, 15.3, -0.61, -0.19, -2.72]),
    "CLAMP_X_B44-B42_L:1": (0.3015, [-6.877, 0.327, 1.264], [1.37, 15.52, 15.3, 0.61, -0.19, 2.72]),
    "CLAMP_X_B38-B41_R:1": (0.2983, [6.047, 1.834, 58.444], [1020.6, 1030.42, 12.86, -3.26, -31.98, -105.43]),
    "CLAMP_X_B45-B42_L:1": (0.2983, [-6.047, 1.834, 58.444], [1020.6, 1030.42, 12.86, 3.26, -31.98, 105.43]),
    "CLAMP_X_B38-B39_R:1": (0.293, [27.853, 12.53, 11.678], [86.93, 268.0, 274.16, -102.23, -42.91, -95.42]),
    "CLAMP_X_B45-B39_L:1": (0.293, [-27.853, 12.53, 11.678], [86.93, 268.0, 274.16, 102.23, -42.91, 95.42]),
    "CLAMP_HUB_TIP_R:1": (0.814, [40.833, 19.841, -16.984], [560.35, 1598.24, 1681.36, -660.08, 274.77, 565.54]),
    "CLAMP_HUB_NOSE:1": (0.7805, [0.0, 1.264, 68.354], [3650.98, 3650.74, 4.74, 0.0, -66.99, 0.0]),
    "CLAMP_HUB_CENTER:1": (0.4468, [0.0, -4.384, 4.96], [20.59, 13.23, 10.87, 0.0, 9.87, 0.0]),
    "CLAMP_FOILROOT_L:1": (0.3531, [-15.176, 5.467, -2.908], [15.44, 86.19, 92.67, 29.21, 5.48, -15.61]),
    "Avionics_Bay:1": (0.5935, [0.0, 0.132, 17.606], [216.52, 224.88, 8.39, 0.0, -1.33, 0.0]),
    "FRONT_JET_TILT30:1": (4.2604, [0.0, 1.689, 45.0], [8783.27, 8810.74, 86.57, 0.0, -323.74, -5.26]),
    "JET_FOIL_V1:1": (6.5122, [26.074, 19.522, -18.832], [5530.23, 7892.16, 7995.54, -3730.16, 2613.87, 3399.63]),
    "CLAMP_FOILROOT_R_V3:1": (0.9213, [19.341, 8.34, -8.509], [146.56, 427.03, 410.97, -148.7, 67.04, 151.89]),
    "JET_FOIL_V1(Mirror):1": (6.7938, [-26.056, 19.964, -19.622], [6210.81, 8527.66, 8494.5, 3973.21, 2943.64, -3685.6]),
    "XFLY 80mm EDF(Mirror) (1):1": (0.8308, [-12.8, 12.201, -9.468], [208.57, 221.19, 264.69, 129.76, 95.98, -100.69]),
    "CLAMP_HUB_TIP_R(Mirror):1": (0.814, [-40.833, 19.841, -16.984], [560.37, 1598.3, 1681.43, 660.11, 274.78, -565.56]),
    "XFLY 80mm EDF:4": (0.8308, [21.55, 17.381, -11.968], [380.41, 515.43, 641.72, -311.2, 172.83, 214.29]),
    "XFLY 80mm EDF:5": (0.8308, [30.4, 22.601, -14.468], [608.73, 952.33, 1197.11, -570.85, 271.68, 365.43]),
    "XFLY 80mm EDF:6": (0.8308, [39.1, 27.851, -16.968], [894.09, 1519.99, 1919.54, -904.77, 392.64, 551.23]),
    "XFLY 80mm EDF(Mirror) (1):2": (0.8308, [-21.55, 17.381, -11.968], [380.41, 515.43, 641.72, 311.2, 172.83, -214.29]),
    "XFLY 80mm EDF(Mirror) (1):3": (0.8308, [-30.4, 22.601, -14.468], [608.73, 952.33, 1197.11, 570.85, 271.68, -365.43]),
    "XFLY 80mm EDF(Mirror) (1):4": (0.8308, [-39.1, 27.851, -16.968], [894.09, 1519.99, 1919.54, 904.77, 392.64, -551.23]),
}
# occurrence transforms (row-major 4x4, cm) for placing the component-frame STL exports
TRANSFORMS = {
    "XFLY 80mm EDF:3": [1, 0, 0, 12.8, 0, 0, -1, 12.2, 0, 1, 0, -10.1, 0, 0, 0, 1],
    "BATTERY GRID:1": [1, 0, 0, 0, 0, 1, 0, -34.3, 0, 0, 1, 0, 0, 0, 0, 1],
    "BATTER PACK:1": [1, 0, 0, 0, 0, 1, 0, -34.3, 0, 0, 1, 0, 0, 0, 0, 1],
    "Avionics_Bay:1": [-1, 0, 0, 0, 0, -1, 0, 0.262122, 0, 0, 1, 17.0, 0, 0, 0, 1],
    "FRONT_JET_TILT30:1": [-1, 0, 0, 0, 0, -1, 0, 4.3, 0, 0, 1, 45.0, 0, 0, 0, 1],
    "JET_FOIL_V1:1": [1, 0, 0, 12.806157, 0, 0, 1, 12.225451, 0, -1, 0, -23.28876, 0, 0, 0, 1],
    "JET_FOIL_V1(Mirror):1": [-1, 0, 0, -12.806157, 0, 0, -1, 12.225451, 0, -1, 0, -23.28876, 0, 0, 0, 1],
    "XFLY 80mm EDF(Mirror) (1):1": [-1, 0, 0, -12.8, 0, 0, 1, 12.2, 0, 1, 0, -10.1, 0, 0, 0, 1],
    "CLAMP_HUB_TIP_R(Mirror):1": [-1, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1],
    "XFLY 80mm EDF:4": [1, 0, 0, 21.55, 0, 0, -1, 17.38, 0, 1, 0, -12.6, 0, 0, 0, 1],
    "XFLY 80mm EDF:5": [1, 0, 0, 30.4, 0, 0, -1, 22.6, 0, 1, 0, -15.1, 0, 0, 0, 1],
    "XFLY 80mm EDF:6": [1, 0, 0, 39.1, 0, 0, -1, 27.85, 0, 1, 0, -17.6, 0, 0, 0, 1],
    "XFLY 80mm EDF(Mirror) (1):2": [-1, 0, 0, -21.55, 0, 0, 1, 17.38, 0, 1, 0, -12.6, 0, 0, 0, 1],
    "XFLY 80mm EDF(Mirror) (1):3": [-1, 0, 0, -30.4, 0, 0, 1, 22.6, 0, 1, 0, -15.1, 0, 0, 0, 1],
    "XFLY 80mm EDF(Mirror) (1):4": [-1, 0, 0, -39.1, 0, 0, 1, 27.85, 0, 1, 0, -17.6, 0, 0, 0, 1],
}
# per-part weighing of 2026-09-14 (grams); the V4 names map onto the weighed parts
WEIGHED_G = {
    "BATTER PACK:1": 3230, "BATTERY GRID:1": 60, "CLAMP_FOILROOT_L:1": 44, "CLAMP_FOILROOT_R_V3:1": 44,
    "CLAMP_HUB_CENTER:1": 45, "CLAMP_HUB_NOSE:1": 48, "CLAMP_HUB_TIP_R:1": 74, "CLAMP_HUB_TIP_R(Mirror):1": 74,
    "CLAMP_X_B37-B41_R:1": 37 + 35 + 73, "CLAMP_X_B44-B42_L:1": 37 + 35 + 73,        # clamp + its two rods
    "CLAMP_X_B38-B41_R:1": 40 + 73 + 73, "CLAMP_X_B45-B42_L:1": 40 + 73 + 73,
    "CLAMP_X_B38-B39_R:1": 40 + 73 + 73, "CLAMP_X_B45-B39_L:1": 40 + 73 + 73,
    "JET_FOIL_V1:1": 950, "JET_FOIL_V1(Mirror):1": 950,
    "FRONT_JET_TILT30:1": 177 + 2 * 520,                                                # the weighed bracket assembly and its two fans
}
EDF_G = 520

# the ten fans: FRD position of the fan centre (from the CAD), thrust (force) direction, physical duct axis
CM = 0.01
def frd(p_cm):  # Fusion (x right, y down, z fwd) cm -> body FRD m
    return [p_cm[2] * CM, p_cm[0] * CM, p_cm[1] * CM]

# Jet exit per station: the aft-lower edge of JET_FOIL_V1 measured on the exported mesh (right foil, FRD m), where
# the foil channel releases the jet; within 6 mm of the PHASE_0.1 exits, so the foil itself is unchanged.
FOIL_STATIONS = [  # (exit FRD x, y, z) right side; assumed deflection from vertical, deg
    ((-0.402, 0.392, 0.365), 25.0), ((-0.373, 0.309, 0.310), 30.0), ((-0.346, 0.214, 0.256), 35.0), ((-0.318, 0.132, 0.204), 40.0)]
FRONT = [  # cad origin of the two nose fans and their duct axis (cad), jet blows +y (down)
    ("EDF_09_FRONT_AFT", (-1.397, 3.06, 39.5), (-0.5, 0.866, 0.0)),
    ("EDF_10_FRONT_FORWARD", (1.397, 3.06, 50.5), (0.5, 0.866, 0.0))]
ROTOR_COMMON = {"km": -0.002, "max_thrust": 33.3, "tau": 0.12, "diameter": 0.1032, "thrust_exponent": 2.0,
                "kind": "ducted", "ram_drag": True, "turn_loss": 0.1, "enabled": True}
# The two JET_FOIL_V1 bodies as one symmetric strip-theory wing (their force in the outside flow; the jet-turning
# reaction is already the rotor thrust at the exit). Planform measured on the part mesh 2026-09-21: |y| 0.075-0.444 m,
# chord 0.31 (root) to 0.34 (tip), leading edge at x -0.06 sweeping to -0.136 at the tip, mid-surface from z 0.115 at
# the root to 0.289 at the tip (28.5 deg anhedral). Section coefficients are ESTIMATES for a thick ducted body.
FOIL_WING = {
    "name": "JET_FOIL_V1 pair (estimated planform)", "enabled": True,
    "pos": [-0.06, 0.0, 0.06], "span": 0.888, "root_chord": 0.31, "tip_chord": 0.34, "sweep_deg": 10.0,
    "dihedral_deg": -28.5, "incidence_deg": 0.0, "twist_deg": 0.0, "pitch_deg": 0.0, "symmetric": True, "panels": 4,
    "aspect_ratio": None,
    "aero": {"model": "linear", "cl_alpha": 6.2832, "cl0": 0.0, "cd0": 0.08, "oswald": 0.7,
             "stall_deg": 12.0, "stall_blend_deg": 10.0, "cd_flat": 1.2, "cm0": 0.0},
}


def masses() -> dict[str, float]:
    out = {}
    for name, (m_cad, _, _) in PARTS.items():
        if "EDF" in name:
            out[name] = EDF_G / 1000.0
        elif name in WEIGHED_G:
            out[name] = WEIGHED_G[name] / 1000.0
    weighed_cad = sum(PARTS[n][0] for n in out)
    ratio = sum(out.values()) / weighed_cad
    for name, (m_cad, _, _) in PARTS.items():
        if name not in out:
            out[name] = round(m_cad * ratio, 4)     # Avionics_Bay: not weighed
    return out


def mass_properties():
    m = masses()
    total = sum(m.values())
    cg_cad = np.zeros(3)
    for name, (m_cad, com, _) in PARTS.items():
        cg_cad += m[name] * np.asarray(com)
    cg_cad /= total
    # inertia about the new CG: each part's CAD inertia about its own CoM, scaled by weighed/CAD, plus point-mass term
    I = np.zeros((3, 3))
    for name, (m_cad, com, (xx, yy, zz, xy, yz, xz)) in PARTS.items():
        c = np.asarray(com)
        # Fusion's moments are about the origin; move to the part's CoM (parallel axis), scale, move to the CG
        I_o = np.array([[xx, -xy, -xz], [-xy, yy, -yz], [-xz, -yz, zz]])
        I_com = I_o - m_cad * (np.dot(c, c) * np.eye(3) - np.outer(c, c))
        d = c - cg_cad
        I += I_com * (m[name] / m_cad) + m[name] * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    I *= 1e-4                       # kg cm^2 -> kg m^2
    P = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)   # FRD = P @ cad
    I_frd = P @ I @ P.T
    return total, frd(cg_cad.tolist()), I_frd, m


def rotors():
    out = []
    for i, ((x, y, z), defl) in enumerate(FOIL_STATIONS):
        for side, sgn in (("LEFT", -1.0), ("RIGHT", 1.0)):
            pos = [x, sgn * y, z]
            a = math.radians(defl)
            out.append({"name": f"EDF_{2 * i + (1 if side == 'LEFT' else 2):02d}_FOIL_{side}_{['OUTER', 'MID_OUTER', 'MID_INNER', 'INNER'][i]}",
                        "pos": [round(v, 6) for v in pos], "axis": [round(math.sin(a), 6), 0.0, round(-math.cos(a), 6)],
                        "duct_axis": [1.0, 0.0, 0.0], **ROTOR_COMMON})
    for name, origin, duct in FRONT:
        force = [-duct[0], -duct[1], -duct[2]]           # the jet blows along the duct axis (down); thrust is opposite
        out.append({"name": name, "pos": [round(v, 6) for v in frd(list(origin))],
                    "axis": [round(v, 6) for v in frd(force)], "duct_axis": None, **ROTOR_COMMON})
    # order like the other ATLAS files: outer -> inner, left/right, then front aft, front forward
    return out


def merge_mesh(out_path: Path) -> int:
    tris = []
    for f in sorted(glob.glob(str(MESH_DIR / "_og_parts" / "*.stl"))):
        name = Path(f).stem
        occ = next((k for k in PARTS if "".join(ch if ch.isalnum() else "_" for ch in k) == name), None)
        if occ is None:
            continue
        T = np.asarray(TRANSFORMS.get(occ, [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]), float).reshape(4, 4)
        b = open(f, "rb").read()
        n = struct.unpack("<I", b[80:84])[0]
        v = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))["v"].reshape(-1, 3)
        w = (v / 10.0) @ T[:3, :3].T + T[:3, 3]          # mm -> cm, component -> design frame
        p = np.stack([w[:, 2], w[:, 0], w[:, 1]], axis=1) * CM   # -> FRD, m
        tris.append(p.reshape(-1, 3, 3).astype("<f4"))
    allt = np.concatenate(tris)
    rec = np.zeros(len(allt), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    rec["v"] = allt
    e1 = allt[:, 1] - allt[:, 0]; e2 = allt[:, 2] - allt[:, 0]
    nrm = np.cross(e1, e2); nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    rec["n"] = nrm
    with open(out_path, "wb") as fh:
        fh.write(b"PHASE_0_V4 visual, body FRD, m".ljust(80, b"\0"))
        fh.write(struct.pack("<I", len(rec)))
        fh.write(rec.tobytes())
    return len(rec)


def main() -> None:
    total, cg, I, m = mass_properties()
    stand = json.loads((ROOT / "airframes" / "atlas_phase01_best.json").read_text())
    af = {
        "schema": 2, "name": "ATLAS_OG",
        "mass": {"mass": round(total, 3), "cg": [round(v, 6) for v in cg],
                 "inertia": [round(float(I[i, i]), 6) for i in range(3)],
                 "inertia_products": [round(float(-I[0, 1]), 6), round(float(-I[0, 2]), 6), round(float(-I[1, 2]), 6)],
                 "items": [], "from_items": False},
        "body": dict(stand["body"]),
        "rotors": rotors(),
        "wings": [FOIL_WING],
        "legs": stand["legs"],                       # same clamp hard points as PHASE_0.1; re-staged below
        "hover_pitch_deg": stand["hover_pitch_deg"],
        "landed_pitch_deg": stand["landed_pitch_deg"],
        "px4_overrides": {"MC_AIRMODE": 1, "THR_MDL_FAC": 1.0, "MPC_TKO_RAMP_T": 1.5, "MPC_THR_HOVER": 0.4,
                          "MIS_TAKEOFF_ALT": 3, "IMU_INTEG_RATE": 250},
        "design": {"nose_lift": {"enabled": True, "motors": [8, 9], "target_pitch_deg": stand["hover_pitch_deg"],
                                 "rate_deg_s": 8, "assist_cmd": 0.0}},
        "mesh": {"file": "atlas_og.stl", "frame": "frd", "scale": 1.0, "opacity": 0.85,
                 "source": "Fusion PHASE_0_V4, per-part STL export 2026-09-18, merged by scripts/atlas_og_from_fusion.py"},
        "notes": ("ATLAS as built in the lab: geometry, hard points and mesh from the Fusion design PHASE_0_V4 "
                  "(JET_FOIL_V1, FRONT_JET_TILT30 nose fans canted +-30 deg), masses from the 2026-09-14 weighing. "
                  "Jet deflection angles, thrust curve, km and the foil turning loss are assumptions carried over from "
                  "atlas_phase01; see scripts/atlas_og_from_fusion.py."),
    }
    (ROOT / "airframes" / "atlas_og_stand.json").write_text(json.dumps(af, indent=2))
    n = merge_mesh(MESH_DIR / "atlas_og.stl")
    print(f"mass {total:.3f} kg, cg FRD {np.round(cg, 4).tolist()}, inertia diag {np.round(np.diag(I), 4).tolist()}, mesh {n} triangles")
    print("masses:", {k: v for k, v in m.items()})
    print("wrote airframes/atlas_og_stand.json (on the stand) - run atlas_pivot_stance.py for the -16 deg park")


if __name__ == "__main__":
    main()
