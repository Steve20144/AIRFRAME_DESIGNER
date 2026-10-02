"""Real-flight parameter set for the Pixhawk 6X from an airframe: the PX4 export (geometry, board rotation, gains,
nose lift, H-FLOW estimator offsets for the real mount) with every HITL / bench setting put back for flight, and a
diff against a read-only dump of the board. Nothing is written to the board.

  python scripts/real_flight_params.py --airframe airframes/atlas_v3_30_cfd.json \
      --board results/board_params/params_YYYYMMDD_HHMMSS_readonly_decoded.json --flow-mount-deg 20.3 \
      --out airframes/board/atlas_v3_30_cfd_real_flight.params

Left out on purpose: output functions (PWM_MAIN/AUX_FUNC, HIL_ACT_FUNC: the board's wiring stays as it is), sensor
calibrations, and anything the airframe cannot know (battery cells, flight-mode channel): see the .md next to it.
"""
from __future__ import annotations
import argparse, json, math
from datetime import datetime
from pathlib import Path
from airframe_designer.geometry.airframe import Airframe

ROOT = Path(__file__).resolve().parents[1]
INT_NL = {"NL_EN", "NL_MOT_MSK", "NL_RC_CH", "NL_RC_TH", "NL_RC_LOW"}

# HITL / bench settings from the sim work, put back for flight (why in the comment)
REAL_FLIGHT = {
    "SYS_HITL": 0,              # real sensors and real PWM outputs (1 = simulator only, motors never spin)
    "COM_RC_IN_MODE": 0,        # RC only: the transmitter's arm (ch 8) and kill (ch 5) switches work again
    "NL_EN": 1,                 # firmware nose lift on: it pitches the aircraft up from the legs before take-off
    "COM_DISARM_PRFLT": 10.0,   # armed but not taking off: disarm after 10 s (was -1, never)
    "COM_DISARM_LAND": 2.0,     # disarm 2 s after landing (the airframe carried 120 from bench work)
    "UAVCAN_SUB_FLOW": 1,       # use the real H-FLOW (DroneCAN) flow ...
    "UAVCAN_SUB_RNG": 1,        # ... and range
    "EKF2_OF_GYR_SRC": 1,       # compensate flow with the autopilot's own (board-rotated) gyro, not the sensor's
    "EKF2_DECL_TYPE": 3,        # real field: declination from the world model once GPS has a fix (sim pinned 0)
    "EKF2_MULTI_IMU": 3,        # 6X redundancy defaults (single IMU was to match SITL)
    "SENS_IMU_MODE": 0,
    "SENS_IMU_AUTOCAL": 1,      # temperature / bias autocal back on
    "COM_ARM_WO_GPS": 1,        # flow + range navigation, no GPS needed to arm
}


def build(airframe: Path, board: Path | None, mount_deg: float, out: Path) -> None:
    af = Airframe.load(airframe)
    fs = dict(af.design.get("flow_sensor") or {})
    fs.update(enabled=True, mount="structure", extra_pitch_deg=float(mount_deg))
    af.design["flow_sensor"] = fs
    af.resolve_mass()
    p = af.px4_params(hitl=False)
    p = {k: v for k, v in p.items() if not k.startswith(("HIL_ACT_FUNC", "PWM_MAIN_FUNC", "PWM_AUX_FUNC"))}
    p.update(REAL_FLIGHT)
    meta = json.load(open(Path.home() / "PX4-Autopilot/build/px4_sitl_default/parameters.json"))
    types = {g["name"]: g.get("type") for g in meta.get("parameters", [])}
    cur = {}
    if board:
        cur = {k: v["value"] for k, v in json.load(open(board))["params"].items()}
    missing = sorted(k for k in p if cur and k not in cur)
    ts = datetime.now().astimezone().isoformat(timespec="seconds")
    lines = ["# Onboard parameters for QGroundControl (Vehicle-Id Component-Id Name Value Type)",
             f"# REAL-FLIGHT set for {af.name} from {airframe.relative_to(ROOT)}, generated {ts}",
             f"# hover {af.hover_pitch_deg} deg (SENS_BOARD_Y_OFF), parked {af.landed_pitch_deg} deg, H-FLOW fixed {mount_deg} deg to the frame",
             "# Review the .md next to this file before loading. Output functions and calibrations are not included."]
    for k in sorted(p):
        v = p[k]
        is_int = (types.get(k) == "Int32") or k in INT_NL or (types.get(k) is None and isinstance(v, int))
        lines.append(f"1\t1\t{k}\t{int(round(float(v)))}\t6" if is_int else f"1\t1\t{k}\t{float(v):.6f}\t9")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines) + "\n")
    changes = []
    for k in sorted(p):
        if k in cur and (cur[k] is None or abs(float(cur[k]) - float(p[k])) > 1e-4 * max(1.0, abs(float(p[k])))):
            changes.append((k, cur[k], p[k]))
    rng = math.degrees(af.px4_params(hitl=False).get("EKF2_RNG_PITCH", 0.0))
    (out.with_suffix(".diff.json")).write_text(json.dumps({"generated": ts, "airframe": str(airframe.relative_to(ROOT)),
        "board_dump": str(board.relative_to(ROOT)) if board else None, "count": len(p), "changes": changes,
        "not_on_board": missing, "rng_pitch_deg": round(rng, 2)}, indent=1, default=float))
    print(f"{out.relative_to(ROOT)}: {len(p)} params, {len(changes)} differ from the board, not on board now: {missing}")
    for k, a, b in changes:
        print(f"  {k:20s} {a!s:>14} -> {b}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--airframe", default=str(ROOT / "airframes/atlas_v3_30_cfd.json"))
    ap.add_argument("--board", default=None)
    ap.add_argument("--flow-mount-deg", type=float, default=20.3)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    build(Path(a.airframe).resolve(), Path(a.board).resolve() if a.board else None, a.flow_mount_deg, Path(a.out).resolve())
