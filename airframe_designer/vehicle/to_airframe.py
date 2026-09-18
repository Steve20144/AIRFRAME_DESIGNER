"""Turn the CAD-derived vehicle documents into a schema-2 airframe, and record what the result rests on.

Two outputs, always together:

* the **airframe**, a plain schema-2 file the rest of the project flies, edits and optimises. It carries no
  provenance, because ``Airframe.to_dict`` writes a fixed set of keys and anything else would be dropped the
  first time somebody saved from the UI. A number that survives only until the next save is worse than no number.
* the **provenance sidecar**, one entry per generated field saying where the value came from and which blocker
  governs it. This is what makes a later claim about the real aircraft checkable, and it is why the conversion
  lives here rather than in a throwaway script.

The geometry of this aircraft is unusual enough to be worth stating. Eight of the ten fans are ducted units whose
efflux leaves towards the tail, so their undeflected reaction pushes the airframe *forward*; a foil then turns
each jet, and the reaction turns with it. That is what ``Rotor.duct_axis`` and ``Rotor.turn_loss`` describe: the
duct axis is where the fan blows, ``axis`` is where the turned jet pushes, and the loss is the thrust the turn
costs. The remaining two fans sit at the nose, push straight up, and are tilted sideways in opposition so their
difference makes yaw. All ten turn the same way, so their reaction torques add instead of cancelling, and holding
heading is a real question rather than a formality.
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np

from ..geometry.gear import generate_legs
from .manifest import CadVehicle, Sourced

AIR_DENSITY = 1.225


def _rot_y(deg: float) -> np.ndarray:
    """Right-handed rotation about body +y. Applied to +x it sweeps towards -z, which is the direction a foil
    turns a rearward-blowing jet to make lift."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_x(deg: float) -> np.ndarray:
    """Right-handed rotation about body +x: positive tilt carries a thrust axis towards body +y, which is the
    sign convention the EDF map declares for the front-motor tilt."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _unit(v) -> list[float]:
    a = np.asarray(v, float)
    n = float(np.linalg.norm(a))
    return (a / n).tolist() if n > 1e-12 else [0.0, 0.0, -1.0]


def _thrust_map_max(vehicle: CadVehicle, ref: str | None) -> Sourced:
    m = vehicle.thrust_maps.get(ref or "") or {}
    points = m.get("points") or []
    if not points:
        return Sourced(None, "absent", note=f"no thrust map '{ref}'")
    peak = max(float(p.get("thrust_N") or 0.0) for p in points)
    p = m.get("provenance") or {}
    return Sourced(peak, str(p.get("source_kind", "assumed")), str(p.get("source", "")), str(p.get("note", "")),
                   m.get("blocker_ref"))


def _rotors(vehicle: CadVehicle, turn_loss: float, km_magnitude: float,
            tau_s: float) -> tuple[list[dict], list[dict]]:
    """One rotor per EDF in PX4 actuator order, with a provenance entry each."""
    rotors: list[dict] = []
    prov: list[dict] = []
    for edf in vehicle.edfs:
        eid = str(edf["id"])
        pos = list((edf.get("position") or {}).get("value") or [0.0, 0.0, 0.0])
        base = np.asarray((edf.get("reference_thrust_axis") or {}).get("value") or [0.0, 0.0, -1.0], float)

        fields: dict[str, Sourced] = {}
        duct_axis: list[float] | None = None
        exit_angle = vehicle.exit_angle_deg(eid)
        tilt = vehicle.front_tilt_deg(eid)

        if vehicle.foil_of(eid) and exit_angle.present:
            # The fan blows along `base`; the foil turns that jet by the exit angle, so the reaction turns with it.
            duct_axis = _unit(base)
            axis = _unit(_rot_y(float(exit_angle.value)) @ base)
            fields["axis"] = Sourced(axis, exit_angle.source_kind,
                                     note=f"duct axis turned {exit_angle.value:g} deg by {vehicle.foil_of(eid)}",
                                     blocker="ATLAS-AERO-FOIL-DEFLECTION")
        elif tilt.present:
            axis = _unit(_rot_x(float(tilt.value)) @ base)
            fields["axis"] = Sourced(axis, tilt.source_kind,
                                     note=f"front fan tilted {tilt.value:g} deg towards body +y", blocker=tilt.blocker)
        else:
            axis = _unit(base)
            fields["axis"] = Sourced(axis, "cad-derived", note="undeflected duct axis")

        sense = str(((edf.get("rotation_direction") or {}).get("sense") or "cw")).lower()
        # CA_ROTORn_KM is positive for a counter-clockwise rotor and negative for a clockwise one.
        km = math.copysign(km_magnitude, 1.0 if sense == "ccw" else -1.0)
        radius = float((((edf.get("duct") or {}).get("radius") or {}).get("value")) or 0.06)
        thrust = _thrust_map_max(vehicle, edf.get("thrust_map_ref"))

        rotors.append({
            "name": eid, "enabled": True, "pos": [round(v, 6) for v in pos],
            "axis": [round(v, 6) for v in axis],
            "km": round(km, 6), "max_thrust": round(float(thrust.value or 0.0), 4),
            "tau": tau_s, "diameter": round(2.0 * radius, 4), "thrust_exponent": 2.0,
            "kind": "ducted", "ram_drag": True,
            "duct_axis": [round(v, 6) for v in duct_axis] if duct_axis else None,
            "turn_loss": turn_loss,
        })
        fields["pos"] = Sourced(pos, ((edf.get("position") or {}).get("provenance") or {}).get("source_kind", "cad-derived"),
                                note="jet exit point on the foil trailing edge" if duct_axis else "fan centre")
        fields["max_thrust"] = thrust
        fields["km"] = Sourced(km, "assumed", note=f"sense '{sense}' is measured; the magnitude is not",
                               blocker="ATLAS-EDF-REACTION-TORQUE")
        fields["tau"] = Sourced(tau_s, "assumed", note="ducted-fan default", blocker="ATLAS-EDF-RESPONSE")
        fields["turn_loss"] = Sourced(turn_loss, "assumed", note="thrust lost at 90 deg of jet turning",
                                      blocker="ATLAS-AERO-FOIL-DEFLECTION")
        fields["diameter"] = Sourced(round(2.0 * radius, 4), "cad-derived", note="duct radius doubled")
        prov.append({"rotor": eid, "fields": {k: v.to_dict() for k, v in fields.items()}})
    return rotors, prov


def _hover_pitch_deg(rotors: list[dict]) -> float:
    """The nose-up attitude at which the combined thrust axis points straight up.

    Every fan is weighted by the thrust it can actually deliver after the turning loss, because a fan whose jet is
    turned further contributes less. PX4 is told to treat this attitude as level, so the allocator sees vertical
    thrust axes and does not have to lean the aircraft to hover.
    """
    total = np.zeros(3)
    for r in rotors:
        axis = np.asarray(r["axis"], float)
        deflection = 0.0
        if r.get("duct_axis"):
            d = np.asarray(r["duct_axis"], float)
            deflection = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(axis, d))))))
        total += axis * r["max_thrust"] * max(0.0, 1.0 - r["turn_loss"] * deflection / 90.0)
    if float(np.linalg.norm(total)) < 1e-9:
        return 0.0
    return math.degrees(math.atan2(float(total[0]), float(-total[2])))


def _body(vehicle: CadVehicle) -> tuple[dict, dict]:
    """A bluff-body drag box from the CAD bounding box.

    There is no aerodynamic dataset for this aircraft (ATLAS-AERO-COEFFICIENTS), so this is not measured drag. It
    is a stated upper bound: a drag coefficient of 1.0 over the full projected area of the bounding box, which a
    largely open carbon truss will not reach. Written as a formula rather than a number so it can be argued with.
    """
    lo = np.asarray(vehicle.bbox["min"], float)
    hi = np.asarray(vehicle.bbox["max"], float)
    size = (hi - lo).tolist()
    area = [size[1] * size[2], size[0] * size[2], size[0] * size[1]]
    quad = [round(0.5 * AIR_DENSITY * 1.0 * a, 4) for a in area]
    centre = ((hi + lo) / 2.0).tolist()
    body = {"size": [round(v, 4) for v in size], "drag_quadratic": quad,
            "drag_angular": [0.02, 0.02, 0.02], "drag_center": [round(v, 4) for v in centre]}
    prov = {
        "size": Sourced(body["size"], "cad-derived", note="CAD bounding box").to_dict(),
        "drag_quadratic": Sourced(quad, "assumed", note="0.5 * rho * Cd * A with Cd = 1.0 over the bounding box; an "
                                  "upper bound on a bluff body, not measured drag",
                                  blocker="ATLAS-AERO-COEFFICIENTS").to_dict(),
        "drag_angular": Sourced(body["drag_angular"], "assumed", note="nominal rotational damping",
                                blocker="ATLAS-AERO-COEFFICIENTS").to_dict(),
    }
    return body, prov


def _legs(vehicle: CadVehicle, landed_pitch_deg: float, clearance_m: float,
          splay_deg: float) -> tuple[list[dict], dict]:
    """A provisional four-leg stand that holds the aircraft at ``landed_pitch_deg``.

    This aircraft's hull has no statically stable resting attitude, so it cannot be stood on the ground without
    something to rest on. The legs are therefore part of the test, not an afterthought, and these are a starting
    point to iterate on rather than a design: the attachment points are taken from the bounding box, not from
    structure anybody has checked can carry the load.

    The height clears the lowest part of the airframe *at the landed attitude*, since pitching the aircraft swings
    the tail down. The footprint puts each foot ``splay_deg`` off vertical as seen from the centre of gravity,
    which is the usual first cut at a tip-over margin.
    """
    cg = np.asarray(vehicle.cg.value or [0.0, 0.0, 0.0], float)
    lo = np.asarray(vehicle.bbox["min"], float)
    hi = np.asarray(vehicle.bbox["max"], float)
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    # "Down" in the landed frame, expressed in structural coordinates: world up is [sin p, 0, -cos p].
    p = math.radians(landed_pitch_deg)
    down = np.array([-math.sin(p), 0.0, math.cos(p)])
    drop = float(np.max((corners - cg) @ down))
    height = round(drop + clearance_m, 4)
    spread = round(min(height * math.tan(math.radians(splay_deg)), float(hi[1]) * 0.9), 4)
    attach_z = round(float(np.max(corners[:, 2]) - cg[2]), 4)

    legs = generate_legs(height=height, spread_x=spread, spread_y=spread, landed_pitch_deg=landed_pitch_deg,
                         attach_z=attach_z, cg=cg.tolist(), mass=float(vehicle.mass.value or 1.0), friction=0.8,
                         foot_radius=0.015)
    prov = {
        "_note": Sourced(None, "assumed",
                         note=f"provisional stand: {height:g} m below the centre of gravity at {landed_pitch_deg:g} deg "
                              f"nose-up, feet {spread:g} m out ({splay_deg:g} deg splay), springs sized for 20 mm of "
                              f"static sink at a damping ratio of 0.8. Attachment points come from the bounding box, "
                              f"not from verified structure.").to_dict()
    }
    return [l.to_dict() for l in legs], prov


def airframe_from_cad(vehicle: CadVehicle, *, name: str | None = None, turn_loss: float = 0.1,
                      km_magnitude: float = 0.01, tau_s: float = 0.12, leg_clearance_m: float = 0.10,
                      leg_splay_deg: float = 30.0) -> tuple[dict[str, Any], dict[str, Any]]:
    """Build (airframe dict, provenance sidecar) from a loaded CAD vehicle.

    The keyword arguments are the values the documents do not carry. They are arguments rather than constants so a
    study can sweep them, and so that changing one is a visible act: every one of them is recorded in the sidecar
    as ``assumed`` against the blocker that would close it.
    """
    rotors, rotor_prov = _rotors(vehicle, turn_loss, km_magnitude, tau_s)
    hover_pitch = round(_hover_pitch_deg(rotors), 3)
    body, body_prov = _body(vehicle)
    # Standing at the hover attitude means the aircraft lifts straight off without rotating on the ground first.
    legs, leg_prov = _legs(vehicle, hover_pitch, leg_clearance_m, leg_splay_deg)

    tensor = vehicle.inertia_tensor.value
    if tensor:
        T = np.asarray(tensor, float)
        inertia = [round(float(T[0, 0]), 6), round(float(T[1, 1]), 6), round(float(T[2, 2]), 6)]
        # The manifest carries the inertia tensor; this project stores products of inertia, which are its negated
        # off-diagonals. Getting this backwards silently mirrors the pitch/yaw coupling of a long, tall airframe.
        products = [round(float(-T[0, 1]), 6), round(float(-T[0, 2]), 6), round(float(-T[1, 2]), 6)]
    else:
        inertia, products = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

    airframe = {
        "schema": 2,
        "name": name or vehicle.vehicle_id.replace("-", "_").upper(),
        "mass": {
            "mass": round(float(vehicle.mass.value or 0.0), 6),
            "cg": [round(float(v), 6) for v in (vehicle.cg.value or [0.0, 0.0, 0.0])],
            "inertia": inertia, "inertia_products": products, "items": [], "from_items": False,
        },
        "body": body,
        "rotors": rotors,
        "wings": [],
        "legs": legs,
        "hover_pitch_deg": hover_pitch,
        "landed_pitch_deg": hover_pitch,
        "px4_overrides": {},
        "design": {},
        "notes": (f"Generated from {vehicle.display_name} revision {vehicle.revision}. Do not edit by hand: "
                  f"regenerate from the CAD documents, or edit a copy. Declared state: {vehicle.declared_state}. "
                  f"See the provenance sidecar for what each value rests on."),
    }

    sidecar = {
        "generated_from": {
            "vehicle_id": vehicle.vehicle_id, "display_name": vehicle.display_name, "revision": vehicle.revision,
            "declared_state": vehicle.declared_state, "manifest_root": str(vehicle.root),
            "applied_geometry": vehicle.geometry.get("_path"),
        },
        "mass": {
            "mass": vehicle.mass.to_dict(), "cg": vehicle.cg.to_dict(),
            "inertia": vehicle.inertia_tensor.to_dict(),
            "unweighed_parts": vehicle.unweighed,
        },
        "body": body_prov,
        "rotors": rotor_prov,
        "legs": leg_prov,
        "derived": {
            "hover_pitch_deg": Sourced(hover_pitch, "assumed",
                                       note="attitude at which the thrust-weighted mean axis is vertical; it moves "
                                            "with the foil exit angles and the turning loss, neither of which is "
                                            "validated", blocker="ATLAS-AERO-FOIL-DEFLECTION").to_dict(),
            "landed_pitch_deg": Sourced(hover_pitch, "assumed",
                                        note="legs chosen to stand the aircraft at its hover attitude, so lift-off "
                                             "needs no rotation on the ground").to_dict(),
            "wings": Sourced([], "absent", note="no lifting surfaces are modelled: the foils are jet-turning "
                                                "channels, and no aerodynamic dataset exists",
                             blocker="ATLAS-AERO-COEFFICIENTS").to_dict(),
        },
        "blockers": vehicle.blockers,
    }
    return airframe, sidecar
