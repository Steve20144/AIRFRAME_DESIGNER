"""Design knobs: the handful of geometry choices ATLAS keeps coming back to, as named values that move the rotors
and the CAD parts together, so the Geometry tab, the Tuning tab's sweeps and the studies all edit the same thing.

Stored in ``design.knobs`` (a list) and ``design.cad_links`` (a dict):

  {"name": "foil_1_deg", "label": "Jetfoil 1 (outer)", "kind": "tilt", "rotors": ["M1", "M2"], "range": [15, 50]}
  {"name": "front_cant_deg", "label": "Front jets sideways", "kind": "cant", "rotors": ["M9", "M10"],
   "signs": [1, -1], "range": [0, 40]}
  {"name": "cg_dx", "label": "Battery fore/aft", "kind": "shift_x", "bodies": ["12:BATTER PACK", ...], "value": 0.0,
   "range": [-0.08, 0.08]}

Kinds:
  tilt     forward lean of the thrust axis from vertical, deg; the value IS the rotors' tilt (read back from them),
           so editing a rotor by hand and turning the knob never disagree. For a foil fan this is the jet angle the
           foil gives (90 minus the foil's deflection).
  cant     sideways lean, deg, as a magnitude; ``signs`` says which way each rotor leans (kept so 0 is reversible).
  shift_x  moves the listed CAD bodies (the battery) fore/aft by ``value`` metres, which moves the CG the way the
  shift_z  real aircraft would; with no CAD bodies it shifts mass.cg itself from ``base_cg``.

``design.cad_links`` ties CAD bodies to a rotor: {"M9": {"bodies": [...], "pos0": [...], "axis0": [...]}} where
pos0 / axis0 are the rotor as the CAD draws it. On every resolve the linked bodies are turned by the rotation that
takes axis0 to the rotor's axis, about pos0, and carried to the rotor's position, so the part (and its mass) follows
whatever changed the rotor. A foil's own surface is not linked by default: a new deflection angle is a new foil
shape, not the old part turned.
"""
from __future__ import annotations

from typing import Any

import numpy as np

SHIFT_AXES = {"shift_x": 0, "shift_y": 1, "shift_z": 2}


def knobs(af) -> list[dict]:
    return list((af.design or {}).get("knobs") or [])


def find(af, name: str) -> dict:
    for k in knobs(af):
        if k.get("name") == name:
            return k
    raise KeyError(f"no design knob '{name}' (have: {', '.join(k.get('name', '?') for k in knobs(af)) or 'none'})")


def _rotors(af, names) -> list:
    by_name = {r.name: r for r in af.rotors}
    return [by_name[n] for n in names or [] if n in by_name]


def get_value(af, name: str) -> float:
    k = find(af, name)
    kind = k.get("kind")
    if kind in ("tilt", "cant"):
        rs = _rotors(af, k.get("rotors"))
        if not rs:
            return float("nan")
        return round(rs[0].tilt_deg, 3) if kind == "tilt" else round(abs(rs[0].cant_deg), 3)
    return float(k.get("value", 0.0) or 0.0)


def set_value(af, name: str, value: float) -> None:
    """Write a knob (in place). Rotor knobs change the rotors; shift knobs store the value, applied by update_poses."""
    k = find(af, name)
    kind, value = k.get("kind"), float(value)
    if kind == "tilt":
        for r in _rotors(af, k.get("rotors")):
            r.tilt_deg = value
    elif kind == "cant":
        rs = _rotors(af, k.get("rotors"))
        signs = list(k.get("signs") or [])
        signs += [1.0] * (len(rs) - len(signs))
        for r, s in zip(rs, signs):
            r.cant_deg = value * (1.0 if s >= 0 else -1.0)
    elif kind in SHIFT_AXES:
        k["value"] = value
        if not (af.cad and k.get("bodies")):         # no CAD parts to move: shift the CG itself
            base = k.setdefault("base_cg", list(af.mass.cg))
            cg = list(base)
            cg[SHIFT_AXES[kind]] = base[SHIFT_AXES[kind]] + value
            af.mass.cg = [round(float(v), 5) for v in cg]
    else:
        raise ValueError(f"knob '{name}' has unknown kind '{kind}'")


def describe(af) -> list[dict]:
    """The knobs with their current values, for the UI."""
    out = []
    for k in knobs(af):
        d = {key: k.get(key) for key in ("name", "label", "kind", "rotors", "bodies", "range", "signs")}
        d["value"] = get_value(af, k["name"])
        d["path"] = f"knobs.{k['name']}"
        out.append(d)
    return out


# ------------------------------------------------------------------ CAD poses
def _rotation_between(a0, a1) -> np.ndarray:
    """Smallest rotation taking unit vector a0 onto a1 (Rodrigues)."""
    a0 = np.asarray(a0, float); a0 = a0 / (np.linalg.norm(a0) or 1.0)
    a1 = np.asarray(a1, float); a1 = a1 / (np.linalg.norm(a1) or 1.0)
    v = np.cross(a0, a1)
    c = float(np.dot(a0, a1))
    s = float(np.linalg.norm(v))
    if s < 1e-12:
        if c > 0:
            return np.eye(3)
        # opposite: turn 180 deg about any axis normal to a0
        n = np.cross(a0, [1.0, 0.0, 0.0] if abs(a0[0]) < 0.9 else [0.0, 1.0, 0.0]); n /= np.linalg.norm(n)
        return 2.0 * np.outer(n, n) - np.eye(3)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * ((1.0 - c) / (s * s))


def update_poses(af) -> None:
    """Derive every CAD body's knob pose from the rotor links and the shift knobs (idempotent)."""
    if not af.cad:
        return
    design = af.design or {}
    links = design.get("cad_links") or {}
    shift_knobs = [k for k in design.get("knobs") or [] if k.get("kind") in SHIFT_AXES and k.get("bodies")]
    if not links and not shift_knobs and all(b.rot[0] == 1.0 and not any(b.shift) for b in af.cad.bodies):
        return
    by_id = {b.id: b for b in af.cad.bodies}
    for b in af.cad.bodies:
        b.reset_pose()
    rotors = {r.name: r for r in af.rotors}
    for rname, link in links.items():
        r = rotors.get(rname)
        if r is None:
            continue
        pos0 = np.asarray(link.get("pos0") or r.pos, float)
        R = _rotation_between(link.get("axis0") or r.axis, r.axis)
        for bid in link.get("bodies") or []:
            b = by_id.get(bid)
            if b is None:
                continue
            b.rot = [round(float(v), 9) for v in R.ravel()]
            b.pivot = [float(v) for v in pos0]
            b.shift = [float(v) for v in np.asarray(r.pos, float) - pos0]
    for k in shift_knobs:
        ax, v = SHIFT_AXES[k["kind"]], float(k.get("value", 0.0) or 0.0)
        for bid in k["bodies"]:
            b = by_id.get(bid)
            if b is not None:
                b.shift[ax] += v


def link(af, rotor: str, bodies: list[str]) -> None:
    """Tie CAD bodies to a rotor as it stands now (the CAD pose). Bodies are taken off any other rotor first."""
    design = af.design if af.design is not None else {}
    links = design.setdefault("cad_links", {})
    for other in links.values():
        other["bodies"] = [b for b in other.get("bodies") or [] if b not in bodies]
    r = next((x for x in af.rotors if x.name == rotor), None)
    if r is None:
        raise KeyError(f"no rotor named '{rotor}'")
    cur = links.get(rotor)
    if cur and cur.get("bodies"):
        cur["bodies"] = list(dict.fromkeys(list(cur["bodies"]) + list(bodies)))
    else:
        links[rotor] = {"bodies": list(bodies), "pos0": list(r.pos), "axis0": list(r.axis)}
    for name in [n for n, l in links.items() if not l.get("bodies")]:
        del links[name]


def unlink(af, bodies: list[str]) -> None:
    links = (af.design or {}).get("cad_links") or {}
    for l in links.values():
        l["bodies"] = [b for b in l.get("bodies") or [] if b not in bodies]
    for name in [n for n, l in links.items() if not l.get("bodies")]:
        del links[name]


def set_ballast(af, bodies: list[str]) -> None:
    """The CAD bodies the CG knobs move (normally the battery packs)."""
    for k in knobs(af):
        if k.get("kind") in SHIFT_AXES:
            k["bodies"] = list(bodies)


# ------------------------------------------------------------------ defaults
def _pairs(rotors) -> list[list]:
    """Mirror pairs (same |y| within 1 cm), outermost first; a rotor with no mirror stands alone."""
    left = [r for r in rotors if r.pos[1] < -1e-3]
    rest = [r for r in rotors if r not in left]
    out = []
    for l in left:
        m = min(rest, key=lambda r: abs(r.pos[1] + l.pos[1]) + abs(r.pos[0] - l.pos[0]), default=None)
        if m is not None and abs(m.pos[1] + l.pos[1]) < 0.01:
            rest.remove(m); out.append([l, m])
        else:
            out.append([l])
    out += [[r] for r in rest]
    return sorted(out, key=lambda p: -abs(p[0].pos[1]))


def setup_defaults(af, link_front_parts: bool = True) -> list[dict]:
    """Knobs for an ATLAS-layout airframe: one per jetfoil station (mirror pair of fans with a duct axis), the
    front jets' sideways and fore-aft tilt, and the battery fore/aft and up/down. Replaces existing knobs. The
    front fans' CAD bodies (nearest body within 0.6 fan diameters) are linked so they turn with the fans; the
    battery is every CAD body with BATT in its name and a mass."""
    active = [r for r in af.rotors if r.enabled]
    foil = [r for r in active if r.duct_axis]
    front = [r for r in active if not r.duct_axis]
    ks: list[dict] = []
    for i, pair in enumerate(_pairs(foil), 1):
        where = "outer" if i == 1 else ("inner" if i == len(_pairs(foil)) else "")
        ks.append({"name": f"foil_{i}_deg", "label": f"Jetfoil {i}{' (' + where + ')' if where else ''} jet angle",
                   "kind": "tilt", "rotors": [r.name for r in pair], "range": [15.0, 55.0]})
    if front:
        signs = [1.0 if r.cant_deg >= 0 else -1.0 for r in front]
        ks.append({"name": "front_cant_deg", "label": "Front jets sideways tilt", "kind": "cant",
                   "rotors": [r.name for r in front], "signs": signs, "range": [0.0, 40.0]})
        ks.append({"name": "front_tilt_deg", "label": "Front jets fore/aft tilt", "kind": "tilt",
                   "rotors": [r.name for r in front], "range": [-20.0, 20.0]})
    battery = [b.id for b in (af.cad.active() if af.cad else []) if "BATT" in b.name.upper() and b.mass > 0]
    for name, kind, label in (("cg_dx", "shift_x", "Battery fore/aft (moves CG), m"),
                              ("cg_dz", "shift_z", "Battery down/up (moves CG), m")):
        ks.append({"name": name, "label": label, "kind": kind, "bodies": battery, "value": 0.0,
                   "range": [-0.08, 0.08] if kind == "shift_x" else [-0.04, 0.04]})
    if af.design is None:
        af.design = {}
    af.design["knobs"] = ks
    if link_front_parts and af.cad:
        af.design.pop("cad_links", None)
        for r in front:
            near = _nearest_bodies(af, r, 0.6 * (r.diameter or 0.08))
            if near:
                link(af, r.name, near)
    return describe(af)


def _nearest_bodies(af, rotor, radius: float) -> list[str]:
    """Bodies within ``radius`` of the rotor whose closest rotor is this one."""
    out = []
    for b in af.cad.active():
        p = np.asarray(af.cad.body_pos(b), float)
        d = float(np.linalg.norm(p - np.asarray(rotor.pos, float)))
        if d > radius:
            continue
        closest = min(af.rotors, key=lambda r: float(np.linalg.norm(p - np.asarray(r.pos, float))))
        if closest is rotor:
            out.append(b.id)
    return out


def knob_paths(af) -> list[str]:
    return [f"knobs.{k['name']}" for k in knobs(af)]


def apply_values(af, values: dict[str, Any]) -> None:
    for name, v in values.items():
        set_value(af, name, v)
    af.resolve_mass()
