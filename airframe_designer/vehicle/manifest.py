"""Read the CAD-derived vehicle documents, keeping every value attached to where it came from.

The documents are a vehicle manifest (mass properties, foils, sensors, blockers), an EDF map (one entry per fan:
position, thrust axis, rotation sense, duct, thrust map) and an applied geometry configuration (foil exit angles
and front-motor tilts as built). They are written by the CAD import, not by hand.

Every physical quantity in them is wrapped as ``{"value": ..., "provenance": {...}}``. This module keeps that
wrapper rather than unwrapping it, because the difference between a number that was weighed and a number that was
assumed is the whole difference between "the simulation says it hovers" and "the aircraft hovers". A value with
``source_kind`` of ``measured`` or ``cad-derived`` was established; ``manufacturer`` was read off a datasheet;
``assumed`` was chosen so the model would run. Only the first two support a claim about the real aircraft.

Nothing here invents a default. A quantity the documents do not carry comes back absent, and it is the caller's
job to decide what to do about that in the open.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

# Ordered best to worst. A conclusion is only as good as the weakest input it rests on.
EVIDENCE_ORDER = ["measured", "cad-derived", "manufacturer", "assumed", "absent"]


@dataclass(frozen=True)
class Sourced:
    """A value together with the evidence behind it."""
    value: Any
    source_kind: str = "absent"
    source: str = ""
    note: str = ""
    blocker: str | None = None

    @property
    def present(self) -> bool:
        return self.value is not None

    @property
    def established(self) -> bool:
        """True when the value was measured or taken from CAD, i.e. it describes this aircraft."""
        return self.source_kind in ("measured", "cad-derived")

    def to_dict(self) -> dict:
        d = {"value": self.value, "source_kind": self.source_kind}
        if self.source:
            d["source"] = self.source
        if self.note:
            d["note"] = self.note
        if self.blocker:
            d["blocker"] = self.blocker
        return d


def sourced(node: Any, *, blocker: str | None = None, assumed: Any = None, why: str = "") -> Sourced:
    """Wrap a document node. ``assumed`` is the fallback used when the document carries nothing, and saying so is
    the point: it comes back marked ``assumed`` with ``why`` recorded, never silently as a number."""
    if isinstance(node, dict) and "value" in node and node["value"] is not None:
        p = node.get("provenance") or {}
        return Sourced(node["value"], str(p.get("source_kind", "assumed")), str(p.get("source", "")),
                       str(p.get("note", "")), node.get("blocker_ref") or p.get("blocker_ref") or blocker)
    if node is not None and not isinstance(node, dict):
        return Sourced(node, "cad-derived")
    return Sourced(assumed, "assumed" if assumed is not None else "absent", note=why, blocker=blocker)


@dataclass
class CadVehicle:
    """The aircraft as the CAD import and the weighing describe it."""
    vehicle_id: str
    display_name: str
    revision: str
    declared_state: str                       # validated-real | provisional | synthetic-fixture | invalid
    root: Path
    mass: Sourced                             # kg
    cg: Sourced                               # [x, y, z] m, body FRD, from the assembly origin
    inertia_tensor: Sourced                   # 3x3 kg.m2 about the CG, body FRD
    bbox: dict                                # {"min": [...], "max": [...]} m
    edfs: list[dict] = field(default_factory=list)          # raw EDF map entries, actuator order
    foils: list[dict] = field(default_factory=list)
    thrust_maps: dict = field(default_factory=dict)
    geometry: dict = field(default_factory=dict)            # applied foil exit angles and front-motor tilts
    blockers: list[dict] = field(default_factory=list)
    component_masses: list[dict] = field(default_factory=list)
    bodies: dict[str, list[float]] = field(default_factory=dict)   # CAD body name -> centroid, body FRD m

    # ------------------------------------------------------------------ derived
    @property
    def unweighed(self) -> list[str]:
        """Flight parts present in the CAD that nobody has weighed, so the mass and CG are both short by their share.

        Deliberate exclusions are not shortfalls: ground-support hardware is left out of the aircraft on purpose,
        and is recognised by its note saying so rather than by the word "excluded", which the unweighed parts also
        carry ("not weighed; excluded from the total").
        """
        def ground_equipment(c: dict) -> bool:
            note = str(c.get("note", "")).lower()
            return "not flight hardware" in note or "ground test rig" in note
        return [c["name"] for c in self.component_masses
                if c.get("mass_kg") is None and not ground_equipment(c)]

    def blocker(self, blocker_id: str) -> dict | None:
        return next((b for b in self.blockers if b.get("id") == blocker_id), None)

    def foil_of(self, edf_id: str) -> str | None:
        return next((f["id"] for f in self.foils if edf_id in (f.get("served_edfs") or [])), None)

    def exit_angle_deg(self, edf_id: str) -> Sourced:
        """How far the foil turns this fan's jet, as built. Geometry, not a force model: what the turning does to
        the thrust is blocked on ATLAS-AERO-FOIL-DEFLECTION."""
        a = (self.geometry.get("foil_exit_angles_deg") or {}).get(edf_id)
        if a is None:
            return Sourced(None, "absent", note=f"no exit angle recorded for {edf_id}")
        return Sourced(float(a), "cad-derived", source=str(self.geometry.get("_path", "applied geometry")),
                       note="jet turning angle of the foil channel at this fan's station")

    def front_tilt_deg(self, edf_id: str) -> Sourced:
        t = (self.geometry.get("front_motor_tilts_deg") or {}).get(edf_id)
        if t is None:
            return Sourced(None, "absent")
        return Sourced(float(t), "cad-derived", source=str(self.geometry.get("_path", "applied geometry")),
                       note="sideways tilt applied to a front fan, positive towards body +y", blocker="ATLAS-TILT-LIMITS")


def load_vehicle(manifest_path: str | Path, geometry_path: str | Path | None = None) -> CadVehicle:
    """Load a vehicle manifest with its EDF map and an applied geometry configuration.

    ``geometry_path`` defaults to the manifest's own ``test_configuration.as_built_geometry_ref``: the configuration
    the aircraft is actually built in. Passing a different one is how you ask what a rebuild would do.
    """
    mpath = Path(manifest_path)
    root = mpath.parent
    man = json.loads(mpath.read_text())

    edf_path = root / str(man.get("edf_map_ref") or "edf-map.json")
    edf_map = json.loads(edf_path.read_text())
    edfs = sorted(edf_map.get("edfs") or [], key=lambda d: d.get("px4_actuator_index", 0))

    if geometry_path is None:
        ref = (man.get("test_configuration") or {}).get("as_built_geometry_ref")
        geometry_path = root / str(ref) if ref else None
    geometry: dict = {}
    if geometry_path:
        geometry = json.loads(Path(geometry_path).read_text())
        geometry["_path"] = str(Path(geometry_path).name)

    mp = man.get("mass_properties") or {}
    inertia = mp.get("inertia") or {}
    tensor = sourced({"value": inertia.get("tensor"), "provenance": inertia.get("provenance")}) \
        if inertia.get("available") else Sourced(None, "absent", note="no inertia tensor in the manifest")

    bbox = {"min": [0.0, 0.0, 0.0], "max": [0.0, 0.0, 0.0]}
    bodies: dict[str, list[float]] = {}
    cad_path = root / "source" / "cad_geometry.json"
    if cad_path.is_file():
        cad = json.loads(cad_path.read_text())
        bbox = cad.get("bounding_box_m") or bbox
        bodies = {b["name"]: b["centroid_frd_m"] for b in cad.get("bodies") or [] if "centroid_frd_m" in b}

    return CadVehicle(
        vehicle_id=str(man.get("vehicle_id", "vehicle")),
        display_name=str(man.get("display_name", man.get("vehicle_id", "vehicle"))),
        revision=str(man.get("revision", "")),
        declared_state=str((man.get("classification") or {}).get("declared_state", "invalid")),
        root=root,
        mass=sourced(mp.get("total_mass"), blocker="ATLAS-MASS-INCOMPLETE"),
        cg=sourced(mp.get("centre_of_gravity"), blocker="ATLAS-MASS-INCOMPLETE"),
        inertia_tensor=tensor,
        bbox=bbox,
        edfs=edfs,
        foils=man.get("foils") or [],
        thrust_maps=edf_map.get("thrust_maps") or {},
        geometry=geometry,
        blockers=man.get("blockers") or [],
        component_masses=mp.get("component_masses") or [],
        bodies=bodies,
    )
