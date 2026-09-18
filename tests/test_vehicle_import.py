"""Generating an airframe from the CAD-derived vehicle documents.

Most of what can go wrong here is a sign or a convention, and every one of them is silent: a mirrored product of
inertia still integrates, a jet turned the wrong way still produces thrust, a reaction torque with the wrong sign
still balances a different aircraft. So these tests check conventions against hand-computed values rather than
against whatever the code currently returns.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

from airframe_designer.geometry.airframe import Airframe
from airframe_designer.vehicle import airframe_from_cad, load_vehicle

REAL_MANIFEST = Path(__file__).resolve().parents[2] / "mujoco-vibe" / "vehicles" / "atlas-phase01" / "vehicle-manifest.json"


def _provenance(kind="cad-derived"):
    return {"source": "test fixture", "source_kind": kind}


def _write_vehicle(root: Path, *, tensor=None, component_masses=None) -> Path:
    """A mirror-symmetric four-fan fixture: a foil fan each side blowing aft, and a front pair tilted in
    opposition. Symmetry matters, because only a mirrored aircraft has an attitude at which pitch alone stands the
    thrust vector upright: one tilted fan on its own leaves a side force no pitch can remove."""
    root.mkdir(parents=True, exist_ok=True)
    (root / "source").mkdir(exist_ok=True)
    (root / "source" / "cad_geometry.json").write_text(json.dumps(
        {"bounding_box_m": {"min": [-0.5, -0.4, -0.1], "max": [0.7, 0.4, 0.4]}}))
    (root / "edf-map.json").write_text(json.dumps({
        "edfs": [
            {"id": "FOIL_L", "px4_actuator_index": 0,
             "position": {"value": [-0.4, -0.3, 0.35], "provenance": _provenance()},
             "reference_thrust_axis": {"value": [1.0, 0.0, 0.0], "provenance": _provenance()},
             "rotation_direction": {"sense": "cw"}, "thrust_map_ref": "map",
             "duct": {"radius": {"value": 0.05, "provenance": _provenance()}}},
            {"id": "FOIL_R", "px4_actuator_index": 1,
             "position": {"value": [-0.38, 0.3, 0.32], "provenance": _provenance()},
             "reference_thrust_axis": {"value": [1.0, 0.0, 0.0], "provenance": _provenance()},
             "rotation_direction": {"sense": "cw"}, "thrust_map_ref": "map",
             "duct": {"radius": {"value": 0.05, "provenance": _provenance()}}},
            {"id": "FRONT", "px4_actuator_index": 2,
             "position": {"value": [0.35, 0.0, 0.05], "provenance": _provenance()},
             "reference_thrust_axis": {"value": [0.0, 0.0, -1.0], "provenance": _provenance()},
             "rotation_direction": {"sense": "ccw"}, "thrust_map_ref": "map",
             "duct": {"radius": {"value": 0.05, "provenance": _provenance()}}},
            {"id": "FRONT_MIRROR", "px4_actuator_index": 3,
             "position": {"value": [0.45, 0.0, 0.05], "provenance": _provenance()},
             "reference_thrust_axis": {"value": [0.0, 0.0, -1.0], "provenance": _provenance()},
             "rotation_direction": {"sense": "ccw"}, "thrust_map_ref": "map",
             "duct": {"radius": {"value": 0.05, "provenance": _provenance()}}},
        ],
        "thrust_maps": {"map": {"points": [{"command": 0.0, "thrust_N": 0.0}, {"command": 1.0, "thrust_N": 30.0}],
                                "provenance": {"source_kind": "manufacturer"}, "blocker_ref": "THRUST-MAP"}},
    }))
    (root / "geometry.json").write_text(json.dumps({
        "foil_exit_angles_deg": {"FOIL_L": 60.0, "FOIL_R": 60.0},
        "front_motor_tilts_deg": {"FRONT": 30.0, "FRONT_MIRROR": -30.0}}))
    manifest = {
        "vehicle_id": "fixture-01", "display_name": "Fixture", "revision": "r1",
        "classification": {"declared_state": "provisional"},
        "edf_map_ref": "edf-map.json",
        "test_configuration": {"as_built_geometry_ref": "geometry.json"},
        "mass_properties": {
            "total_mass": {"value": 10.0, "provenance": _provenance("measured")},
            "centre_of_gravity": {"value": [-0.1, 0.0, 0.13], "provenance": _provenance()},
            "inertia": {"available": True, "provenance": _provenance(),
                        "tensor": tensor or [[0.6, -0.002, 0.05], [-0.002, 0.7, 0.003], [0.05, 0.003, 1.0]]},
            "component_masses": component_masses or [],
        },
        "foils": [{"id": "foil_left", "served_edfs": ["FOIL_L"]},
                  {"id": "foil_right", "served_edfs": ["FOIL_R"]}],
        "blockers": [{"id": "THRUST-MAP", "summary": "no measured thrust curve"}],
    }
    path = root / "vehicle-manifest.json"
    path.write_text(json.dumps(manifest))
    return path


def test_inertia_tensor_survives_the_product_of_inertia_convention(tmp_path):
    """The manifest carries an inertia tensor; the airframe stores products of inertia, which are its negated
    off-diagonals. Round-tripping through Airframe must reproduce the tensor exactly."""
    tensor = [[0.6, -0.002, 0.05], [-0.002, 0.7, 0.003], [0.05, 0.003, 1.0]]
    d, _ = airframe_from_cad(load_vehicle(_write_vehicle(tmp_path / "v", tensor=tensor)))
    af = Airframe.from_dict(d)
    assert np.allclose(af.mass.tensor(), np.asarray(tensor)), "tensor changed on the way through"
    assert af.mass.inertia_products[1] == pytest.approx(-0.05)      # Ixz = -tensor[0][2]


def test_foil_turns_the_jet_towards_lift(tmp_path):
    """A fan blowing aft reacts along +x. A foil turning the jet by 60 degrees must rotate that reaction towards
    -z (up), not towards +z, and the duct axis must be kept so the turning loss knows the angle."""
    d, _ = airframe_from_cad(load_vehicle(_write_vehicle(tmp_path / "v")))
    foil = d["rotors"][0]
    assert foil["duct_axis"] == pytest.approx([1.0, 0.0, 0.0])
    assert foil["axis"] == pytest.approx([math.cos(math.radians(60)), 0.0, -math.sin(math.radians(60))], abs=1e-6)
    assert Airframe.from_dict(d).rotors[0].deflection_deg() == pytest.approx(60.0, abs=1e-3)


def test_front_tilt_is_positive_towards_starboard(tmp_path):
    """The EDF map declares positive tilt as rotating the thrust axis towards body +y. A fan pushing up ([0,0,-1])
    tilted +30 degrees must gain a +y component, which is what makes the two front fans differential in yaw."""
    d, _ = airframe_from_cad(load_vehicle(_write_vehicle(tmp_path / "v")))
    front = d["rotors"][2]
    assert front["duct_axis"] is None
    assert front["axis"] == pytest.approx([0.0, math.sin(math.radians(30)), -math.cos(math.radians(30))], abs=1e-6)


def test_km_sign_follows_the_rotation_sense(tmp_path):
    """CA_ROTORn_KM is positive for a counter-clockwise rotor and negative for a clockwise one. Inverting this
    inverts yaw, which diverges rather than failing visibly."""
    d, _ = airframe_from_cad(load_vehicle(_write_vehicle(tmp_path / "v")), km_magnitude=0.01)
    assert d["rotors"][0]["km"] == pytest.approx(-0.01)   # cw
    assert d["rotors"][2]["km"] == pytest.approx(+0.01)   # ccw


def test_hover_pitch_makes_the_thrust_axis_vertical(tmp_path):
    """The generated hover pitch must be the attitude at which the thrust-weighted mean axis points straight up,
    because PX4 is told to treat that attitude as level."""
    d, _ = airframe_from_cad(load_vehicle(_write_vehicle(tmp_path / "v")))
    total = np.zeros(3)
    for r in Airframe.from_dict(d).active_rotors():
        total += np.asarray(r.axis) * r.effective_max_thrust()
    p = math.radians(d["hover_pitch_deg"])
    world_up_in_body = np.array([math.sin(p), 0.0, -math.cos(p)])
    total /= np.linalg.norm(total)
    assert float(np.dot(total, world_up_in_body)) == pytest.approx(1.0, abs=1e-6)


def test_symmetry_correction_mirrors_one_side_onto_the_other(tmp_path):
    """The two foil fans in the fixture share a spanwise station but differ in x and z, the way the real aircraft's
    do because its two foil components are not identical. Symmetrising must copy the donor's x and z across while
    leaving each fan's own y alone, and must say what it moved."""
    path = _write_vehicle(tmp_path / "v")
    plain, _ = airframe_from_cad(load_vehicle(path))
    left, right = plain["rotors"][0], plain["rotors"][1]
    assert left["pos"][0] != right["pos"][0] and left["pos"][2] != right["pos"][2]

    fixed, side = airframe_from_cad(load_vehicle(path), symmetrise="right")
    l, r = fixed["rotors"][0], fixed["rotors"][1]
    assert l["pos"][0] == pytest.approx(r["pos"][0])
    assert l["pos"][2] == pytest.approx(r["pos"][2])
    assert l["pos"][1] == pytest.approx(-0.3), "the spanwise station must not be mirrored away"
    assert side["symmetry_correction"]["donor_side"] == "right"
    assert side["symmetry_correction"]["moved"][0]["rotor"] == "FOIL_L"


def test_symmetry_correction_is_off_by_default(tmp_path):
    """Following the documents is the default. A silent correction would make the airframe disagree with the CAD
    without saying so."""
    d, side = airframe_from_cad(load_vehicle(_write_vehicle(tmp_path / "v")))
    assert side["symmetry_correction"] is None
    assert d["rotors"][0]["pos"][0] != d["rotors"][1]["pos"][0]


def test_unweighed_flight_parts_are_reported_but_ground_equipment_is_not(tmp_path):
    """An unweighed flight part means the mass and CG are both short by its share, and must be visible. A part
    deliberately left off the aircraft is not a shortfall. Both notes contain the word "excluded"."""
    components = [
        {"name": "weighed", "mass_kg": 1.0, "source_kind": "measured"},
        {"name": "Body39", "mass_kg": None, "note": "not weighed; excluded from the total"},
        {"name": "PID RIG MOUNT:1", "mass_kg": None, "note": "excluded: Ground test rig, not flight hardware."},
    ]
    v = load_vehicle(_write_vehicle(tmp_path / "v", component_masses=components))
    assert v.unweighed == ["Body39"]


def test_provenance_marks_the_values_the_documents_do_not_carry(tmp_path):
    """Every value chosen so the model would run must come back as ``assumed`` against a blocker. If this stops
    holding, a generated airframe stops being distinguishable from a measured one."""
    _, side = airframe_from_cad(load_vehicle(_write_vehicle(tmp_path / "v")))
    fields = side["rotors"][0]["fields"]
    assert fields["max_thrust"]["source_kind"] == "manufacturer"
    for name in ("km", "tau", "turn_loss"):
        assert fields[name]["source_kind"] == "assumed", name
        assert fields[name]["blocker"], f"{name} is assumed but names no blocker"
    assert side["body"]["drag_quadratic"]["source_kind"] == "assumed"


@pytest.mark.skipif(not REAL_MANIFEST.is_file(), reason="the Atlas CAD documents are not checked out beside this repo")
def test_real_atlas_generates_a_valid_airframe():
    """The real aircraft: ten fans, the weighed mass, and an airframe the rest of the project accepts."""
    d, side = airframe_from_cad(load_vehicle(REAL_MANIFEST))
    af = Airframe.from_dict(d)
    assert len(af.rotors) == 10
    assert af.mass.mass == pytest.approx(11.191, abs=1e-3)
    assert np.linalg.eigvalsh(af.mass.tensor()).min() > 0, "inertia tensor is not positive definite"
    assert all(r.km < 0 for r in af.rotors), "every fan turns clockwise, so every km is negative"
    # Eight fans are turned by a foil; the two at the nose are not.
    assert sum(1 for r in af.rotors if r.duct_axis) == 8
    # Derived independently of atlas-sim, which arrived at 26 degrees for the same aircraft.
    assert af.hover_pitch_deg == pytest.approx(26.6, abs=1.0)
    assert side["mass"]["unweighed_parts"], "the unweighed parts must stay visible"
