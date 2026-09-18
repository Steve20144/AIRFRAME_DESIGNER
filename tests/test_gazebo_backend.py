"""The Gazebo backend's frame conversions and generated world.

Gazebo is not needed for any of this, and that is the point: the conversions between this project's world NED /
body FRD and Gazebo's world ENU / model FLU are where a silent sign error would live, and they are pure
functions. A mirrored axis produces a flight that looks entirely plausible and is wrong, so these check against
hand-computed values rather than against whatever the code returns today.

The parts that do need a running server (stepping, wrenches, state readback) are not covered here. State readback
is unresolved: see the module docstring.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from airframe_designer.dynamics.gazebo_backend import (
    _frd_flu,
    _inertia_flu,
    _ned_enu,
    _q_from_rotmat,
    _rot_ned_frd,
    generate_world,
)
from airframe_designer.dynamics.quaternion import q_from_euler, q_to_rotmat
from airframe_designer.geometry.airframe import Airframe


def test_frd_flu_flips_only_the_right_and_down_axes():
    """Forward is shared; right becomes left and down becomes up. Applying it twice is the identity."""
    assert _frd_flu([1.0, 2.0, 3.0]) == pytest.approx([1.0, -2.0, -3.0])
    assert _frd_flu(_frd_flu([1.0, 2.0, 3.0])) == pytest.approx([1.0, 2.0, 3.0])


def test_ned_enu_swaps_north_and_east_and_flips_down():
    """North-east-down to east-north-up. Also its own inverse."""
    assert _ned_enu([1.0, 2.0, 3.0]) == pytest.approx([2.0, 1.0, -3.0])
    assert _ned_enu(_ned_enu([1.0, 2.0, 3.0])) == pytest.approx([1.0, 2.0, 3.0])


def test_rotation_round_trips_through_the_gazebo_frames():
    """A body-to-world rotation carried into Gazebo's frames and back must come out unchanged, for an attitude
    with all three angles non-zero so a single mirrored axis cannot hide."""
    R = q_to_rotmat(q_from_euler(math.radians(11.0), math.radians(-23.0), math.radians(47.0)))
    # _rot_ned_frd is its own inverse, because both change-of-basis matrices are.
    assert np.allclose(_rot_ned_frd(_rot_ned_frd(R)), R)


def test_a_vector_rotated_in_either_frame_agrees():
    """The real invariant: rotating a body vector to the world in NED/FRD must equal doing it in ENU/FLU and
    converting the answer. This is what catches a mirrored axis that survives a round trip."""
    R_ned_frd = q_to_rotmat(q_from_euler(math.radians(11.0), math.radians(-23.0), math.radians(47.0)))
    R_enu_flu = _rot_ned_frd(R_ned_frd)
    v_frd = np.array([0.3, -1.7, 2.2])
    direct = R_ned_frd @ v_frd
    via_gazebo = _ned_enu(R_enu_flu @ _frd_flu(v_frd))
    assert direct == pytest.approx(via_gazebo)


def test_inertia_tensor_survives_the_frame_change():
    """Products of inertia about the shared forward axis flip; the one between right and down does not."""
    I = np.array([[0.6, 0.01, 0.05], [0.01, 0.7, 0.02], [0.05, 0.02, 1.0]])
    J = _inertia_flu(I)
    assert np.allclose(np.diag(J), np.diag(I))
    assert J[0, 1] == pytest.approx(-I[0, 1])
    assert J[0, 2] == pytest.approx(-I[0, 2])
    assert J[1, 2] == pytest.approx(+I[1, 2])
    assert np.allclose(_inertia_flu(J), I)
    # the physics must not change: same eigenvalues either way
    assert np.allclose(sorted(np.linalg.eigvalsh(I)), sorted(np.linalg.eigvalsh(J)))


@pytest.mark.parametrize("angles", [(0.0, 0.0, 0.0), (11.0, -23.0, 47.0), (0.0, 179.0, 0.0), (180.0, 0.0, 0.0)])
def test_quaternion_recovery_from_a_rotation_matrix(angles):
    """Including attitudes near 180 degrees, where the shortest-arc branch divides by something near zero."""
    q = q_from_euler(*(math.radians(a) for a in angles))
    R = q_to_rotmat(q)
    assert np.allclose(q_to_rotmat(_q_from_rotmat(R)), R, atol=1e-9)


def _airframe() -> Airframe:
    return Airframe.from_dict({
        "schema": 2, "name": "gz_fixture",
        "mass": {"mass": 4.0, "cg": [0.05, 0.0, 0.02],
                 "inertia": [0.2, 0.3, 0.4], "inertia_products": [0.001, 0.02, 0.003]},
        "body": {"size": [0.4, 0.2, 0.1]},
        "rotors": [{"name": "M1", "pos": [0.2, 0.2, 0.0], "axis": [0, 0, -1], "km": 0.01, "max_thrust": 20.0},
                   {"name": "M2", "pos": [0.2, -0.2, 0.0], "axis": [0, 0, -1], "km": -0.01, "max_thrust": 20.0}],
        "legs": [{"name": "L", "attach": [0.1, 0.1, 0.0], "length": 0.2, "foot_radius": 0.02, "friction": 0.9},
                 {"name": "R", "attach": [0.1, -0.1, 0.0], "length": 0.2, "foot_radius": 0.02, "friction": 0.9}],
        "landed_pitch_deg": 0.0, "hover_pitch_deg": 0.0,
    })


def test_generated_world_is_valid_sdf_with_the_pieces_the_backend_needs(tmp_path):
    af = _airframe()
    path = generate_world(af, "fixture", tmp_path)
    root = ET.parse(path).getroot()
    world = root.find("world")
    assert world.get("name") == "fixture"

    systems = {p.get("filename") for p in world.findall("plugin")}
    for needed in ("gz-sim-physics-system", "gz-sim-apply-link-wrench-system"):
        assert needed in systems, f"{needed} missing: the backend cannot step or apply force without it"

    model = next(m for m in world.findall("model") if m.get("name") == "fixture")
    link = model.find("link")
    assert link.get("name") == "base_link"
    assert float(link.find("inertial/mass").text) == pytest.approx(4.0)

    # The link frame is the centre of gravity, so the tensor written out is the airframe's carried into FLU.
    J = _inertia_flu(af.mass.tensor())
    inertia = link.find("inertial/inertia")
    for tag, value in (("ixx", J[0, 0]), ("iyy", J[1, 1]), ("izz", J[2, 2]),
                       ("ixy", J[0, 1]), ("ixz", J[0, 2]), ("iyz", J[1, 2])):
        assert float(inertia.find(tag).text) == pytest.approx(value, abs=1e-9), tag

    # One collision sphere per leg, at the foot, in the model frame relative to the centre of gravity.
    feet = [c for c in link.findall("collision") if c.get("name").startswith("foot")]
    assert len(feet) == len(af.active_legs())
    expected = _frd_flu(np.asarray(af.active_legs()[0].foot(), float) - np.asarray(af.cg, float))
    assert [float(v) for v in feet[0].find("pose").text.split()[:3]] == pytest.approx(list(expected), abs=1e-9)


def test_generated_world_is_a_pure_function_of_the_airframe(tmp_path):
    """Regenerating must be safe, so the same airframe has to produce byte-identical SDF."""
    af = _airframe()
    a = generate_world(af, "fixture", tmp_path / "a").read_text()
    b = generate_world(af, "fixture", tmp_path / "b").read_text()
    assert a == b
