"""Parameter paths: reading, writing and listing any numeric value of an airframe by a string address."""
from __future__ import annotations

import pytest

from airframe_designer.geometry import apply_variables, get_path, list_paths, set_path
from airframe_designer.geometry.airframe import quad_x


def test_single_element_paths(quad):
    assert get_path(quad, "rotors[0].pos[2]") == 0.0
    set_path(quad, "rotors[0].pos[2]", -0.05)
    assert quad.rotors[0].pos[2] == -0.05
    assert get_path(quad, "mass.cg[0]") == 0.0
    set_path(quad, "mass.cg[0]", 0.03)
    assert quad.mass.cg == [0.03, 0.0, 0.0]
    set_path(quad, "hover_pitch_deg", 12.5)
    assert quad.hover_pitch_deg == 12.5 and get_path(quad, "hover_pitch_deg") == 12.5


def test_multi_index_virtual_attribute(quad):
    assert get_path(quad, "rotors[0,1].tilt_deg") == pytest.approx([0.0, 0.0])
    set_path(quad, "rotors[0,1].tilt_deg", 20.0)
    assert [r.tilt_deg for r in quad.rotors] == pytest.approx([20.0, 20.0, 0.0, 0.0], abs=1e-3)
    assert get_path(quad, "rotors[0,1].tilt_deg") == pytest.approx([20.0, 20.0], abs=1e-3)


def test_wildcard_and_slice(quad):
    set_path(quad, "rotors[*].tau", 0.1)
    assert [r.tau for r in quad.rotors] == [0.1] * 4
    assert get_path(quad, "rotors[*].tau") == [0.1] * 4
    set_path(quad, "rotors[0:2].max_thrust", 10.0)
    assert [r.max_thrust for r in quad.rotors] == [10.0, 10.0, 8.0, 8.0]
    assert get_path(quad, "rotors[0:2].max_thrust") == [10.0, 10.0]
    set_path(quad, "legs[*].length", 0.2)
    assert [l.length for l in quad.legs] == [0.2] * 4
    assert get_path(quad, "legs[-1].length") == 0.2


def test_by_name(quad):
    set_path(quad, "rotors[M1].km", 0.07)
    assert quad.rotors[0].km == 0.07 and quad.rotors[1].km == 0.05
    assert get_path(quad, "rotors[M1].km") == 0.07
    set_path(quad, "legs[FR,RL].length", 0.3)
    assert [l.length for l in quad.legs] == [0.3, 0.12, 0.12, 0.3]


def test_wing_aero_path(plane):
    assert get_path(plane, "wings[0].aero.cd0") == pytest.approx(0.02)
    set_path(plane, "wings[0].aero.cd0", 0.04)
    assert plane.wings[0].aero.cd0 == 0.04
    set_path(plane, "wings[main].incidence_deg", 5.0)
    assert plane.wings[0].incidence_deg == 5.0


def test_px4_override_path_creates_entry(quad):
    assert "MC_PITCH_P" not in quad.px4_overrides
    set_path(quad, "px4.MC_PITCH_P", 4.5)
    assert quad.px4_overrides == {"MC_PITCH_P": 4.5}
    assert get_path(quad, "px4.MC_PITCH_P") == 4.5
    set_path(quad, "px4.MC_PITCH_P", 5.0)
    assert quad.px4_overrides["MC_PITCH_P"] == 5.0


def test_design_path(quad):
    quad.design["cruise_speed_kmh"] = 50
    assert get_path(quad, "design.cruise_speed_kmh") == 50
    set_path(quad, "design.cruise_speed_kmh", 72.4)
    assert quad.design["cruise_speed_kmh"] == 72              # coerced to the current int type
    set_path(quad, "design.new_key", 3.5)
    assert quad.design["new_key"] == 3.5


def test_coercion_keeps_types(quad):
    set_path(quad, "rotors[0].enabled", 0)
    assert quad.rotors[0].enabled is False
    set_path(quad, "rotors[0].max_thrust", 9)
    assert quad.rotors[0].max_thrust == 9.0 and isinstance(quad.rotors[0].max_thrust, float)
    set_path(quad, "wings", [])
    assert quad.wings == []


def test_apply_variables_leaves_original_untouched(quad):
    new = apply_variables(quad, {"rotors[*].tilt_deg": 15.0, "mass.cg[0]": 0.02, "px4.MC_PITCH_P": 3.0, "hover_pitch_deg": 10.0})
    assert new is not quad
    assert [r.tilt_deg for r in quad.rotors] == pytest.approx([0.0] * 4)
    assert quad.mass.cg == [0.0, 0.0, 0.0] and quad.px4_overrides == {} and quad.hover_pitch_deg == 0.0
    assert [r.tilt_deg for r in new.rotors] == pytest.approx([15.0] * 4, abs=1e-3)
    assert new.mass.cg[0] == 0.02 and new.px4_overrides == {"MC_PITCH_P": 3.0} and new.hover_pitch_deg == 10.0
    assert quad.to_dict() == quad_x().to_dict()


def test_apply_variables_resolves_mass_items():
    af = quad_x()
    af.mass.from_items = True
    af.mass.items = [type(af.mass.items)()] if False else []
    from airframe_designer.geometry import MassItem
    af.mass.items = [MassItem("battery", 1.0, [0.0, 0.0, 0.0]), MassItem("frame", 1.0, [0.0, 0.0, 0.0])]
    af.mass.resolve()
    new = apply_variables(af, {"mass.items[0].pos[0]": 0.2})
    assert new.mass.cg[0] == pytest.approx(0.1)
    assert af.mass.cg[0] == 0.0


def test_list_paths(quad, plane):
    paths = list_paths(quad)
    assert "rotors[0].tilt_deg" in paths and "rotors[0].cant_deg" in paths
    assert "mass.cg[0]" in paths and "mass.cg[2]" in paths
    assert "rotors[3].km" in paths and "legs[0].length" in paths and "hover_pitch_deg" in paths
    assert "rotors[0].pos[1]" in paths
    assert not any(p.startswith("name") for p in paths)
    quad.px4_overrides["MC_PITCH_P"] = 3.0
    quad.design["cruise_speed_kmh"] = 50
    quad.design["groups"] = {"A": {}}
    paths = list_paths(quad)
    assert "px4.MC_PITCH_P" in paths and "design.cruise_speed_kmh" in paths
    assert not any(p.startswith("design.groups") for p in paths)
    assert "wings[0].aero.cd0" in list_paths(plane)
    # every listed path can be read back
    for p in list_paths(plane):
        get_path(plane, p)


@pytest.mark.parametrize("bad", ["nope.x", "rotors[Z9].km", "rotors[0].pos[", "rotors[0].nothing", "1abc", "wings[0].span", "rotors[0].pos.x"])
def test_bad_paths_raise_on_read(quad, bad):
    with pytest.raises((AttributeError, KeyError, ValueError, IndexError, TypeError)):
        get_path(quad, bad)


@pytest.mark.parametrize("bad", ["nope.x", "rotors[Z9].km", "rotors[0].pos[", "1abc", "wings[0].span"])
def test_bad_paths_raise_on_write(quad, bad):
    with pytest.raises((AttributeError, KeyError, ValueError, IndexError)):
        set_path(quad, bad, 1.0)
    assert quad.to_dict() == quad_x().to_dict()


@pytest.mark.parametrize("path", ["rotors[7].km", "mass.cg[7]", "legs[4].length"])
def test_out_of_range_index_raises(quad, path):
    with pytest.raises((IndexError, KeyError, ValueError)):
        get_path(quad, path)


def test_set_path_rejects_unknown_attribute(quad):
    with pytest.raises(AttributeError):
        set_path(quad, "rotors[0].max_thurst", 5.0)
