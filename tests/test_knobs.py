"""Design knobs: jetfoil angles, front jets and battery as named values that move rotors and CAD parts together."""
from pathlib import Path

import numpy as np
import pytest

from airframe_designer.geometry import knobs
from airframe_designer.geometry.airframe import Airframe
from airframe_designer.geometry.paths import apply_variables, get_path
from airframe_designer.server.tuning import sweep_spec

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture()
def atlas():
    af = Airframe.load(ROOT / "airframes" / "atlas_09b.json")
    knobs.setup_defaults(af)
    af.resolve_mass()
    return af


def test_defaults_cover_foil_stations_front_and_battery(atlas):
    ks = {k["name"]: k for k in knobs.describe(atlas)}
    assert [ks[f"foil_{i}_deg"]["rotors"] for i in range(1, 5)] == [["M1", "M2"], ["M3", "M4"], ["M5", "M6"], ["M7", "M8"]]
    assert [round(ks[f"foil_{i}_deg"]["value"]) for i in range(1, 5)] == [25, 30, 35, 40]
    assert ks["front_cant_deg"]["rotors"] == ["M9", "M10"] and round(ks["front_cant_deg"]["value"]) == 30
    assert len(ks["cg_dx"]["bodies"]) == 5                       # the five battery packs
    assert set(atlas.design["cad_links"]) == {"M9", "M10"}         # the front fans' parts turn with them


def test_setup_leaves_the_aircraft_unchanged():
    af = Airframe.load(ROOT / "airframes" / "atlas_09b.json")
    cg0, axes0 = list(af.mass.cg), [list(r.axis) for r in af.rotors]
    knobs.setup_defaults(af)
    af.resolve_mass()
    assert af.mass.cg == cg0 and [list(r.axis) for r in af.rotors] == axes0


def test_front_tilt_turns_the_fans_and_their_parts(atlas):
    b = apply_variables(atlas, {"knobs.front_cant_deg": 0})
    assert [round(r.cant_deg, 3) + 0.0 for r in b.rotors[8:]] == [0.0, 0.0]
    fan = next(x for x in b.cad.bodies if x.id in b.design["cad_links"]["M9"]["bodies"])
    R = np.asarray(fan.rot).reshape(3, 3)
    assert np.allclose(R @ np.asarray(atlas.rotors[8].axis), b.rotors[8].axis, atol=1e-5)
    assert b.mass.cg != atlas.mass.cg                              # the fan's mass moved with it


def test_battery_knob_moves_the_cg_and_is_reversible(atlas):
    b = apply_variables(atlas, {"knobs.cg_dx": 0.05})
    battery = sum(x.mass for x in atlas.cad.bodies if x.id in knobs.find(atlas, "cg_dx")["bodies"])
    assert b.mass.cg[0] - atlas.mass.cg[0] == pytest.approx(0.05 * battery / atlas.mass.mass, abs=2e-5)
    assert get_path(b, "knobs.cg_dx") == 0.05
    back = apply_variables(b, {"knobs.cg_dx": 0.0, "knobs.front_cant_deg": 30})
    assert back.mass.cg == atlas.mass.cg


def test_knobs_survive_the_json_round_trip(atlas):
    b = apply_variables(atlas, {"knobs.foil_1_deg": 32, "knobs.cg_dz": -0.02})
    c = Airframe.from_dict(b.to_dict())
    assert get_path(c, "knobs.foil_1_deg") == pytest.approx(32, abs=0.01)
    assert c.mass.cg == b.mass.cg


def test_without_cad_the_battery_knob_shifts_the_cg():
    af = Airframe.load(ROOT / "airframes" / "atlas_09b.json")
    af.cad = None
    af.mass.from_items = False
    knobs.setup_defaults(af)
    cg0 = list(af.mass.cg)
    knobs.set_value(af, "cg_dx", 0.03)
    assert af.mass.cg[0] == pytest.approx(cg0[0] + 0.03)
    knobs.set_value(af, "cg_dx", 0.0)
    assert af.mass.cg == cg0


def test_sweep_form_takes_knob_paths_and_px4_names():
    spec = sweep_spec({}, "stab_lab", [{"param": "knobs.front_cant_deg", "min": 0, "max": 30, "levels": 4},
                                       {"param": "MC_ROLL_P", "min": 3, "max": 6, "levels": 2}], name="n")
    assert [v["path"] for v in spec["variables"]] == ["knobs.front_cant_deg", "px4.MC_ROLL_P"]
    assert spec["name"] == "n" and spec["algorithm"]["budget"] == 8


def test_v3_front_tilt_keeps_the_centre_nose_fan_upright():
    af = Airframe.load(ROOT / "airframes" / "atlas_v3_small.json")
    k = knobs.find(af, "front_cant_deg")
    assert k["rotors"] == ["M7", "M8", "M9"] and k["signs"] == [-1.0, -1.0, 0.0]
    b = apply_variables(af, {"knobs.front_cant_deg": 15})
    assert [round(r.cant_deg, 2) + 0.0 for r in b.rotors[6:]] == [-15.0, -15.0, 0.0]


def test_trim_pitch_follows_the_jetfoil_angles():
    af = Airframe.load(ROOT / "airframes" / "atlas_v3_small.json")
    p0 = af.trim_hover_pitch()
    assert 11.0 <= p0 <= 12.5                      # the draft with ATLAS_09B's masses
    best = apply_variables(af, {"knobs.foil_1_deg": 10, "knobs.foil_2_deg": 25, "knobs.foil_3_deg": 10, "knobs.front_cant_deg": 0})
    best.hover_pitch_deg = best.trim_hover_pitch()
    hc = best.hover_check()
    assert hc["ok"] and max(hc["hover_utilisation"]) < 0.6
    b = apply_variables(af, {"knobs.foil_1_deg": 10, "knobs.foil_2_deg": 20, "knobs.foil_3_deg": 30})
    assert b.trim_hover_pitch() > p0 + 1          # more forward lean on average: hovers more nose-up


def test_trim_matches_the_flown_atlas_09b():
    assert abs(Airframe.load(ROOT / "airframes" / "atlas_09b.json").trim_hover_pitch() - 24.0) < 1.0
