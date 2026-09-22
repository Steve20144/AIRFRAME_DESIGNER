# Scenario runner and ground sequences

## Scenario JSON

```
{ "name", "description", "max_time", "params": {PX4 name: value}, "wind": [n,e,d], "abort": {max_tilt_deg, max_alt,
  crash_speed}, "attitude": {"park_pitch_deg", "hover_pitch_deg"}, "phases": [...] }
```

Phase types: `wait_ready`, `nose_lift`, `nose_lower`, `takeoff`, `hold`, `land`, `offboard_position`,
`offboard_velocity`, `manual`, `stick`, `wind`, `motor_failure`, `param`, `mode`, `arm`, `wait`.

`stick` streams a pilot's sticks at 50 Hz in a manual mode (`mode`: stabilized, altitude, position ...): `arm`,
`throttle`, cosine `ramp_s` from the previous phase's throttle, `alt_hold` {target, rate, kp, kv, max_corr, min, max}
(a throttle hand; attitude stays with PX4; `rate` slews the target, i.e. a descent), `until_ground` + `settle` (ends
on ground contact after having been airborne), `until_pitch_deg`. `nose_lift` / `nose_lower` default their target to
`design.nose_lift.target_pitch_deg` / `design.nose_lower.target_pitch_deg`, else hover / landed pitch.

## The attitude block

`attitude` makes the scenario own the two angles every ATLAS flight is built around. Before the run,
`Airframe.with_attitude` re-solves the legs under the same hard points so the aircraft rests at `park_pitch_deg`
(`gear.legs_for_park_pitch`: feet on the ground plane, mean leg length kept, front feet moved to 8 cm ahead of the
CG if needed, rear feet keep their horizontal arm from the old park so the lift lever does not change; tricycle
aware), sets `landed_pitch_deg`, sets `hover_pitch_deg` (PX4's level, SENS_BOARD_Y_OFF, rotor export), and shifts
the design lift / lower targets by the same offsets. A stand whose CG projects outside the feet raises a clear
error instead of flipping the model. The Tuning tab's Park / Hover fields override the block per attempt.

## Ground sequences (`sim/nose_lift.py`)

- **NoseLift**: before arming, the chosen (front) motors raise the nose from the parked pitch to the target at
  `rate_deg_s` with a balance-plus-rate loop about the rear feet, hold, then hand over: the floor stays under PX4's
  commands until PX4's own commands reach it. A split between the two lift motors nulls yaw (and roll) from canted
  fans and their reaction torque.
- **NoseLower**: armed in the air (`design.nose_lower.enabled` arms it automatically above 1 m); at touchdown it
  cuts every other motor and lowers the nose to the parked pitch with the same loop, fades, then force-disarms PX4.
  The scenario's `nose_lower` phase adopts a running hook (`px4_land: false` for manual landings).
- Lift margin: the nose fans' maximum moment about the rear feet must exceed gravity's; ATLAS_OG at +8 has ample
  margin, ATLAS_09B at -12 has 5 percent and stalls, at +4 19 percent. See lessons/stands-and-lift-margin.md.

## Scenarios that matter

- `stab_lab` (attitude 8 / 26, ATLAS_OG): nose lift, arm in Stabilized, throttle hand to 2.5 m, hover 12 s, descend
  0.4 m/s, cut, nose lower. `stab_lab_fast`: 1 s liftoff ramp. `stab_lab_pivot`, `stab_lab_pivot14`: -10 / -14 parks.
- `stab_lab_native`, `stab_lab_native_fast`: the same at the airframe's own angles (ATLAS_09B). `stab_lab_nolift`:
  diagnostic, parked at hover pitch.
- `nose_lift_gust_stab`: Stabilized twin of the gust test. `pivot_stab`, `pivot_land`, `nose_lift_takeoff`,
  `nose_lift_gust` (Position mode flows).
