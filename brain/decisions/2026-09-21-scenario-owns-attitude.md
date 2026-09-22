# Decision: The scenario owns the parked and hover pitch

Date: 2026-09-21
Status: accepted

## Context

The user's requirement for every ATLAS flight, sweeps included: rotate from a parked pitch the scenario defines
(for example -10 deg) to a hover pitch the scenario defines (for example 25 deg), lock stable, fly, land at the
hover pitch, rotate back. Previously the parked pitch came from the airframe's legs and the hover pitch from the
airframe; a scenario could not start a model from another angle without editing the airframe.

## Decision

Scenarios carry `"attitude": {"park_pitch_deg", "hover_pitch_deg"}`. Before a run the airframe is re-stood at the
parked pitch (legs re-solved under the same hard points, tip-over checked) and its hover pitch set; the nose lift
and nose lower targets follow. Headless runs, live runs and the Tuning tab (Park / Hover fields) all honour it.

## Reasoning

The parked pitch is a property of the stand, not the aircraft, and the hover pitch is PX4's level: both are test
conditions. Putting them in the scenario makes a sweep self-describing and lets the same model be tried from
several parks. Re-solving legs keeps the CAD hard points, so only the stand changes.

## Alternatives Considered

- One airframe file per stand: many near-duplicate models drifting apart.
- `--set` paths for legs: eight numbers per stand, easy to get wrong, no tip-over check.

## Consequences

`Airframe.with_attitude`, `gear.legs_for_park_pitch`, `Scenario.attitude`, `apply_attitude` in `run_once` and in
the live start. The solver moves the front feet to 8 cm ahead of the CG when needed and keeps the rear feet's
horizontal arm (the lift lever). Some requests are physically impossible on given hard points (ATLAS_OG at -5:
CG ahead of the front feet; ATLAS_09B at -12: lift margin 5 percent) and now fail with a sentence.
