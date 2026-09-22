# Decision: Tuning lives in the app, on the existing scenario and study runners

Date: 2026-09-21
Status: accepted

## Context

Tuning was done with CLI studies and ad-hoc scripts; results were JSON files the user could not watch. The user
wanted to launch attempts, oversee sweeps and visualise takeoff, hover and landing in the browser tab next to the
3D simulator, and to fly the final version live.

## Decision

A Tuning tab (architecture/tuning-tab.md): attempts headless or live, grid sweeps, a persistent flight library
under `results/tuning/` and `results/<sweep>/`, inline SVG charts of every flight with PX4's setpoints, and
one-click apply of a trial's values. Built on `run_once`, `run_study`, the live `ScenarioRunner` and
`MetricsRecorder`; no second simulation path.

## Reasoning

Reusing the runners guarantees that what the tab shows is what the CLI and the HITL session fly. Persisting every
flight with its time series makes tuning arguments reproducible. Drawing charts in the page avoids a build step.

## Alternatives Considered

- Notebook or matplotlib workflow: not visible from the app, not shareable, no live flights.
- A separate dashboard app: a second front end for the same data.

## Consequences

Metrics gained setpoint and estimate columns; studies gained `base_variables` and `timeseries: true`; attempts
reserve PX4 instances (9 downward, skipping a running sweep's). `scripts/report_runs.py` produces the same charts
as static HTML for sharing.
