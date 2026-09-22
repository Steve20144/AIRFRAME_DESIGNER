# Tuning tab

Added 2026-09-21. Server helpers in `airframe_designer/server/tuning.py`, routes in `server/app.py` (section
"tuning tab"), UI in `ui/index.html` (`#tab-tuning`), `ui/app.js` (section "tuning tab"), `ui/style.css`.

## Attempt

`POST /api/tuning/run` `{name, scenario, params: {PX4 name: value}, variables: {path: value}, attitude:
{park_pitch_deg, hover_pitch_deg}, options: {physics, seed, noise}, live: bool}`. Headless: copies the app's
airframe (via `base_airframe()`: the one from before a live scenario applied its attitude block, while the live
one is still exactly that), applies `px4.<name>` variables, reserves a PX4 instance (9 downward, skipping a running sweep's), runs
`run_once` with a time-series path, saves `results/tuning/<id>.json` + `<id>_ts.json` with `name`, `kind`,
`params`. Live: pushes `params` to the connected PX4 and starts the scenario on the live simulator; the flight is
saved when the scenario ends (`scenario_status` writes it, `tuning_id` in the status).

## Sweep

`POST /api/tuning/sweep` `{name, scenario, variables: [{param, min, max, levels}], workers, base_variables,
objective, constraints, algorithm}` builds a study spec (`tuning.sweep_spec`, grid by default, `timeseries: true`)
on the current airframe and runs it through the shared study job. Default objective and constraints live in
`tuning.py` (`TUNING_OBJECTIVE`, `TUNING_CONSTRAINTS`): hover tracking error + rates + liftoff and landing
excursions + touchdown speed + saturation; feasible when ok, not crashed, touchdown under 0.8 m/s, yaw drift under
10 deg, hover drift under 2 m.

## Library and charts

`GET /api/tuning/runs` (library + running jobs + free instances), `GET /api/tuning/run/<id>?points=1500`
(decimated, NaN -> null), `DELETE /api/tuning/run/<id>`, `GET /api/tuning/sweeps` (every `results/*/trials.jsonl`
with briefs per trial), `GET /api/tuning/sweep/<name>/trial/<k>`, `GET /api/tuning/defaults` (current gains,
objective). `tuning.brief()` is the dozen numbers a row shows. Charts are inline SVG drawn in `app.js`
(`tuneChart`): height, attitude with PX4's setpoint dashed, yaw (unwrapped), rates, position, motors; phases
shaded; "flight only, +-5 deg" clamps the view; two selected flights overlay, the second dashed. The list
re-renders only when its data signature changes so clicks are never lost.

## Reports

`scripts/report_runs.py --out r.html [--study results/<sweep>]... results/*.json` writes a self-contained HTML with
the same charts (decimated to 700 points) and, per sweep, objective-vs-variable scatter plots and a ranked table.
