# How to run

## Environment

The repo sits on the Windows filesystem but runs only inside WSL `Ubuntu-24.04`:

- repo: `~/utopia/AIRFRAME_DESIGNER` (`~/utopia` -> `C:\Users\stefa\Documents\Utopia Labs`)
- interpreter: `~/.venvs/airframe/bin/python` (Python 3.12.3; there is no `.venv` in the repo)
- PX4 SITL: `~/PX4-Autopilot/build/px4_sitl_default` (v1.17.0, shared with other projects, dirty tree)
- HITL firmware worktree: `~/PX4-hitl`; Gazebo Harmonic installed in the same distro

From Windows tooling, run commands through `wsl.exe -d Ubuntu-24.04 bash -l -s <<'EOF' ... EOF` (stdin script),
not as a quoted argument; Git Bash mangles paths and quotes otherwise.

## Commands (substitute the WSL interpreter for `.venv/bin/python`)

- Interactive app: `python -m airframe_designer ui [--airframe airframes/X.json] [--mode auto]` -> http://127.0.0.1:8080
  Instance 0 / port 8080 may already be in use by a running session: never kill it. Restart only an app you started.
- Headless: `python -m airframe_designer run --airframe X --scenario Y [--set path=value] [--param NAME=VALUE]
  [--instance N] [--seed S] [--physics jsbsim] --out r.json --timeseries r_ts.json`
- Parallel: `batch --tasks t.json --workers 6 --instances 1,2,3,4,5,6 --out results.jsonl` (tasks carry
  `variables` and per-task `options` such as `seed`, `physics`, `timeseries_path`).
- Studies: `study --spec studies/<name>.json` (spec: airframe, `base_variables`, scenario, variables with ranges,
  objective, constraints, algorithm grid/random/cmaes/nelder_mead, workers, `timeseries: true`).
- Compare engines: `compare --airframe X --scenario Y`. Static analysis: `analyse --airframe X`.
- Export for the board: `export --airframe X --hitl --out X.params`.
- Reports: `python scripts/report_runs.py --out r.html --study results/<sweep> results/*.json`.
- Tests: `python -m pytest -q -m "not px4"` (222 tests, ~10 s); the `px4` marked test boots PX4 on instance 8.

## Ports and instances

PX4 instance `i` uses TCP 4560+i for HIL and UDP 14540+i for control. The app uses instance 0. Batch, studies and
Tuning attempts use 1 to 9; a running sweep cycles PX4 on the lowest instances between trials, so attempts started
from the app reserve instances from 9 downward and skip the sweep's.

## Conventions

- Structural frame FRD, origin = reference point, CG in `mass.cg`; force models take positions relative to the CG.
- PX4 export: rotor positions relative to the CG rotated into the hover frame; `km` sign = spin direction;
  CA_ROTORn_CT proportional to effective max thrust (jet turning loss included).
- Batch runs seed parameters through `fs/parameters.bson` before boot; `SYS_AUTOSTART` must match the model
  (10016 for none_iris) or PX4 resets everything.
- Keep `run_once` results JSON-serialisable (no NaN in API responses: the Tuning routes null them).
- When changing physics: run `tests/`, compare forces on random states before and after, keep the per-step cost.
