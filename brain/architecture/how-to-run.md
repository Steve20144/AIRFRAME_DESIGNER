# How to run

## macOS (Apple Silicon, since 2026-09-26)

The repo also runs natively on macOS (arm64, macOS 26), no Homebrew or sudo needed:

- repo: `~/Documents/UtopiaLabs/AIRFRAME_DESIGNER`; interpreter `.venv/bin/python` (Python 3.12 from `uv`, installed
  to `~/.local/bin/uv`): `uv venv --python 3.12 .venv && uv pip install -e ".[dev]" cadquery-ocp matplotlib resvg-py`
- PX4 SITL: `~/PX4-Autopilot` at v1.17.0 (clean clone, `--recurse-submodules`), build tools in `~/.venvs/px4-build`
  (PX4's `Tools/setup/requirements.txt` + pip `cmake ninja`). Build:
  `PATH=~/.venvs/px4-build/bin:$PATH CMAKE_POLICY_VERSION_MINIMUM=3.5 SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk make px4_sitl_default`
- `SDKROOT` matters: the Command Line Tools default to the MacOSX27.0 SDK, whose `.tbd` stubs (arm64e.x1) the
  installed ld-1267 cannot read, so every link fails ("tapi error: malformed file"). CMake 4 needs the policy floor.
- Commands are the same as below with `.venv/bin/python`; no `wsl.exe` wrapper. One V3 flight takes ~5 s wall
  (rtf ~10), a 286-design x 3-pitch study ~19 min on 8 workers. All 248 fast tests and the `px4` test pass.
- `airframes/cad/*.bodies.json` (CAD meshes, gitignored) is rebuilt by `geometry.cad.import_step(path)` (needs
  cadquery-ocp). `scripts/v3_foil_views.py` writes the PNGs through resvg off Windows (it inlines the CSS vars).
- Not ported: the HITL/board tooling (`usbip.py`, COM ports, firmware flashing); on macOS the board is a `/dev/cu.*`
  device and needs no usbipd.

## Environment (Windows + WSL)

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

## The aircraft's board (Pixhawk 6X Pro)

- Parameters: `python scripts/board_params.py backup|diff|get|set --port COM3` over USB (Windows Python, which has
  pymavlink), or `--port udpin:127.0.0.1:14550` over the SiK radio while `scripts/throttle_dashboard.py` runs; `set`
  reads every value back; backups in `results/board_params/`.
- Firmware: USB only. `bash scripts/build_nose_lift_firmware.sh board build` (PX4 tree ~/PX4-nl; the script copies the
  firmware/px4_ext of the tree it is run from), then PX4's `Tools/px_uploader.py` from Windows Python on the board's
  COM port (never COM6, the radio). Around it: back up parameters, keep the board's image as a rollback
  (`~/firmware_backups`), compare strings/size, flash, `diff` the parameters, set, confirm with `ver all`.
- Before a flash, ask the board what it runs (`ver all` over the radio, parameters present or not); a commit message
  is not evidence of what was flashed.


## Theory notebooks (2026-10-01)

`notebooks/00_index.ipynb` .. `07_jsbsim_and_chrono.ipynb`: frames, quaternions, equations of motion, ground contact,
sensors (H-FLOW), fans/jetfoils/Coanda, JSBSim/Chrono. Theory and plain-language notes above runnable cells that call
the real package on `airframes/atlas_v3_v34_foils_50_65_50_tuned.json`. Kernel "AIRFRAME_DESIGNER (.venv)"
(registered with `.venv/bin/python -m ipykernel install --user --name airframe-designer`; nbformat, nbclient and
ipykernel installed with `uv pip install --python .venv/bin/python`, the venv has no pip). Source of truth is
`notebooks/_build_notebooks.py`; rebuild with it rather than hand-editing the .ipynb files. Long-form version:
`docs/ATLAS_Simulator_Physics.pdf`.
