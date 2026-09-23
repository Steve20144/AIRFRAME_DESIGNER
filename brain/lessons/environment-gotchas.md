# Lesson: environment and process gotchas

- Run everything in WSL Ubuntu-24.04 with `~/.venvs/airframe/bin/python`; pass scripts to `wsl.exe bash -l -s` on
  stdin, not as quoted arguments (Git Bash rewrites paths and quotes; long heredocs through the Windows shell were
  truncated once at about 100 lines; write big files with a file tool and append with Python).
- `pkill -f "airframe_designer ui --mode hitl"` once killed the user's 8080 app; match a pid or a port, and restart
  only an app you started. Instance 0 / 8080 is sacred.
- Two headless runs started in the same second collide on a PX4 instance ("Address already in use"); the Tuning
  attempts reserve instances (9 downward) and skip a running sweep's lowest ones.
- Studies write their trials to `results/<name>/trials.jsonl` as they go; `run_study` resumes from it.
- FastAPI refuses NaN in JSON; time series with NaN setpoint columns must be nulled before returning.
- The browser pane cannot screenshot a hidden or minimised window; verify pages with `read_page` / JavaScript.
- JSBSim positions were unsigned once (every metre west came back east); fixed in commit c62a11b; compare engines when a
  horizontal result looks odd.
- Sensor noise off makes PX4 refuse to arm (preflight drift checks dislike a perfectly quiet IMU).
- A new `NL_*` parameter in `firmware/px4_ext/.../params.c` failed to compile ("not a member of px4::params"):
  PX4 regenerates `parameters.xml` only for params.c under its own `src/`. `build_nose_lift_firmware.sh` now deletes
  the generated file when ours change (fixed 2026-09-23).
- `wsl -- bash -lc '...$VAR...'` from PowerShell expands `$VAR` too early; put the commands in a script file and run
  `wsl -d Ubuntu-24.04 -- bash /mnt/c/.../script.sh`.
- Edits to the physics core need the test suite (`pytest -q`, 235 tests, ~15 s): a misplaced method once cut
  `RotorAero.__init__` short and every fan made zero thrust while SITL scenarios merely "timed out".
- Landing on the +8 stand in Position mode: PX4 holds the hover attitude until land detection; the nose lower hook
  handles it; tip-overs fault the compass and need an estimator restart (on the board: reboot, never restart EKF2
  in place).

## A live scenario's attitude block used to leak into headless work (fixed 2026-09-21)

`POST /api/scenario/start` applies the scenario's `attitude` (or the Tuning card's Park/Hover) to the live airframe
with `sim.set_airframe`, and that airframe stayed the app's airframe afterwards. Headless attempts, sweeps, study
runs and `save` copied it, so a round on `stab_lab_native*` (no attitude block) silently ran at the last live
scenario's stance (hover 26 / park -5 with re-solved legs instead of the file's 24 / +4) and every trial crashed.
Fix: `state.attitude_base` remembers the pre-attitude airframe plus a fingerprint of what the scenario set;
`base_airframe()` in `server/app.py` returns the base while the live airframe is still exactly that, and headless
work and saves start from it. Check the first trial's `SENS_BOARD_Y_OFF` in `px4_params_verified` and the nose
lift's `start_pitch_deg` when a sweep behaves unlike the model's history.
