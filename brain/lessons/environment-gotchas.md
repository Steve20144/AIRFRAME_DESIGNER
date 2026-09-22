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
- Landing on the +8 stand in Position mode: PX4 holds the hover attitude until land detection; the nose lower hook
  handles it; tip-overs fault the compass and need an estimator restart (on the board: reboot, never restart EKF2
  in place).
