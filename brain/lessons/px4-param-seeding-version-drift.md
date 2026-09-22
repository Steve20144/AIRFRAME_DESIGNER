# Lesson: PX4 parameter seeding drifts with the firmware version

Batch runs seed parameters through `fs/parameters.bson` before PX4 boots. Two silent failure modes look exactly
like flight-dynamics bugs:

- `SYS_AUTOSTART` not matching the model (10016 for none_iris) makes PX4 reset every parameter at boot.
- Parameter names or types that the firmware does not know are ignored; the export then flies a different aircraft
  than the JSON says. `px4_params_verified` in every result compares seeded and vehicle values for a handful of
  keys; read it before believing a strange flight.

Related: the shared PX4 checkout is v1.17.0 with a dirty tree; stock release firmware for the board lacked the
HITL driver, a HITL build was flashed 2026-09-18 (`scripts/build_hitl_firmware.sh`, worktree `~/PX4-hitl`).
