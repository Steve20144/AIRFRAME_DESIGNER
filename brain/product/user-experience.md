# User Experience

## Primary User Flow

1. Load or edit the airframe in the Geometry tab; Update PX4 writes the geometry to the connected flight controller.
2. Pick the scenario that matches the lab flow (`stab_lab`, `stab_lab_native`, or a pivot variant) in the Tuning tab.
3. Run headless attempts and grid sweeps; every flight lands in the Flights table with its numbers; click a row for
   charts, tick two to compare; apply a trial's values to the airframe with one click.
4. Fly live: the same sequence on the app's PX4 (SITL or the HITL board), the 3D view following the aircraft, the
   flight saved when the scenario ends.
5. Save the airframe; export `.params`; go to HITL in the Connect tab (Attach USB hands the board to WSL).
6. In the lab, the pilot's remote: SB down rotates the nose (nose lift), the pilot arms in Stabilized and flies,
   lands, and the nose lower runs on touchdown.

## Important Interactions

- Controller tab (2026-09-22): a RadioMaster T8L drawing (SA, SD push buttons; SB, SC 3-position; SE 2-position on
  the back; S1 dial) with live sticks and switches. Learn watches every channel for 6 s while the pilot moves one
  control; each control gets a function (emergency stop, arm, flight mode per position, staged takeoff = the
  nose-lift switch, return, hold; staged landing listed but not on the flight controller yet). The plan shows the
  exact PX4 parameters (thresholds from RCn_MIN/TRIM/MAX/REV as rc_update computes them, mode slots 1-6, clears a
  function left on a channel that now does something else, COM_KILL_DISARM 0 for the kill, COM_ARM_SWISBTN for a
  button) and Write to board sets and saves them. Learned channels live in ~/.airframe_designer/controller.json.
  Embedded browsers answer window.confirm() with cancel: confirmations are a second click (confirmClick).
- Tabs: Geometry, PX4, Optimize, Batch, Tuning, Flight, Connect, Controller, Flash. The Flight tab holds modes, wind, noise, the
  nose lift card and motor overrides; Batch holds the generic scenario and study runners; Tuning is the tuning loop.
- Flash tab (2026-09-22, `px4/firmware_images.py`, `/api/flash*`): board and USB state, one card per image (Nose
  lift from `~/PX4-nl`, HITL stock from `~/PX4-hitl`) with Build and Flash, a progress bar and the job's full output
  streamed live; flashing starts `usbipd attach --auto-attach` (the bootloader is a new USB device), releases the
  serial port and reconnects to whichever ttyACM the board comes back on. "Check the board" runs `ver all` and
  `nose_lift status`. Stop is refused while the board is being written.
- Tuning attempt card: name, scenario, physics, seed, Park and Hover pitch overrides (placeholders show the
  scenario's own values), PX4 parameter table prefilled with the current gains, Run headless / Fly live / Apply.
- Sweep card: parameter, min, max, levels rows; runs = product of levels; progress and best score while running.
- Flights: attempts, live flights and sweep groups (expandable, best first) in one table; charts underneath with a
  "flight only, +-5 deg" toggle so hover tracking is readable next to the 20 to 35 degree rotation.
- HTML reports for sharing: `python scripts/report_runs.py --out r.html --study results/<sweep> results/*.json`.

## UX Principles

- The scenario, not a hidden default, states what the flight is: park pitch, hover pitch, phases, params.
- Numbers first, pictures second: every table cell is a metric with a tooltip saying what it is.
- Never redraw under the user's click: lists re-render only when their data changed.
- Failures are sentences ("parked at -5 deg the CG is 0.001 m ahead of the front feet: this stand tips over"), not
  crashes of the model.
- Headless work never touches the live simulation; live work never starts while a scenario is already running.

## Open UX Questions

- Remote-switch takeoff and land sequences for Stabilized in HITL (what the switch does while the pilot holds sticks).
- Whether the Tuning tab should offer position-hold sticks for the scripted pilot (a pilot trims drift; hands-off
  acceptance punishes an authority limit the pilot would never notice).
