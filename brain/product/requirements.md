# Product Requirements

## Functional Requirements

- Edit an airframe (mass and CG, rotors with positions, thrust axes, duct axes and jet turning loss, wings by strip
  theory, individual landing legs, body drag) and push it to PX4 as control-allocation parameters.
- Fly it live against PX4 SITL or a Pixhawk over USB (HITL), with a USB remote, in Manual, Stabilized, Altitude,
  Position, Takeoff and Land.
- Run scripted scenarios headless with metrics per phase; run many in parallel; run optimisation studies over
  parameter paths (geometry and `px4.*` parameters) with an objective and constraints.
- Every scenario for ATLAS runs the full ground flow: rotate from the parked pitch to the hover pitch and hold until
  stable, arm, fly, land at the hover pitch, rotate back down. The scenario may own the two angles (`attitude`
  block); the stand is re-solved under the same hard points and must not tip.
- A scripted pilot for manual modes: throttle hand that holds or slews a height, sticks otherwise centred, landing
  ends on ground contact.
- Tuning from the app: single attempts (headless or live), grid sweeps over PX4 parameters, a flight library with
  charts of height, attitude against PX4's setpoint, yaw, rates, position and motor use, phases shaded.
- Record PX4's attitude setpoint and attitude estimate in every time series; report tracking error and estimator
  bias per phase.
- Export a `.params` file for the board; verify seeded parameters after boot.

## Non-Functional Requirements

- Headless speed: about 5 to 7x real time per worker for the ten-fan airframe with a wing; a 60 s sequence in
  under a minute of wall time; six workers in parallel on PX4 instances 1 to 9.
- The simulation loop never blocks on MAVLink replies in lockstep.
- Results are plain JSON; the metrics dictionary is a stable contract for studies and the UI.
- The UI is vanilla JS with vendored three.js, no build step.
- Physics cost about 85 us per sub-step for ten rotors plus a wing; avoid `np.cross` on small arrays.

## Constraints

- Runs only inside WSL Ubuntu-24.04 (`~/utopia/AIRFRAME_DESIGNER`, interpreter `~/.venvs/airframe/bin/python`);
  PX4 v1.17 at `~/PX4-Autopilot`, a HITL build at `~/PX4-hitl`.
- Port 8080 and PX4 instance 0 belong to the interactive app.
- Structural frame FRD, origin at the reference point, CG explicit; PX4's body frame is the structural frame
  pitched by `hover_pitch_deg`.
- PX4 parameter seeding must match the firmware version and `SYS_AUTOSTART`, or PX4 resets everything silently.

## Success Criteria

For a configuration to be called ready for a hardware test, in the Stabilized lab sequence:

| measure | target |
|---|---|
| hover attitude tracking error, roll and pitch | under 0.5 deg RMS |
| hover yaw drift over 12 s | under 5 deg |
| hover position drift over 12 s, hands off | under 1.5 m |
| largest attitude excursion at liftoff and landing | under 2 deg |
| touchdown speed | under 0.5 m/s |
| motor saturation over the flight | under 1 percent |
| nose lowered onto the legs after touchdown | yes, no tip-over |

ATLAS_OG with flat brackets meets all of these. ATLAS_09B meets all but the drift, which is an authority limit.
