# 2026-09-29 V3 v34 from Fusion: CG / battery placement, authority, H-FLOW placement

Model: `airframes/atlas_v3_v34*.json` built from the live Fusion design ([pipeline](../architecture/atlas-v3-fusion.md)).
Report for the CAD designer: `docs/v3_views/v34/V3_CAD_change_notice.html` (`scripts/v3_cad_change_report.py`),
published at https://claude.ai/artifact/FMpp72ZswAN2W8zxpsVkJX. 154 SITL flights.

## Battery placement (all 9/9 flights per layout, v3_stab_nolift at trim and ±0.5, seeds 1-3)

| layout | CG x FRD | trim | busiest fan | drift median / max |
|---|---|---|---|---|
| as drawn | -0.045 | 18.4 | 55 % | 1.86 / 2.40 |
| all six -130 mm (collides with V6X/RC08 stack) | -0.095 | 20.4 | 46 % | 0.95 / 1.38 |
| front pair into rear block | -0.083 | 19.8 | 48 % | 2.10 / 2.72 |
| front pair into rear block, block -45 mm | -0.101 | 20.5 | 47 % | 1.49 / 1.92 |
| **rear block of four -195 mm (Fusion Z +195), outer two 10 mm lower (clear of All_mounts)** | -0.095 | 20.4 | 47 % | **0.53 / 0.73** |

`airframes/atlas_v3_v34_recommended.json`. The geometric middle (x -0.047) is where the CG already is; the
load-balanced point is ~50 mm aft because the foil thrust acts along the exit jets, far aft and low. Static loads do
not rank flights alone (front-pair layouts load the fans like the best one but drift 1.5-2.1 m: the known hover-pitch /
yaw-authority sensitivity); always fly.

## Authority (recommended layout)

LP at hover (`results/fusion_v3/authority_lp.py`): roll ±10.5 N·m (~915 deg/s²), pitch ±20 N·m (~1240), yaw ±3.3 N·m
(~155): yaw ~6x weaker. Flown (`scenarios/v3_authority.json`): half stick reaches ±5.5 of 6 deg; full yaw 38 deg/s right,
33 left of 60 commanded; `v3_gust.json` 4 m/s crosswind swings heading 28-31 deg, fans 62 %. Inner foil steeper
(72, 75 deg) was no better / 2x worse drift: keep 69.

## H-FLOW (new flow + range sensor model, `scenarios/v3_flow_hold.json`, GPS off, Position hold 20 s)

Six seeds on the recommended airframe: CAD spot flat (as drawn) 0.38 / 0.65 m, height noise 1.4 cm; CAD spot tilted
to the hover frame (20.4 deg) 0.28 / 0.44, 0.6 cm; nose (FRD 0.30, 0, 0.077) tilted 0.26 / 0.48, 0.6 cm; GPS 0.07.
Tilt is what matters; location does not in the sim (seed scatter 0.05-0.5 m is larger than belly vs nose). A 3 cm
EKF2_OF_POS error changed nothing. Jets in the 42 deg view: belly spot clear up to ~1.1 m hover (inner foil jets M5/M6
enter above), nose clear up to ~1.8 m; the nose spot sits under ESC__M200g:2. Recommendation: tilt in place (A), nose
(B) if hovering higher. The model has no dust, texture or lens vibration.

## Labels (the mass model ignores unlabelled parts)

Unlabelled: All_mounts (909 cm³), Four_feet_assembled, clamp caps, right jetfoil mounts (STEP translator 1.9/1.11
Mirror). Foils labelled 920 vs 800 g (same volume). 9 unlabelled duplicates sit on labelled parts. Lateral CG -4 mm
from these.

## Infra

Batch runs occasionally fail with "PX4 did not connect" / "never became ready to arm" (startup race, ~2 %); re-fly
those tasks alone. `results/fusion_v3/summarise.py` skips status "error" runs.

## Ground rotation flight: `scenarios/20_deg_v3_rotate_fly.json` (2026-09-30)

Recommended airframe parked nose-down at -8 deg (legs solved for -6.2: they settle 1.8 deg under 11.8 kg), nose fans
M7-M9 (0-based 6,7,8) lift it to the 20.4 trim at 3 deg/s, Stabilized lift-off to 2.5 m, 12 s hover, descent 0.4 m/s to
0.7 m then a 0.15 m/s flare, cut, nose lowered back to -8 at 3 deg/s with a 4 s fade. Seeds 1-3 all OK: up in 10.6 s
(nose fans max 72 %), hover drift 0.64-0.74 m, touchdown 0.17-0.18 m/s, lowering peak 3.0-3.8 deg/s. Without the flare
touchdown was 0.39 m/s with a 7-9 deg/s jolt as the feet hit. Results in `results/20_deg_v3_rotate_fly/`.

## Foils 50 / 65 / 50 (exit angle outer / middle / inner) on the recommended layout (2026-09-30)

`airframes/atlas_v3_v34_foils_50_65_50.json` (lean 40 / 25 / 40; rotors kept on the v34 exit lines). Trim 23.85 deg.
Static: busiest fan 50 % (47), authority roll -12 %, pitch -35 %, yaw -18 % vs 69/59/49. Headless 9/9 OK but hover
drift 1.32 / 1.62 m (0.57 / 0.75 on 69/59/49), pitch error 0.25 (0.05), manoeuvre fans 70 % (64), yaw 35/31 deg/s.
Flyable, worse on every measure: keep 69/59/49. Live app run: 4.14 m drift with a 0.37 deg pitch estimator bias (the
known live-app gap). Results in `results/foils_50_65_50/`.

## The flight board now carries the V3 parameters (2026-09-30, from macOS over USB /dev/cu.usbmodem01)

Board: Pixhawk 6X, PX4 1.17.0 custom nose-lift build, git d6f12ad1c4f7, built 25 Sep 2026 21:41 (identity in
`results/board_firmware/board_id_*.json`; the image itself cannot be read back over USB, the .px4 file is on the
Windows machine). Before: 09B set (10 rotors, SENS_BOARD_Y_OFF 24, NL on motors 9/10), and MC_PITCHRATE_P was 0.
Backups: `results/board_params/params_20260930_151741.json` (another board, same session) and
`params_20260930_152158.json` (this board, before). Uploaded `v3_upload_20260930_152249.json`: 146 params from
`airframes/atlas_v3_v34_foils_50_65_50_tuned.json` (9 rotors, board rotation 23.85, tuned pitch gains, V3 nose-lift
geometry NL_MOT_MSK 448 = motors 7-9, default NL gains), without HIL_ACT_FUNC*; outputs kept (Motor 1-8 MAIN1-8,
Motor 9 AUX4), PWM_AUX_FUNC2 (Motor 10) off. No full reset (calibrations kept). Rebooted, 146/146 verified.
Not done: motor order / direction check on the V3 (ESC order unknown), NL gains for V3, real foils are 69/59/49 in CAD.

## Board firmware built on macOS (2026-09-30)

Toolchain: Arm GNU 13.2.Rel1 darwin-arm64 in ~/toolchains (the board's own build used GCC 13.2.1 20231009);
`~/.venvs/px4-build` needs setuptools<70 (libuavcan's DSDL compiler imports pkg_resources). Builds need
SDKROOT=.../MacOSX26.5.sdk and CMAKE_POLICY_VERSION_MINIMUM=3.5 like SITL.
- Stock HITL image (no nose lift): ~/PX4-Autopilot/build/px4_fmu-v6x_multicopter/*.px4 (scripts/build_hitl_firmware.sh).
- **Nose lift + HIL driver (the one to flash)**: `scripts/build_nose_lift_firmware.sh board build` in the ~/PX4-nl
  worktree -> copy in `results/board_firmware/px4_fmu-v6x_multicopter_noselift_hitl_*.px4`, 96.5 % flash, board_id 53.
  Its 57 NL_* params equal the flying board's (NL_CEIL included), so the module source is what flew. The worktree's
  first submodule fetch left NuttX apps and heatshrink empty (files "deleted" in the index): fixed with
  `git submodule foreach --recursive 'git reset -q --hard HEAD'`.

## Park at -17 deg and HITL arming (2026-10-01)

`legs_for_park_pitch` kept the mean leg length, so past about -12 deg the front leg went under 3 cm and it refused.
It now lowers the ground plane instead (shortest leg 3 cm, the others grow). On the tuned 50/65/50 airframe the
rest angle is the park setting minus ~1.85 deg (-6.2 -> -7.95, -15.2 -> -17.07). `scenarios/v3_park_m17_rotate_fly.json`
(rotate_fly with park and nose_lower target -15.2): headless OK, rotate-up 14.8 s with nose fans at 52 %, drift
0.64 m, touchdown 0.16 m/s, back to -17. Time series `pitch` is in the hover frame, in rad.
HITL: the board has EKF2_GPS_CTRL 0 and EKF2_OF_CTRL 1, but the simulator sends HIL_OPTICAL_FLOW / DISTANCE_SENSOR
only in SITL (`simulator.py`), so the board never gets a horizontal position. It arms in manual / acro / altctl /
stab only (see `commander_arming_check_summary`); Hold / Takeoff / Position are refused ("Resolve system health
failures"). The checklist's "board can arm (Takeoff / Hold)" step stays grey for that reason.

## HITL tuning session on the 6X (2026-10-01): setup faults first, gains were already right

-16 batch flown live on the board. Fixed on the way (app code): wait_ready accepts the mode the scenario arms in
(stab), neutral MAVLink sticks streamed while a stick-mode flight waits, HITL fresh start waits for the board to go
quiet and stream again (the USB link reopens before the watcher sees it drop), param count tolerance 3 %.
Board faults found: firmware nose lift (NL_EN 1) holds every motor until the RC switch (set 0 for sim-driven
flights); COM_RC_IN_MODE 3 let the real transmitter (throttle 0) win over app sticks (use 1); CAL_ACC3 (the
simulated accel, id 1310988) held a 0.14 m/s^2 Y offset saved by SENS_IMU_AUTOCAL from an earlier bad estimator
bias: 1 deg roll lean, 38 m drift. Zeroed (real IMU slots 0-2 untouched).
Gains: MC_PITCHRATE_P 0.75 best (0.65 and 0.9 both raise hover wobble 0.039 -> 0.06-0.07 deg).
Remaining HITL gap: no velocity aiding (EKF2_GPS_CTRL 0, flow only in SITL), so a drift's drag reads as tilt and the
estimate sits 0.05-0.3 deg off truth (drift 4-15 m in 60 s Stabilized). Sending the sim H-FLOW in HITL made it worse
(0.67 deg, 42 m; board H-FLOW params vs sim model unchecked) and was reverted. Board settings restored afterwards:
COM_RC_IN_MODE 0, NL_EN 1, COM_DISARM_LAND 120, COM_DISARM_PRFLT 120, SENS_IMU_AUTOCAL 1 (MC_AIRMODE stays 0).

## H-FLOW in HITL, first pass (2026-10-01)

Real H-FLOW: fixed to the frame tilted 20.4 deg, right below the Pixhawk = CAD spot [-0.015, 0, 0.062]
(`design.flow_sensor` mount "structure", extra_pitch_deg 20.4: 3.45 deg off body down in the 23.85 hover).
`flow.ekf2_params` is now part of `px4_params` when the sensor is enabled (offsets from the CG in the hover frame,
EKF2_RNG_PITCH 0.060 rad, delays 0, IMU_POS 0, HGT_REF 2); the simulator sends flow/range in HITL too. The board
receives it (distance_sensor 0.20 m on the legs, flow quality 255). Board CAN: UAVCAN_ENABLE 2 with traffic, but
UAVCAN_SUB_FLOW/RNG 0 (the real sensor is ignored; keep it so in HITL).
Open problem: fusing the simulated flow makes EKF2's heading jump 1-2 deg every ~0.5 s (yaw std 1.6 vs 0.1 deg,
yaw rate noise 0.9 vs 0.05 deg/s), in SITL as well as HITL, with any mount, at the CG, with baro or range height and
with GPS also on; flow sent but not fused (EKF2_OF_CTRL 0) is clean. The 2026-09-29 flow study already had it
(yaw std 1.0-1.2 vs 0.2-0.35 deg on GPS, rates rms 4.6-7.7 vs 1.3): a flow model / EKF2 consistency issue, not
the mount. HITL flight with flow: drift 42 m, roll err 1.0 (SITL same setup 5.3 m, 0.14).

## Flow model fixed (2026-10-01): the heading jitter was flow noise 7x too high

`flow.py` scaled flow_noise by sqrt(dt) as a density: at 50 Hz each reading carried 0.14 rad/s (0.37 m/s of apparent
motion at 2.6 m) instead of the documented 0.02 rad/s, and EKF2's heading jumped 1-2 deg per update. Now
noise * dt (test). Also the message now carries the sensor-frame gyro integral: left NaN, PX4 integrates the raw
sensor_gyro, which is NOT rotated by SENS_BOARD_Y_OFF (23.85 here), so yaw leaks into the x compensation. Real
aircraft: if the H-FLOW sends no gyro, set EKF2_OF_GYR_SRC 1 (EKF2's own, rotated gyro).
Results, -16 batch: SITL Stabilized yaw std 0.12-0.14 (GPS 0.11), drift 2-3.6 m; HITL Stabilized yaw std 0.18,
drift 13.6 m (flow fused, the board measures the slide but Stabilized does not hold position). Position-mode batch
`scenarios/2026-09-30_2136_..._rotate_poshold_hflow.json` (1.5 m, 60 s Position on flow): SITL drift 0.09 / 0.08 m;
HITL 0.50 m, rates 1.8 deg/s (SITL 0.7), tilt estimate 0.47 deg off truth (SITL 0.03), touchdown 0.52 m/s. The
earlier flow study (CAD report) ran with the 7x noise: its 0.26-0.48 m flow-hold drift is pessimistic.
