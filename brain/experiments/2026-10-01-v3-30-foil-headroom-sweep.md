# 2026-10-01 V3_30 foil angles for thrust headroom (CFD-calibrated)

Run `results/v3_30/foil_headroom/20261001_165843/` (gitignored), script `scripts/v3_30_foil_headroom.py`, report
`docs/V3_30_Foil_Angles_20261001.pdf`. Every result row carries an ISO timestamp (user asked; for the CFD guy).

- Jet model per foil fan fitted to the single CFD point (75/65/50 -> (-56.12, 46.48) N per group): jet angle = k x foil
  angle (k ~0.63), loss A linear / B quadratic / C flat 32 %. 3 x 2197 triples on 30-90 deg, PX4 pinv trim.
- Spare moments by linear programme (other 5 axes held). The pinv-direction margin gave fake ~50 N m yaw for equal
  foil angles: equal angles make the effectiveness matrix rank 5 (yaw not independent).
- **Foils cannot buy thrust headroom**: group force ~73 N whatever the split; busiest fan 92.9 -> ~91.7 %. They buy
  control: pick **inner 35 / middle 75 / outer 70** (alt 65/45/75): roll spare 2.5 -> 5.1 N m, yaw 0.53 -> 0.70; SITL
  (fastlift scenario, 3 seeds) drift 29 -> 5 m, bounce 5.6 -> 1.3 deg, touchdown 0.68 -> 0.18 m/s.
- +-5 deg on one station can push the busiest fan to 96-98 %: tolerance +-2.5. Real headroom: -1 kg ~ -6.5 pts,
  batteries 12 cm forward ~ -4.5 pts.
- Open: CFD of 35/75/70 with per-fan forces; a second CFD point decides A/B vs C.

## Limits added: hover <= 20 deg and rotation from the ground <= 20 deg (user, 1 Oct 2026 17:25)

- Live flight of 35/75/70 (app, fastlift scenario): done, drift 5.5 m, touchdown 0.18, but hover 46 deg: "too steep".
- Foils cannot reach it: lowest controllable hover in the sweep is ~40 deg (the jet turns only ~63 % of the foil).
- Target (`scripts/v3_30_jet_target.py`, run `results/v3_30/jet_target/20261001_172609/`): group jet **60 deg above the
  fan axis** (per group X -36.4, Y +63.1 N; stations 45/69/66), thrust kept 67.5 %, **parked level** (front strut
  325 mm vs 100, rear legs 949 vs 985 mm): hover 19.5, rotation 19.5, busiest fan 70 %, roll spare 11.4 N m;
  SITL 3/3 drift 1.2 m, saturation 0. 65/-4 and 70/-8 also fly. Ground wobble 4-5 deg (tune later).
  `airframes/atlas_v3_30_jet60_park0.json`. PDF `docs/V3_30_Foil_Angles_20261001.pdf` leads with this.

## Park -10 (hover <= 10 from the 20 deg rotation limit), 1 Oct 2026 17:35-17:50

- `scripts/v3_30_jet_sweep.py --park -10`: 9261 station jet triples 40-90 step 2.5, thrust kept 67.5 % and 60 %;
  run `results/v3_30/jet_sweep/20261001_173459/` (ranker `results/v3_30/jet_sweep_rank.py`).
- Needs the group jet ~76 deg up (today 39.6). Busiest fan flat at 67-72 % over most passing sets, so picked the
  smallest turn that keeps hover <= 10 at +-2.5 deg build error: **57.5 / 85 / 85** (in/mid/out; per group
  X -17.3, Y +68.9 N), alt 82.5 / 62.5 / 82.5. Hover 8.5, rotation 18.5, front strut 198 mm, rear legs 972 mm,
  roll spare 12.9 N m; SITL 3/3 drift 2.7 m. Ground wobble 6-8 deg (lift-off tuning).
  `airframes/atlas_v3_30_jets_57.5_85_85_park-10.json`; PDF section added.

## Live drift of the park -10 design (open), and Position mode (1 Oct 2026 18:30-18:54)

- Live Stabilized drift 12-45 m vs 3 m headless. Cause found as a mechanism, not a root: live PX4's estimate misses
  ~5 % of the on-ground rotate-up (lag ~120 ms; rotate-down tracked fine), leaving a steady ~0.3 deg est-vs-truth
  tilt in hover. Not the stock-SITL params (multi-EKF etc.), not SDLOG_MODE, not pre-flight settling (a settle gate
  was added to wait_ready anyway, `_estimator_settled`, test in tests/test_sim.py). Next suspect: how the live app
  sends HIL_SENSOR during the nose-lift (simulator.py step / lockstep).
- Instance 0 now has EKF2_MULTI_IMU 0 / SENS_IMU_MODE 1 / IMU_GYRO_RATEMAX 400 (headless and PX4 defaults); pushing
  the stock SITL multi-EKF back gave "ekf2 missing data".
- Position mode on H-FLOW, scenario `2026-10-01_1851_v3_30_park-10_jets_57.5_85_85_poshold_hflow` (live
  live905874558): held within 0.39 m for 60 s at 1.7 m, roll err 0.06, touchdown 0.35 m/s. Flow nulls the bias.
- H-FLOW mount fixed 20.4 deg looks ~12 deg off vertical at the 8.5 deg hover: re-angle to the hover pitch in CAD.

## Board prepared for HITL of the park -10 design (2026-10-01 19:16-19:25)

Backup before: `results/board_params/params_20261001_191629.json` (old V3 v34 -16 set, SENS_BOARD_Y_OFF 23.85,
EKF2_MAG_DECL 5.04 saved, EKF2_MULTI_IMU 3). Pushed the HITL export of `atlas_v3_30_jets_57.5_85_85_park-10.json`
with flow enabled (geometry, SENS_BOARD_Y_OFF 8.5, MPC_THR_HOVER 0.592, MC_PITCHRATE_P/I 0.9/0.05, NL_* geometry,
H-FLOW EKF2 offsets, EKF2_RNG_PITCH -0.2077 = mount 20.4 vs hover 8.5) plus EKF2_DECL_TYPE 0, EKF2_MAG_DECL 0,
EKF2_MULTI_IMU 0, SENS_IMU_MODE 1 (NL_EN 0, COM_RC_IN_MODE 1, SENS_IMU_AUTOCAL 0, UAVCAN_SUB_FLOW/RNG 0, MC_AIRMODE 0
already). 190/190 verified after reboot; checklist all green. NL_HOV_PITCH / NL_TGT needed a second set (the first
push's values did not survive the rotation-change reboot). Before real flight restore: SYS_HITL 0, NL_EN 1,
COM_RC_IN_MODE 0, SENS_IMU_AUTOCAL 1, EKF2_MULTI_IMU 3 / SENS_IMU_MODE 0 (6X redundancy), EKF2_DECL_TYPE 3.
HITL flight (live907599734, 19:20, COM_RC_IN_MODE 1 for the flight): Position hold max 0.42 m (SITL live 0.39),
alt 1.68-1.70, roll err 0.12 (0.06), est-truth pitch +0.45 (+0.41), rates 0.82 deg/s (0.75), touchdown 0.39
(0.35), fans max 69 %, no failures. HITL now matches live SITL; both share the ~0.4 deg live estimate offset.

## Board set for real use with the nose-lift switch (2026-10-02)

Written over USB (`results/board_params/write_nose_switch_20261002.py`, 20/20 verified after reboot; backup before:
`params_20261001_211932_readonly_decoded.json`): SYS_HITL 0, COM_RC_IN_MODE 0 (RC arm ch 8 / kill ch 5 live), NL_EN 1,
NL_FLY_HOLD 0, NL_HO_THR 0.15 (was -0.01: instant handover at zero throttle), NL_TGT / NL_HOV_PITCH 8.5, NL_RC_CH 7
(on below 1300), COM_DISARM_PRFLT 120 (longer than NL_HOLD_TOUT 60 so it never disarms mid-hold), COM_DISARM_LAND 2,
real H-FLOW on (UAVCAN_SUB_FLOW/RNG 1, EKF2_OF_GYR_SRC 1, EKF2_RNG_PITCH -0.2059 for the 20.3 deg mount), 6X IMU
defaults back. Firmware facts (PX4-nl NoseLift.cpp): ONE switch; on = lift to NL_TGT and hold; off while lifting /
holding = lower to the start pitch then disarm; after a flight the firmware lowers only in NL_FLY_HOLD mode
(nose fans hold the pitch all flight; NL_F_TGT was 26 from v34, untested for this design). Kill: PX4 cuts outputs in
every state, NL latches Aborted. RC was not connected during the write (input_rc lost): switches not yet verified live.
H-FLOW live check (2026-10-02 00:20): DroneCAN node 125 OK on CAN1, flow ~63 Hz quality 99-104, range 0.27 m.
Written and saved: EKF2_OF_DELAY 0 -> 20 ms, EKF2_RNG_DELAY 0 -> 5 ms (sim values were 0), SYS_HAS_NUM_OF / _DIST
0 -> 1 (no arming without the H-FLOW). Open: EKF2_IMU_POS still 0 (Pixhawk offset from CG needed), SENS_FLOW_ROT 0
unverified (push-forward sign test), mount 20.3 deg looks 11.8 deg off vertical at the 8.5 deg hover.

## First real nose-lift tests, logs 303-312 (2026-10-02 00:54-01:08, pulled over USB to results/flight_logs/)

Rotate switch works on ch 7 (1011 = on); lifts from -12..-19 to 8.5, holds 8.2-8.5 deg; switch-off lowering and
kill both worked. Log 312 (takeoff attempt, Position mode): hold reached 14.45 s; stick crossed NL_HO_THR 15 % at
14.46 -> handover; stick rose slowly to max 55 %, below Position mode's climb threshold (centre + deadzone ~60 %), so
PX4 never spun anything up (own commands 0, thrust sp 0); NL_HO_TOUT 8 s expired at 22.46, the hold faded over
NL_FADE_S 2 s and the nose fell 8.5 -> -12 deg (22.5-23.5 s). "Takeoff detected" at 23.0 was the drop, not a
lift-off. Fix options: NL_HO_THR ~0.65 for Position/Altitude take-offs (handover only once PX4 climbs), longer
NL_HO_TOUT, or take off in Stabilized (stick = thrust, hover ~59 %). Also seen: "Strong magnetic interference",
"heading estimate not stable" preflight fails.
Written 2026-10-02: NL_HO_THR 0.15 -> 0.65, NL_HO_TOUT 8 -> 15 s (saved, verified; results/board_params/write_nl_handover_20261002.json).

## First real flight, log 313 (2026-10-03 17:29, results/flight_logs/log_313_*.ulg + dashboard run_20261003_172929)

Lift -15.8 -> hold 10.2 (11.9 s); throttle > 65 % at 18.41 s -> handover, takeoff at 18.46, "PX4 took over" 20.50.
Altitude mode: held hover-frame pitch ~0 but accelerated forward to 1.95 m/s (heading ~-170, vx -1.95) and yawed
-6 then -13 deg/s while PX4 asked for +yaw up to MC_YAWRATE_MAX 60 deg/s (torque sp rising, allocator "achieved").
Position at 20.007 s: commanded 1.2-1.5 m/s^2 deceleration = +13..15 deg nose-up (correct), slowed only to 0.7 m/s
while holding +10..12 deg -> the real hover balance is ~20+ deg structural, not 8.5 (jets not 57.5/85/85). Rear fans
split left 0-45 % / right 87-100 % (M2 pinned 100 % from 21.25 s, M1 ~0) fighting roll/yaw; sank 0.7 m/s, pitch
request to +34 deg, allocation saturated at 22.5 s, roll 25 -> 38 deg, kill 23.18. Flow only fused from 20.8 s.
Model gaps: board CA_ROTOR*_KM +0.002 = all CCW (user: fans spin CW); sim and export put km along the turned jet
axis, but a jetfoil fan's reaction torque acts about its duct axis (x, roll). Output map changed again by the user:
MAIN1..6 = M3 M2 M1 M6 M5 M4, MAIN8 = M9, AUX2 = M8, AUX4 = M7.
Hover pitch set to 15 deg from the dashboard (2026-10-03 17:51): 44 params (SENS_BOARD_Y_OFF, NL_TGT, NL_HOV_PITCH, CA_ROTOR0-8 PX/PZ/AX/AZ, EKF2_RNG_PITCH, OF/RNG POS X/Z) written, saved, rebooted, 44/44 verified by read-back.
