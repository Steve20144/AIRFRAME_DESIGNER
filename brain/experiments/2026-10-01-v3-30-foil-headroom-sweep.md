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
