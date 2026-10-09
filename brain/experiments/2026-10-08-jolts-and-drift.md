# 2026-10-08 evening: every jolt and the hover drift (user: "fix every abrupt shake and the drifting; don't stop")

Targets set: no pitch/roll rate above ~5 deg/s anywhere in the flight at 36/32/30 N; drift from the lift-off point
<= 0.2 m at 36 N, <= 0.5 m at 30 N. Packs j1..j11, mount, final2, lift in results/sitl_pack/ (scripts/sitl_pack.py now
also scores per-segment jolts: handover/liftoff/climb/hover/descent/touchdown/lowering, and drift_total; time series
carry n_est/e_est/vn_est/ve_est = PX4's own estimate).

Findings and firmware changes (all in firmware/px4_ext nose_lift, params.c documents each):
1. Climb in Stabilized drifted 3.7 m at 30 N and PX4 Position's braking at the hover was the worst jolt (20-38 deg/s):
   NL_AUTO_PCLB (Position climb with a climb-rate stick; stick_for_climb() inverts PX4's expo/deadzone stick map).
   PCLB 2 = Position as soon as the throttle ramp reaches the hover thrust, still on the legs (best). NL_AUTO_PTKO
   (Position takeoff from the legs) rejected: PX4 steps the thrust 0 -> hover (land detector already "not landed").
2. The estimator height crept +0.3..0.4 m per flight (vibration): the "1 m" hover was 0.64 m and the touchdown cue
   never fired. NL_AUTO_RNG: height above takeoff = range finder - its takeoff reading.
3. Landing: NL_AUTO_PDSC Position descent (VDN easing to NL_AUTO_VTD at NL_AUTO_HTD), VTD 0.05 m/s; touchdown cue below
   6 cm (range) with stillness < half the creep speed; bumpless lowering from actuator_motors (nose_lift_feedback is
   only published while the module overrides: it was stale and the lowering started every motor from 0, +-20 deg/s);
   rear fans fade over NL_LOW_RRAMP 1 s. Switch-off under Position starts the same gentle descent.
4. Handover on the legs: PX4 lets the nose sag ~2 deg while the rear thrust ramps, then snaps it back at lift-off.
   NL_HO_LIFT keeps the module's floor until NL_HO_LIFT_H 0.06 m off the legs (0.03 fires on the leg springs).
   NL_HO_FF (rear-thrust feed-forward) > 0 crashed: keep 0.
5. SIM HARNESS BUG: design.nose_lower (Python) took over every touchdown above 1 m CG: lessons/sim-python-nose-lower-
   overrides-firmware.md. Disabled since; earlier touchdown/lowering numbers of high hovers were the simulator's.
6. Remaining drift 0.25 m at 36 N is the H-FLOW MOUNT: 20.4 deg fixed = 11.9 deg off vertical at the 8.5 hover; climbing
   reads as forward flow (vz*tan 11.9 = 0.06 m/s at 0.3 m/s): PX4's estimate stays at 0.03 m while the truth goes
   0.25 m forward (reverses on the descent). PX4 has no flow pitch rotation (SENS_FLOW_ROT is yaw); a software fix
   needs the EKF2 flow model. SITL with the mount at 8.5 deg: drift 0.03 (36 N) to 0.12 m (28 N). => re-angle the mount.
Params also changed: MC_PITCH_P 4, MC_PITCHRATE_D 0.012 (climb tremble), MPC_ACC_HOR 2, MPC_JERK_MAX 2, MPC_XY_P 1.5,
MPC_XY_VEL_P_ACC 2.4, MPC_XY_VEL_I_ACC 1.0, NL_AUTO_VUP 0.3, NL_AUTO_PACC 0.3, NL_AUTO_RAMP 4. Rejected: MC_*RATE_MAX
10 (crashes), MPC_POS_MODE 0/3 (more jolt on the legs), stiffer xy (more lift-off jolt at 28 N).

Final (final2, 3 seeds): 24/24 PASS, switch-off 4/4 (touchdown 0.05), kill 4/4. Worst jolt 4.9 (36 N), 5.8 (32 N,
a smooth 0.6 s nose correction at lift-off), 4.9 (30 N), 7.5 (28 N); hover height std < 1 cm; touchdown 0.05 m/s;
drift 0.26 m as built (hover 0.08), 0.03-0.12 m with the mount at 8.5 deg. Live app run: pass, flight pitch-rate max
4.4 deg/s. Image + sources + params: results/sitl_pack/final/ (final_params.json, firmware/*_final_v2_*). NOT on the board.
Run-to-run noise of the jolt metric under CPU load is +-1..2 deg/s: judge on 3 seeds. The 8080 app's PX4 (stock,
~2 cores) slows packs from ~20 to ~3 flights/min when busy.
