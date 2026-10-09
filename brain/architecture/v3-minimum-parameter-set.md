# V3 six-phase flight: the minimum parameter set (2026-10-08)

Question (user): fewest PX4 parameters to rotate to 8.5 deg without jitter, spin up the rear fans with the nose held,
climb vertically to 1.5 m smoothly, hover still, descend, touch down, rotate back.

Answer: 18 dials plus 4 strategy switches (17 of the 22 differ from the board today), on top of ~10 definition parameters the board already has right
(SENS_BOARD_Y_OFF 8.5, NL_HOV_PITCH 8.5, NL_TGT 8.5, NL_MOT_MSK 448, CA_ROTOR*, MPC_THR_HOVER 0.592, NL_WEIGHT/PIV/A*,
EKF2_OF_CTRL 1 / RNG_CTRL 1 / HGT_REF 2, NL_EN, NL_RC_CH, kill on ch 5).

| phase | dials (latest SITL value) |
|---|---|
| 1 rotate | NL_RATE 3, NL_KQ 0.02, NL_Q_FC (50 = raw rate; 2 to 5 filters the 10-15 Hz leg rocking but lags, real aircraft only) |
| 2 handover | NL_AUTO_RAMP 4, MC_PITCH_P 4.0, MC_PITCHRATE_D 0.012 |
| 3 climb | NL_AUTO_ALT (1.0 tested, user wants 1.5: untested), NL_AUTO_VUP 0.3, NL_AUTO_PACC 0.3 |
| 4 hover | MPC_XY_P 1.5, MPC_XY_VEL_P_ACC 2.4, MPC_XY_VEL_I_ACC (0.4; j9 tests 1 and 2), MPC_ACC_HOR 2, MPC_JERK_MAX 2 |
| 5 descend | NL_AUTO_VDN 0.3, NL_AUTO_VTD 0.05 |
| 6 lower | NL_LOW_RRAMP 1, COM_DISARM_LAND -1 |

Switches: NL_AUTO_HOLD 1, NL_AUTO_PCLB 2, NL_AUTO_PDSC 1, NL_AUTO_TD 3 (with NL_AUTO_RNG 1, NL_HO_LIFT 1).
SITL j8 `vup03_xy_acc2` (= j9 base): 8/8 pass, hover drift 0.10 m, height std 3 mm, 36..28 N nose fans.

Not fixable by parameters: vibration (1P fan imbalance, [vibration model](vibration-model.md)); hands-off drift in
Stabilized (no position loop by definition) and the same-spin yaw limit
([lesson](../lessons/v3-hover-pitch-sensitivity-is-yaw-authority.md)); nose-fan pack sag (NL_AUTO_HCMD only refuses).
Most NL_AUTO_P*/HO_* parameters exist only in the uncommitted `firmware/px4_ext` module, not on the board (firmware A).
