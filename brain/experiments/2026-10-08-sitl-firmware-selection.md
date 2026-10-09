# 2026-10-08 SITL firmware + parameter selection (for HITL next)

Runner `scripts/fw_select_sweep.py` (board parameters from the reconstructed 7 Oct dump, simulated H-FLOW on,
sticks centred, physical nose-fan thrust swept while PX4 and the module believe 36 N). Results and per-run files in
`results/fw_select_20261008/` (summary_A.json, summary_C.json).

CORRECTION to `2026-10-07-sitl-board-state-nose-thrust-sweep.md`: the sim's `util_max` is normalised thrust
(actuator_motors), the dashboard / module `nl_cmd` is the motor command (thrust ~ cmd^2). The module command at hold
in SITL is 0.72 (36 N), 0.79 (30), 0.82 (28), 0.85 (26), 0.87 (25). The aircraft held at nl_cmd 0.62 to 0.67, so its
nose fans hold the nose at LEAST as well as the 36 N model; the "real aircraft = 28 to 30 N" mapping was wrong.
What the model misses is the handover: on 7 Oct 15:38 the nose fans saturated while the rear fans were at 0.1 to
0.4, which the 36 N model never does. The thrust sweep is therefore a margin sweep, not a match to the aircraft.

Firmware A = the board's (pre-first-test code). Firmware C = yesterday-evening code (PX4 Hold/Land, anti-windup,
stuck abort, rate filter, overspeed fix) + two new guards:
- NL_AUTO_HCMD (0.83, motor command): at the end of the wait, a nose command averaged over 1 s above it refuses the
  automatic takeoff and lowers the nose ("nose fans too weak").
- NL_AUTO_HDROP (5 deg): in the first 3 s of the automatic climb below 0.3 m, a nose more than this under the target
  cuts the automatic throttle (0.5 s) and lowers the nose (the 15:38 failure).

| candidate (seed 0) | 36 | 30 | 28 | 26 | 25 | 23 N |
|---|---|---|---|---|---|---|
| A, ramp 0.5 | PASS | PASS | PASS | - | CRASH | NOGO |
| A, ramp 2 | PASS | PASS | PASS | - | CRASH | NOGO |
| C no guards, module hover, ramp 0.5 | PASS | PASS | PASS | CRASH | pass (0.33 m) | NOGO |
| C guards, module hover, ramp 0.5 | PASS | PASS | NOGO (HDROP) | NOGO | NOGO | NOGO |
| C guards, PX4 Hold, ramp 0.5 | PARTIAL | PARTIAL | PARTIAL | NOGO | NOGO | NOGO |
| **C guards, module hover, ramp 2 (WINNER)** | PASS | PASS | PASS | NOGO | NOGO | NOGO |
| C guards, PX4 Hold, ramp 2 | PARTIAL | PARTIAL | CRASH | NOGO | NOGO | NOGO |
| C guards, PX4 Hold, ramp 0.5, I_MAX 20 | PARTIAL | PARTIAL | PARTIAL | NOGO | NOGO | NOGO |

PX4 Hold/Land: never completes (module still "flying" 90 s later: the PX4 Land -> landed hand-back does not work)
and once crashed: excluded. Winner seeds 1 and 7 at 36/32/30/29/28/27/26 N: PASS down to 28, NOGO at 27 and 26,
no crash (one seed-7 run lost to a PX4 start-up timeout, rerun PASS). Hands-off forward drift 1.3 to 1.9 m/s at
30 to 28 N; peak height 1.7 to 2.05 m for NL_AUTO_ALT 1.0.
Winner: image `results/fw_select_20261008/px4_fmu-v6x_multicopter_C_gates_20261008.px4` (sources src_C/),
parameters `winner_C_gate_mod_r2_params.json` (NL_AUTO_HCMD 0.83, HDROP 5, HOLD 0, RAMP 2, SLEW 0.3, I_MAX 40,
Q_FC 50, STUCK_S 10, MAX_CMD 1, ALT 1.0, HOV 5). The board still runs firmware A. `firmware/px4_ext` holds C now;
A's sources are in `results/board_firmware/src_oct6state/`.
Not modelled: the legs' rocking (so NL_Q_FC cannot be judged here) and the real handover moment.
