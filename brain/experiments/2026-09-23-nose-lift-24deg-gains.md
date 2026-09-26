# Experiment: nose-lift gains for NL_TGT 24 (SITL, aircraft-bracketing fan models, 2026-09-23 evening)

Firmware: the flashed NL_CEIL build (~/PX4-nl). Park 5, rocking 0.35 rad/s, fans tau 0.15 s up. Scenarios
`fw_nose_lift_balance24` (30 s at 24 with the throttle at zero, then SB off) and `fw_nose_lift_midcancel24` (SB off
as the nose passes 15 deg on its way up, the 17:49 tip-back case). Script `scripts/nose_lift_smoothing.py --what
hover24 | fit24 | robust24 | confirm24`, results in `results/nose_lift_smoothing/<what>/`. The model's CG passes over
the rear feet at 51-54 deg, so at 24 gravity still pulls the nose down (the fans need about 0.5 of full): a
nose-fan-only balance at 24 is possible. The model has no tail contact: past ~51 deg it tumbles ("crashed"), where
the aircraft lands on its tail at 40-49.

## The model was optimistic (hover24 vs the aircraft's 24-deg runs, 17:21-18:01, NL_TGT 24 then)

| gains | aircraft | fans down 1.0 s |
|---|---|---|
| G4, raise | peaks 27.2-28.6, 8->14 deg at 4.2-5.7 deg/s | 25-26 |
| G4, cancel at 17-18 / at 26 | 20-23 / 43 | 15-17 |
| 17:49 set (KQ .03, LOW .018/.009), cancel at 11-14 | 41-49 (tail) | 16-23 |

In the 17:50:48 log the nose sped up after the cancel (14 -> 28 deg in 1.3 s) while the command fell 0.69 -> 0.

## Fit (fit24) and bracket

A slower spin-down is what matters (thrust exponent 1.2-2 far less): tau_down 1.0 s is too optimistic, 2.0 s too
pessimistic (G4 tips on the way up, which the aircraft never did). The aircraft sits between 1.5 and 2.0. Gains
were judged on the worst case over tau_down 1.0 / 1.5 / 2.0 x NL_A right / x0.85.

## Result (robust24 seed 1, confirm24 seeds 2-3)

| set | tips | worst peak, balance | worst peak, cancel at 15 | at 24 | flips/s |
|---|---|---|---|---|---|
| C5 KQ .04 KQI .02, LOW .06/.03, K_ANG .3, RATE 1.5, CEIL 1 | 0 / 36 | 27.3 | 22.4 | 23.1-24.0 +- 0.9 | 8.4-8.5 |
| C6 KQ .05 KQI .025, LOW .06/.03, K_ANG .5, RATE 2, CEIL 1 | 0 / 36 | 29.0 | 20.5 | +- 1.7 | 9.5-10.2 |
| C1 board (KQ .03/.015, LOW .06/.03, K_ANG .4, RATE 3, CEIL 1) | 6 / 36 | tips | tips | +- 1.0 | 3.3-4.3 |
| G4 (no ceiling) | 1 / 12 | tips | 18 | | 10.5 |
| G4 with CEIL 1 | 4 / 12 | tips | 18 | | 6.8 |

Every smooth KQ 0.03 set tips in the pessimistic models: too little rate gain to brake the slow fans. C5 is the pick
for 24: from the board today NL_TGT 12 -> 24, NL_KQ 0.03 -> 0.04, NL_KQI 0.015 -> 0.02, NL_K_ANG 0.4 -> 0.3,
NL_RATE -> 1.5 (LOW 0.06/0.03 and NL_CEIL 1 unchanged). Price: it bursts like G4 (8.5 flips/s). Do not set NL_TGT 24
with the board's current gains. Not tested: a cancel above 24 (the 17:21 G4 case), parks other than 5.

On the board 2026-09-23 ~20:55 (radio, disarmed, read back): NL_TGT 24, NL_KQ 0.04, NL_KQI 0.02, NL_K_ANG 0.3,
NL_RATE 1.5; lowering 0.06/0.03 and NL_CEIL 1 unchanged.
