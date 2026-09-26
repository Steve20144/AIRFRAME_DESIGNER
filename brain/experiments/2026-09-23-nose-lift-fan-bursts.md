# Experiment: the nose-lift fan bursts on the aircraft (ATLAS_09B, 2026-09-23)

## Evidence

- Dashboard live logs `results/telemetry_runs/log_20260923_163917.csv` (park 1.9: 11 s at full command, the nose
  never rose, battery reading 7.85 V) and `log_20260923_163956.csv` (park 5.3: rose to 19.4 in 4.5 s, then SB off
  and lowered; board ULog `/fs/microsd/log/2026-09-23/22_59_07.ulg`). In the second the nose-lift command jumps
  0 -> 1 -> 0 within 0.1-0.3 s all through the raise (1.0 at 2.50 s, 0.0 at 2.73, 1.0 at 2.97, 0.0 at 3.07 ...),
  outputs 9/10 follow between 1100 and 1900 us; the lowering is rougher than smooth but less violent.
  The logs sample at 10 Hz over the radio: the real switching may be faster.
- Video IMG_1508 (an earlier flight, same behaviour): the fans' loudness swings 10-20 dB inside every 0.5 s with
  envelope peaks at 3.6-4.9 Hz, about 7 surges a second.
- Firmware: `cmd = thrust_to_cmd(ff + NL_KQ * (q_des - q) + NL_KQI * integral)`, q in deg/s. With the board's G4
  NL_KQ 0.10 the 5-10 Hz, +-20 deg/s rocking of the frame on its legs (log 184) is +-2.0 of command on a 0-1
  range: bang-bang. The slow fans turn every cut into a lag and every restart into a surge, which rocks the frame.

## SITL (`scripts/nose_lift_smoothing.py`, results in `results/nose_lift_smoothing/`)

- `repro`: the committed model (fans tau 0.12, no rocking) with G4 flies smoothly (0.14 flips/s). With the fans
  at tau 0.4 / tau_down 1.0, NL_A0/1 x0.85 and 0.35 rad/s of gyro rocking (SensorNoise.vib_gyro, 5.4 + 10.3 Hz)
  the command flips off <-> full 7-8 times a second (the video's ~4 Hz cycle), 64-70 % of the time at an end,
  and the nose never rises (park 2 and 5.3): too pessimistic, flight 2 rose.
- `fixes`: a feedback band around the feed-forward (NL_FB_BAND 0.25) with a command slew limit (NL_SLEW 2/s) tips
  the aircraft onto its back within 4 s (no braking authority against 1 s fan spin-down near the balance point);
  NL_Q_LPF 5 Hz alone overshoots and crashes (lag).
- `sweep` (NL_Q_LPF 0/2/3.5 x NL_KQ 0.03/0.06/0.10 x NL_K_ANG 1/0.4, park 2): nothing lifts; unfiltered never
  rises, filtered tumbles (150-450 deg/s).
- `fit` (tau 0.15/0.25/0.4 x tau_down 0.5/1.0 x NL_A x1/x0.85, park 5.3, G4): the nose stays at 5.7 in all 12,
  so the fan lag is not the missing piece. Likely the rocking: on the aircraft it is physical and partly caused by
  the surges, in the sim a gyro-only tone at full size whenever a fan runs.

CORRECTION (same day): "the nose never rises" above was a metric error: the phase ended after the lift timeout had
lowered the nose again. Re-read (nose max, time to 20 deg): the aircraft-like model rises like the aircraft and
bursts like it, and never reaches Holding because the hold check needs |q| < 3 deg/s, which the rocking never
allows. The aircraft agrees: none of its 18 dashboard runs (22-23 Sep) ever reached Holding, even at 26-37 deg.
Best fit to flight 2 (park 5.3 -> 19.4 in 4.5 s): fans tau 0.15 / tau_down 1.0, NL_A x0.85, rocking 0.35 rad/s
(3.3 deg/s to 20 deg, ~9.8 flips/s).

Corrected park-2 sweep: every NL_Q_LPF setting tumbles or overshoots (37-114 deg); unfiltered KQ 0.03 with K_ANG
0.4 cuts the flips 7x (0.9-1.0/s) with nose max 26-28.5; K_ANG 1 at KQ 0.03 overshoots to 36 deg (the aircraft's
log 174 over-speed with the old 0.02 gains). Park 5.3, fitted model: G4 9.8 flips/s; KQ 0.04 5.3; KQ 0.03 2.1-2.4
(rise 5.3-5.7 deg/s, nose max 26); KQ 0.02 0.2-0.3 (rise 6.8, nose max 30-31). K_ANG hardly matters there.

Firmware fix: the hold and lowering-fade checks use `_q_settled`, the pitch rate low-passed at 1 Hz (decisions
only; the control path is unchanged). Confirmation, seeds 1-3, parks 5.3 and 2 (`confirm_park*`):

| setting | cancel ok | raise flips/s | nose max | rise deg/s | lowering flips/s |
|---|---|---|---|---|---|
| G4 | 6/6 | 9.6-10.0 | 25.2-26.7 | 3.3-3.4 | 9.3-9.7 |
| KQ 0.03 KQI 0.015 LOW 0.018/0.009 K_ANG 0.4 | 6/6 | 1.3-1.8 | 24.7-26.0 | 5.4-5.9 | 0.04 |
| KQ 0.02 KQI 0.01 LOW 0.012/0.006 K_ANG 0.25 | 6/6 | 0.6-0.7 | 28.7-30.5 | 6.7-7.6 | 0.04 |

With the fix every takeoff reaches Holding, hands over and flies (none did before); all 18 then land at 2.3 m/s,
G4 included: the gyro rocking model stays on in the air (vib_gyro scales with fan speed, not with ground contact),
a model artefact. The faster rise comes from the x0.85 feed-forward the weak loop cannot hold back; with a true
NL_A it would be slower.

Conclusion (superseding the one below): KQ 0.03 / K_ANG 0.4 plus the settled-rate hold check is the candidate
for the board, to be tried with short raises first; the fit rests on assumed fan lag and underrate.

## What the board actually runs (checked 2026-09-23 over the radio, `ver all`)

Build datetime Sep 22 2026 22:49:36 PDT (WSL runs in America/Los_Angeles), PX4 v1.17.0 multicopter with
pwm_out_sim; `~/firmware_backups/nose_lift_board_2026-09-22_2249.px4` is that image (same build time). It was built
from uncommitted work between 219bb62 (15:49 PDT that day) and cae85cb: it has the clock-after-copy fix (tlog 21:34)
and the baro liftoff check but no NL_Q_LPF. Rebuilt today: 219bb62 = board - 272 bytes, cae85cb without NL_Q_LPF =
board + 80 bytes (4 % of bytes differ, shifted code), so the board's exact source is not in git; the nearest is
cae85cb without NL_Q_LPF, carrying ~80 bytes of later nose-lift code (probably a baro-check refinement). A first
reading of the build time as UTC ("219bb62, 8 s before its commit") was wrong. Board gains until 2026-09-23 17:40:
G4 (NL_KQ 0.1, KQI 0.05, LOW 0.06/0.03), NL_K_ANG 1, NL_TGT 24, NL_TOUT 25; then set over the radio to NL_KQ 0.03,
NL_KQI 0.015, NL_LOW_KQ 0.018, NL_LOW_KQI 0.009, NL_K_ANG 0.4 (read back). Flashing needs USB (the SiK radio cannot carry a firmware update);
parameters can be set over the radio through the dashboard's UDP relay (`scripts/board_params.py --port
udpin:127.0.0.1:14550`).

## The new gains on the aircraft (2026-09-23 17:49-17:51, dashboard logs log_20260923_1749*/1750*)

NL_KQ 0.03 / KQI 0.015 / LOW 0.018/0.009 / K_ANG 0.4 on the unchanged firmware. Raise: no off/full flips (G4:
0.7-1.3/s at 10 Hz), command 0.2-0.94 in 0.04 steps (G4 0.21-0.25), but it over-sped to 5-7 deg/s (G4 ~3, SITL
predicted 5.4-5.9). SB off at 14-15.5 deg while rising fast: the weak lowering gain and the slow spin-down let the
nose carry past the balance point to 45 / 48.5 / 40.8 deg, onto the tail with the fans at idle; two ended with the
kill switch, one came down over 20 s (rising again to 36 on the way). G4 cancels peaked at 20-21. SITL missed it:
its cancel switched off only after Holding at the target, never mid-rise at speed. All five gains restored to G4
over the radio at ~17:55 (read back). Next: a mid-rise cancel in SITL, calibrate NL_A0/1 from the aircraft's
lift-off command (0.94 at 5.2 deg park) so a gentle loop is not fighting an over-high feed-forward, then retry.

## A 12 deg balance with a 14 deg ceiling (SITL, park 5, aircraft-like model, both NL_A variants)

Scenarios `fw_nose_lift_balance12` (30 s on SB) and `fw_nose_lift_midcancel12` (SB off 2.5 s into the raise). On the
board's firmware (params only, `--what ceiling`, `ceiling2`): G4 stays <= 13.9 but bursts 10/s (balance 12.2 +- 0.7);
smooth KQ 0.03 / K_ANG 0.4 balances 12.2 +- 0.1-0.4 but lift-off carries it to 15-19; slower NL_RATE and an
NL_MAX_CMD cap of 0.90/0.92 change nothing (the surplus comes in the first second after the nose breaks free).
Simulated lift-off command 0.93 with NL_A right vs 0.70-0.88 with x0.85; the aircraft lifted at 0.90-0.94 (weak
evidence for "right").

Firmware ceiling (`~/ad_board`: nearest-to-board source + hold fix + `NL_CEIL`): above NL_TGT + NL_CEIL the raise and
hold bring the nose back like the lowering (-NL_RATE on the lowering gains, integrator carried over) until it is
at NL_TGT; a cancel above the ceiling skips the lowering's 1 s ease-in. KQ 0.03/0.015, LOW 0.06/0.03, K_ANG 0.4,
RATE 3, NL_CEIL 1 (`ceiling4`, seeds 1-2): NL_A right: peak 13.6-13.8, mid-cancel 11.5-13.1, balance 12.25-12.29 +-
0.33-0.39, 0.5-0.7 flips/s, never above 14; NL_A x0.85: peak 14.4-16.2 for 0.2-0.7 s. Board image
`~/firmware_backups/FLASH_nearest_board_holdfix_ceiling.px4` (+912 bytes vs the board: NL_CEIL and its message,
no NL_Q_LPF), diff beside it. Flashed 2026-09-23 18:26 over USB (build Sep 23 18:15:30); parameter diff vs the
backup: only NL_CEIL new (plus _HASH_CHECK and the boot baro offset); then NL_CEIL 1, NL_KQ 0.03, NL_KQI 0.015,
NL_K_ANG 0.4 set (TGT 12, TOUT 60, lowering G4 kept). The repo's firmware/ now equals the flashed source.
~/PX4-nl's SITL binary is this build.

## Balancing on the nose fans only (`scenarios/fw_nose_lift_balance.json`)

Park 5.3, NL_TGT 12, 30 s in Holding with the throttle at zero (PX4 never takes over, motors 1-8 at 0), then SB
off. Headless (`--what balance`): committed model 12.0 +- 0.13 (G4) / +- 0.21 (KQ 0.03); aircraft-like model with
G4 never settles (5-14 deg, 9.8 flips/s, lift timeout), with KQ 0.03 / K_ANG 0.4 overshoots to 18.4 on the way up
and then holds 12.2 +- 0.30 at 0.07 flips/s. Live in the app (Tuning live `live209016643`, `live209124434`,
committed model, KQ 0.03): holding after 4.1-4.2 s, motors 1-8 at 0.00 throughout, M9 0.93 / M10 0.82 (the yaw
split), true pitch 10.54 +- 0.36 while the module holds its 12: PX4's pitch estimate reads ~1 deg high already
parked with the fans off (6.3 vs 5.4), so a standing estimate offset, not the balance. Vibration metric 3.8.

Earlier conclusion: the diagnosis stands; the aircraft-like model does not yet reproduce the aircraft's rise, so no gain
set from it can be trusted. Needed: the ULog of flight 2 (full-rate gyro and outputs: the rocking's real spectrum
and amplitude and how it follows the surges) and the fans' bench step response (open item), then refit.

Firmware (uncommitted as of this note): the four rate-loop copies became `NoseLift::rate_loop`, with NL_FB_BAND and
NL_SLEW (default 0 = off, behaviour unchanged: G4 on the rebuilt SITL flips 7.25-7.58/s as before). Built into
~/PX4-nl SITL, not flashed.
