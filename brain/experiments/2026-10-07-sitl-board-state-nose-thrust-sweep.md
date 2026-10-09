# 2026-10-07 20:15 SITL with the flashed firmware and the board's parameters: nose-fan thrust sweep

Setup: SITL built from the reverted sources (= the image flashed 19:55, pre-first-test code), airframe
`atlas_v3_30_jets_57.5_85_85_park-10.json` with `design.flow_sensor.enabled=true` (its EKF2 export then equals the
board's except sensor delays and declination), every board NL_* and CA_ROTOR0..8 parameter plus the flight-relevant
board values that differed from the SITL export (COM_DISARM_LAND -1, COM_RC_OVERRIDE 0, MPC_LAND_SPEED 0.3,
MPC_TKO_SPEED 2, NL_AUTO 1 / ALT 1.0 / HOV 5 / RAMP 0.5 / SLEW 1.5, NL_F_FF 1, NL_HO_THR 0.65, NL_HO_TOUT 15,
NL_LOW_KQ/KQI 0.06/0.03). Board parameters reconstructed in
`results/board_params/params_20261007_board_now_reconstructed.json` (15:20 dump + 5 changes read back 19:55).
Scenario `scenarios/fw_auto_hop.json`, sticks centred (no pilot pitch input). Only the physical max thrust of the
three nose fans was changed (`rotors[6:9].max_thrust`); PX4 and the module still believe 36 N, as on the board.
Results `results/fw_auto_hop/v3_boardstate_*`.

| nose fans | result | lift s | hold cmd | lowest nose after takeoff | nose fans saturated in flight | fwd speed |
|---|---|---|---|---|---|---|
| 36 N | pass | 7.4 | 0.52 | 7.6 | 0 % | 0.1 m/s |
| 32 N | pass | 9.2 | 0.59 | 6.0 | 0 % | 0.7 |
| 30 N | pass | 10.4 | 0.63 | 6.1 | 0 % | 1.3 |
| 28 N | pass | 12.0 | 0.67 | 3.5 | 1 % | 1.9 |
| 25 N | CRASH (lands sliding, 3.7 m/s) | 14.6 (7 s "stuck" at -9.5 while the cmd winds 0.7->0.97) | 0.75 | 4.0 | 56 % | 2.9 |
| 21 N | nose never lifts, cmd 1.0 for 20 s, lift timeout | - | - | - | - | - |
| 18 N | same | - | - | - | - | - |

Seeds 0 and 7 at 36 N: both pass (peak 1.7 to 1.8 m with ALT 1.0, touchdown 0.20 to 0.25 m/s).
Matching to the aircraft: hold cmd 0.62 on 6 Oct and 0.64 / 0.67 today = 28 to 30 N equivalent on fresh-ish packs;
the "stuck" starts and the 15:38 nose-down handover match 21 to 25 N (packs sagged after a few attempts).
Conclusion: firmware and parameters fly; the aircraft has a ~15 % nose-thrust margin (fail between 28 and 25 N
equivalent), and pack sag over a session eats it. The stuck start is lack of thrust at the parked angle (worst lever),
not friction. Hands-off forward drift grows as the nose sits lower (the jets' forward push).
Guard to use on the aircraft: hold command on the dashboard; above ~0.70 (< 28 N) do not take off.
- 2026-10-08: the thrust-to-aircraft mapping in this note is WRONG (util_max is thrust, nl_cmd is command); see 2026-10-08-sitl-firmware-selection.md.
