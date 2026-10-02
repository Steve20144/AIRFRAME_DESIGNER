# Real-flight parameter set: ATLAS V3_30 with the fitted foils 50 / 65 / 50 (2026-10-01)

**Status: NOT cleared for flight.** Generated for review only; nothing written to the board.

File: `atlas_v3_30_foils_50_65_50_real_flight_20261001.params` (175 parameters, QGC format), from
`airframes/atlas_v3_30_foils_50_65_50.json` by `scripts/real_flight_params.py`; 68 values differ from the board
(read-only dump `results/board_params/params_20261001_211932_readonly_decoded.json`), listed in the `.diff.json`.
The arm / disarm / safety settings are the same as in the CFD-foils set (`atlas_v3_30_cfd_real_flight_20261001.md`):
SYS_HITL 0, RC arm ch 8 and kill ch 5 working (COM_RC_IN_MODE 0), disarm 10 s if it does not take off and 2 s after
landing, nose lift on (NL_EN 1, ch 7), safety switch still disabled (CBRK_IO_SAFETY 22027), real H-FLOW on.

## Why it is not cleared

Airframe: V3_30 CAD masses (14.40 kg) with foils 50 / 65 / 50. The foil thrust uses the jet model fitted to the
OpenFOAM point (the jet turns ~63 % as far as the foil, ~25-33 % of the thrust lost): jets 31.7 / 41.2 / 31.7 deg
above the fan axis. Then:

- **Hover pitch 50.5 deg** (the three loss models give 50.1-50.8). Busiest fan 90-98 % of full thrust in hover.
- **Tip-back angle 47.1 deg**: the rear feet are 0.318 m behind and 0.295 m below the CG, so past 47 deg nose-up the
  CG is behind the feet and it falls over backwards.
- Headless SITL, 9 take-offs, all failed: rotating on the legs to 50.5 deg overshoots to 62.6 deg (falls back);
  lifting the nose only to 40 or 35 deg and taking off from there flips it too, because the moment it arms PX4
  pitches toward its 50.5 deg hover attitude while still on the legs.

## The uncertainty, and how to settle it

The hover angle rests on one CFD point taken on *other* foils (75 / 65 / 50). The old model, with the jet following
the foil, gave 23.85 deg for 50 / 65 / 50 (on the lighter v34, 11.8 kg), and that version flew in SITL and HITL.
The real aircraft sits somewhere between, and everything (SENS_BOARD_Y_OFF, the nose-lift target, whether it can
take off) depends on where. Ways to pin it down, cheapest first:

1. **Measure the thrust vector:** aircraft on a load cell / thrust stand (or tethered), foil fans only, read the
   forward and vertical force at a known throttle. The angle and magnitude replace the model directly.
2. **CFD of the fitted 50 / 65 / 50 foils** (same setup as the 75 / 65 / 50 run).
3. Design fixes if it really is ~50 deg: move the rear feet aft (tip-back angle above hover + ~10 deg), or the jet
   targets in `docs/V3_30_Foil_Angles_20261001.pdf` (60 deg jets: hover 19.5 deg).

Re-run `scripts/real_flight_params.py` once the measured thrust is in the airframe; the file then carries the right
hover angle. The H-FLOW mount should then be re-angled to the measured hover pitch (at 50.5 deg a 20.3 deg mount
looks 30 deg off vertical).
