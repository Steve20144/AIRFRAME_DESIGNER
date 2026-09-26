# ATLAS V3 small scale: the draft model (2026-09-26, branch atlas-v5-design)

`airframes/atlas_v3_small.json`, built by `scripts/atlas_v3_from_step.py` from `airframes/cad/SMALL_SCALE_V3.step`
(Fusion export, 65 solids, part names mostly timestamps, so parts are picked by index in the script).

## From the CAD

- Frame: CAD x right, y up, z aft; FRD = (-z, x, -y), origin on the centreline at boom level (CAD y 0.45).
- Nine fans. Six foil fans (3 a side, 106 mm ducts, 96 mm discs) pointing straight aft along a V of about 37 deg
  anhedral; each jet runs through a rectangular nozzle and along a curved Coanda lower wall that ends 72.7 / 74.6 /
  75.4 deg below horizontal (inner / middle / outer), no sideways lean. Three XFLY 80 in the nose: two canted 30 deg
  inward, one vertical.
- Two rear legs from a hub above the nose to feet aft and outboard (y +-0.69 m). No nose leg in the CAD.

## Assumed (constants at the top of the script)

Since 26 Sep the masses are ATLAS_09B's with nine fans (11.785 kg, see the sweep note); the 7.8 kg figures below and
the first results are from the earlier guess.

Masses (no materials in the file): fans 0.34 kg, tube 1.6 g/cm3, printed 1.0 g/cm3, battery 2.0 kg in the nose
carrier: total 7.8 kg, CG 5 cm ahead of the origin. Jet fully attached (thrust 14.6-17.3 deg forward of vertical).
Fan 36 N, km 0.002 same spin, ATLAS_09B PX4 gains, provisional nose leg parking at +4 deg, motor order M1/M2 outer
foil L/R, M3/M4 middle, M5/M6 inner, M7/M8 nose sides, M9 nose centre.

## First results (SITL, Python physics, ATLAS_09B gains)

- Static trim is feasible only at hover pitch 10.5-11 deg (worst fan 49 % at 10.5): very little pitch margin.
- `scenarios/v3_stab_nolift.json` (parked at 10.5, Stabilized, lift, hover 12 s, land): passes on seeds 1 and 2;
  pitch error 0.44-0.50 deg, roll 0.28, touchdown 0.53 m/s, fans at 47 %. But the heading turns 52 deg in the 12 s
  hover and the aircraft drifts 3.9 m: nine same-spin fans, the canted nose pair is the only yaw authority.
- Scenarios with a fixed ATLAS_09B attitude (stab_lab*, 23-24 deg) do not apply: at 23 deg the mix is infeasible and
  PX4 leaves the fans at idle. The plain `hover.json` (PX4 takeoff) tumbled on the first draft.
- MPC_THR_HOVER 0.26 from the first hover (09B's 0.32 stalled the scripted landing).

## Open

Real masses and battery position, which fans the foil ducts hold and their thrust, how far the jet really turns,
the nose support, ESC order. The yaw drift is the first design problem to sweep (front cant, counter-rotation).
