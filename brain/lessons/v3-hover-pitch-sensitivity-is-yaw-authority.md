# V3's hover-pitch sensitivity is the yaw authority limit (2026-09-26)

Symptom: ATLAS V3 small (best configuration, ATLAS_09B masses) in `v3_stab_nolift`, one CG, hover pitch changed by
0.5 deg: hover drift swings 0.5 <-> 6 m, pitch error 0.03 <-> 0.67 deg, periodic about 1.5 deg, identical on every
seed. It made the CG sweep and the fine ranking of the jetfoil sweep meaningless.

## The chain (each step measured, `results/v3_cg_trimcheck`)

1. **Reaction torque with all nine fans spinning the same way (km 0.002, assumed) keeps the yaw loop at its authority
   limit.** In Position mode the heading sits 4-6 deg off its setpoint at every hover pitch. Yaw can only come from
   left/right differences of the forward-leaning foil fans.
2. **That leaks into pitch.** PX4 holds a steady pitch error (0.67 deg at 17.75, 0.16 at 18.25) while its motors deliver
   almost no pitch torque; how much leaks depends on how the hover mix distributes the fans at that pitch, hence the
   periodic look. (The exact PX4 mechanism is not pinned down; motors are not at their limits, 0.48-0.75.)
3. **In hands-off Stabilized nothing corrects speed.** A pitch offset tilts thrust, the aircraft accelerates, and the
   inlets' ram drag (fans below the CG) pitches the nose further down; it settles at 0.5 m/s with the fans' +0.067 N m
   against -0.070 N m of ram-drag moment. The lift-off (the stand parks it 1.65 deg off the hover attitude, so lift-off
   always has a pitch correction) decides which state it lands in.

## Proof

- **km 0 on all fans: the sensitivity disappears**: pitch error 0.06-0.18 deg and drift 0.7-1.5 m, smooth over the
  same six hover pitches.
- Not it: fan vibration (off: same pattern), ram drag (off: same pattern, more drift), MC_PITCHRATE_I 0.2 (worse: winds
  up on the stand), the nose-lower sequence (it only waits; fans get exactly PX4's commands), a PX4-vs-physics model
  mismatch (the simulator's thrust torque equals PX4's allocation model).
- Position mode holds position at every pitch (2 cm std, 7 cm drift) but keeps the pitch-error pattern.

## What it means

- For V3 the reaction torque, i.e. km (never measured) and the spin directions, decides controllability more than the
  jetfoil angles or the CG. Same lesson as ATLAS_09B ([reaction torque](reaction-torque-km.md)).
- Geometry sweeps on V3 must be judged over a km bracket, or with counter-rotating fans, or both; a single-km,
  hands-off Stabilized score mostly measures where the yaw limit happens to bite.
- Minor: re-solve the stand with leg compression so it parks at the hover pitch (removes the lift-off kick).
