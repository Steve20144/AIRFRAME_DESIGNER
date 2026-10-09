# 2026-10-07 Atlas v1: first Aristotelis CFD library (b02r, 60 km/h, 35 attitudes) and free flights with wind

Batch `b02r-sidewind-60kmh-rome` (rome partition, 64 cores per attitude, 8 at a time; 4.5 to 7 min per attitude; the
first try in the home directory died of the 20 GB quota, moved to /scratch/s/<user>). Imported as
`results/cfd/atlas_v1/lib_60kmh_standard-b02r-sidewind-60kmh-rome` (1.29 M cells, 800 it, scatter 1 to 3 %, 0 failed).
Clean foil, no fans, alpha -6..12 x beta 0,5,10,15,20.

## Verdicts (Library.stability)
| Set | kg | CG x | Cm_alpha/deg | SM | NP x | hands-off trim | L/W at 60 km/h | speed for L = W | Cn_beta/deg | Cl_beta/deg |
|---|---|---|---|---|---|---|---|---|---|---|
| M001 | 46 | 1.369 | +0.0009 | -22 % | 1.76 | alpha -2.2 | 0.45 | 89 km/h | +0.00021 | +0.00107 |
| M002b | 92 | 1.423 | +0.0005 | -12 % | 1.64 | alpha +4.8 | 0.19 | 136 km/h | +0.00031 | +0.00083 |
- CL only 0.14 to 0.26 over alpha -6..12 and NOT monotonic (0.19 at -3, 0.14 at +6): the clean body is a poor,
  separated lifter at 60 km/h; the strip model (CL 0.4 to 0.8) is far more optimistic than RANS.
- Pitch statically unstable (NP 0.2 to 0.4 m BEHIND the CG... i.e. ahead in x: NP x 1.76 > CG x 1.37).
- Lateral: Cn_beta small positive (weathercock, the drooped tips), Cl_beta small positive (anhedral, no dihedral
  effect). Cl and Cn vs beta are non-monotonic (negative at beta 5, positive at 15 to 20): separated flow, treat the
  slopes as order-of-magnitude. CFD Cl_beta is 5x smaller than the strip model's +0.005.
- CG sweep: Cn_beta turns positive for CG x >= 1.4 m; Cm_alpha turns negative (stable) for CG x >= 1.6 m.

## Free flights on the CFD table (CFD tab, 90 km/h, release pitch 4 deg, M001)
| Scenario | Result |
|---|---|
| calm, hands off, 40 s | tumbles at 11.8 s (roll 122 deg, pitch -40..40) |
| crosswind 5 m/s, ideal pilot (300/400/150 N m) 60 s | held: roll <= 6 deg, sideslip 2 to 11 deg, drifted 192 m downwind (no crab), 16 % thrust |
| crosswind 10 m/s + turbulence 1.5 m/s RMS, ideal pilot | tumbles at 17.4 s (sideslip to 35 deg, roll 169) |
| gust programme 0 -> 8 m/s side at 15 s, 0 at 35 s, -8 + 2 up at 45 s | tumbles at 33.6 s during the 8 m/s phase |
The ideal pilot's 300 N m of roll and 150 N m of yaw are below what elevons + tip drag rudders give in the strip
model (670 to 1670 N m roll). 23 to 26 % of the steps of the failed flights were outside the computed alpha/beta
range (flat-plate extension), so the tumbles themselves are only indicative; the 5 m/s hold is inside the range.

## Next
b03r (90 km/h) and b04 (fine) bundles are ready in results/cfd/atlas_v1/hpc/ (upload to /scratch). A wider alpha
range (to 18) would cover the tumbles. Raise the pilot's moment limits in the CFD free flight to the elevon/rudder
figures to judge the controlled case on CFD data.

## Where the side force comes from (surface pressure split by region, alpha 3 and 9, wind from the right)
Regions (FRD): nose x > 1.9; centre body; inner wing 0.75 < |y| <= 1.3; tips |y| > 1.3. Lateral projected area one
side / mean arm from the CG: nose 0.50 m2 / +1.01 m, centre 1.97 / -0.67, inner wing 0.98 / -0.74, tips 0.71 / -0.71.
- Nose: always pushed downwind, ahead of the CG: -6 to -27 N m of yaw (nose turns AWAY from the wind), growing
  linearly with beta. The one steady, destabilising term.
- Tips (the Northrop effect): downwind force behind the CG, +2 to +10 N m into the wind at beta 10 to 20. Right
  sign, too small: a third of the nose's moment.
- Centre body + inner wing (the open under-body cup with the fan ducts): at beta 5 to 10 a force INTO the wind of
  +25 to +40 N (the crossflow is caught by the far wall of the cup), with a stabilising roll (Mx -27 to -63 N m);
  at beta 15 to 20 it flips to downwind with a destabilising roll (Mx +34 to +50). This cavity term dominates the
  lateral numbers and makes every derivative non-monotonic; with the fans running the cavity flow is different.
  => the clean-foil library cannot settle the lateral question; a powered CFD (duct inlets and jets) is needed.

## Powered CFD bundles (fans as actuator discs), 2026-10-07 afternoon
`case.write_fan_sources(case, fans)`: per fan a cylinderToCell cellSet (system/topoSetDict) and a vectorSemiImplicitSource
(constant/fvOptions, volumeMode absolute, `sources { U ((F/rho) 0); }`, force on the air along the jet). `hpc.export_bundle(..., fans=)`
writes them into every case and cases.sh runs `topoSet` before decomposePar. Builder `scripts/atlas_v1_powered_bundle.py`
(fans from the airframe rotors; the clicked fan points lie ON the lower skin, so the disc centre is 8 cm below, in
the air; wing jet 47 deg below aft = 63 % of the 75 deg foil per the V3 CFD; nose trio straight down). Smoke test on
the local quick mesh (topoSet, decomposePar, 10 iterations, 8 cores): sets of 6 cells each on the coarse mesh, solver
loads the six sources, runs. Bundles: `b05-powered-60kmh` (6 x 60 N, nose off: +245 N forward, 263 N up) and
`b06-powered-60kmh-nose40` (+ 3 x 40 N nose), 20 attitudes each (alpha -3..9, beta 0..15), rome / 64 cores.
Caveat: the wall `forces` exclude the fan thrust; add it in any trim check. Disc resolution on the standard mesh is
untested (expect a few tens of cells per disc).
