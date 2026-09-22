# Experiment: First Stabilized flights of ATLAS_OG, flat vs as-built brackets

Date: 2026-09-21

## Question

Can ATLAS_OG hover hands-off in Stabilized (no position source), and does the nose-bracket cant matter?

## Setup

SITL, `nose_lift_gust_stab` (Stabilized twin of the gust test: nose lift, arm, scripted throttle hand to 4 m,
hover, 6 m/s headwind and crosswind) and `stab_lab` (indoor flow with landing and nose lower). Board tune of
2026-09-18 (rate P 0.2, att P 4). Variants: as built (nose fans canted +-30) and `rotors[8:10].cant_deg = 0`.
Results under `results/stab/` (base_*, fast_*, og_gust_stab_*).

## Results

| | flat, Stabilized | as built, Stabilized |
|---|---|---|
| hover pitch / roll RMS | 0.04 / 0.09 deg | 2.1 / 9.8 deg |
| hover drift in 12 s | 0.63 m | 34 m |
| liftoff | clean | roll step to 8 deg the instant it leaves the ground |
| landing | 0.38 m/s, nose down in 9.6 s | crashed at 3.6 m/s |
| in 6 m/s wind | attitude within 1 deg, carried downwind at wind speed, weathercocks 60 to 90 deg in crosswind | roll to 21 deg, 233 deg yaw spin, motors saturated |

PX4's attitude setpoint stayed at zero throughout; on the as-built aircraft the actual roll sat at 8 to 13 deg with
only 75 percent motor utilisation: the weak-side authority (idle fan) was exhausted. A decisive 1 s liftoff avoided
the ground skid but not the trim.

Open-loop throttle does not work on this aircraft: mid stick gives thrust-to-weight 1.07 (as built) or 0.98
(flat); a fixed stick either climbed past 60 m under wing lift or never left the ground. The `alt_hold` throttle
hand was added to the `stick` phase for this.

## Conclusion

Flat brackets are flyable in Stabilized hands-off; the canted brackets are not, and it is authority, not tuning.
Position mode had hidden this.

## Follow-up

Gain sweeps on the flat configuration (next experiment); flat brackets are the indoor configuration (decision
2026-09-21-stabilized-indoor-flat-brackets).
