# Decision: Fly indoors in Stabilized mode, on the flat nose brackets

Date: 2026-09-21
Status: accepted

## Context

Indoor tests have no GPS. A quadcopter flies indoors in Stabilized (attitude only, the pilot is the position loop).
ATLAS had only ever been validated in Position mode on a simulated GPS; an earlier note said Stabilized was not
flyable because yaw ran away. Position sources (H-Flow optical flow, external vision, a NMEA indoor GPS) cost money
and time; a GPS repeater gives every receiver the roof antenna's position and is useless for position hold.

## Decision

The indoor test flies in Stabilized with a pilot, no position source, on the ATLAS_OG airframe with **flat** nose
brackets (`airframes/atlas_og_flat.json`, `rotors[8:10].cant_deg = 0`). The as-built canted brackets are not flown
in Stabilized.

## Reasoning

Measured in SITL (experiments/2026-09-21-stabilized-baseline-flat-vs-asbuilt.md): the flat aircraft hovers
hands-off at 0.04 deg pitch RMS, lifts off and lands within 1 to 2 deg, touches down at 0.4 m/s. The as-built
aircraft leaves the ground with a roll step and PX4 then holds 6 to 10 deg of roll against a zero setpoint at 75
percent motor use: the weak-side allocation authority (idle fan) is exhausted. Sixteen combinations of integrator
limit, rate I, airmode, yaw weight, yaw P and yaw rate limit changed nothing; airmode 2 crashed. Position mode
had masked all of this.

## Alternatives Considered

- Buy the H-Flow first: valid later, but the aircraft must be controllable in attitude regardless.
- Tune the as-built brackets: no gain restores authority the geometry does not have.

## Consequences

The bracket is a hardware change before the indoor test. Tuning, HITL parameter files and the acceptance numbers
all refer to the flat configuration. The H-Flow remains the recommended first position source when hands-off hover
is wanted (research/indoor-positioning-options.md).
