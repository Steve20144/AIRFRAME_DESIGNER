# Fan vibration drifts the EKF height on the ground; check liftoff against the baro

Date: 2026-09-22. Aircraft on the bench, two front fans wired (outputs 9/10), park -8 deg, log 164 (pulled over the
SiK radio).

## What happened

Staged takeoff with nobody touching anything: M9 at 100 %, M10 at 82 % (the yaw-cancelling split, NL_W 1.10/0.90),
and after 6.4 s "Nose lift aborted (left the ground)". The pitch never moved from -8 deg.

## Why

Height is baro only (EKF2_HGT_REF 0, GPS off indoors) with EKF2_BARO_NOISE 3.5 m, so the estimate leans on the
accelerometer. The fans raised the accel vibration metric from 0.02 to 3.8 and biased it by a few hundredths of a
m/s^2: the estimate climbed a smooth 0.85 m at 0.3 m/s while the raw baro stayed within 0.4 m (0.30 m low-passed).
The old check (estimate rise > NL_LIFT_DZ and climb > 0.3 m/s) cannot tell this from a real hop. Raising NL_LIFT_DZ
only delays it: the drift accelerates.

## Fix

`lifted_off()` also needs the 0.5 s low-passed `vehicle_air_data.baro_alt_meter` to have risen NL_LIFT_DZ since the
lift began; without a baro in the last 0.5 s the estimate decides alone, as before. Replaying log 164: max 0.30 m
against 0.5 m, so it would not have tripped (0.2 m margin). SITL takeoff/kill/cancel at +4 and -8 pass.

## Still open

Soft-mount the Pixhawk and balance the fans (3.8 will also hurt hover). "Strong magnetic interference" right after the
run: fan currents near the compass. The nose did not rise at -8 deg park with props on: the two fans' margin runs out
between -1.7 and +1.7 deg park (see [aircraft runs](../experiments/2026-09-23-aircraft-nose-lift-bench-runs.md)).
