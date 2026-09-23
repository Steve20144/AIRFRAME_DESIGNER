# A PX4 module must read the clock after copying its messages

Date: 2026-09-22. Found on the aircraft (Pixhawk 6X, only the two front fans wired, outputs 9 and 10).

## What happened

The first staged takeoff on the real aircraft ran 0.4 s: "Nose lift: raising the nose from -8.3 to 24.0 deg", the
fans ramped (cmd 0.18 to 0.50), then "Nose lift aborted (attitude lost): motors stopped" and a force disarm. The QGC
tlog (`Documents/QGroundControl Daily/Telemetry/2026-09-22 21-34-36.tlog`) shows ATTITUDE and HIGHRES_IMU steady at
100 and 50 Hz with no gap, so nothing was actually lost.

## Why

`NoseLift::Run()` read `now = hrt_absolute_time()` and then copied `vehicle_attitude`, `vehicle_angular_velocity`,
`input_rc` and the allocator feedback. On hardware the gyro and estimator threads outrank `nav_and_controllers` and
preempt it; a message published between the clock read and its copy has `timestamp > now`, and the unsigned
`now - timestamp < 100_ms` wraps to a huge age. Lockstep SITL can never do this (time only moves between steps), so
every SITL and sweep run passed. The same race could fire "radio lost" (`_rc_ok`) or miss the handover feedback.

## Fix

The clock is read after the copies (NoseLift.cpp, Run()). SITL takeoff/kill/cancel at +4 and -8 deg still pass.

## Rule

In any PX4 module: copy subscriptions first, then read the clock; or compare as `ts + window > now`. Never trust a
SITL pass for timing checks. See [nose lift on the flight controller](../decisions/2026-09-22-nose-lift-on-the-flight-controller.md).
