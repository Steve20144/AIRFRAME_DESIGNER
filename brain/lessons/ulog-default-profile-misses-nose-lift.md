# Lesson: the board's default ULog cannot identify the nose-lift dynamics

Checked 2026-09-23 in `~/PX4-nl/src/modules/logger/logged_topics.cpp` (v1.17). The default profile (SDLOG_PROFILE
bit 0) logs `sensor_combined` (gyro, accel) at full rate, `vehicle_angular_velocity` at 50 Hz, `vehicle_attitude`
at 20 Hz, but `actuator_outputs` and `actuator_motors` only every 100 ms (10 Hz). Our `nose_lift_output` topic is in
no profile, so the nose-lift command is not in the ULog at all (only outputs 9/10 at 10 Hz). The legs rock at 5 to
10 Hz, so every ULog flown so far records the response at full rate but its cause at 10 Hz: unusable for system
identification of the fans or the rocking.

Fixes: bit 3 (SYSTEM_IDENTIFICATION, SDLOG_PROFILE 9 with the default set) logs `actuator_motors`,
`vehicle_angular_velocity`, `vehicle_acceleration` and `vehicle_torque_setpoint` at full rate, but not
`actuator_outputs` or `nose_lift_output`. A `/fs/microsd/etc/logging/logger_topics.txt` (lines `topic interval_ms`)
adds any topic without a reflash, but it **replaces** the whole profile (`initialize_logged_topics`), so it must list
everything else wanted too. The ESCs are PWM with no telemetry, so no log holds fan speed: that must come from a
thrust stand, a tachometer, or the blade-pass pitch in an audio recording.

Installed 2026-09-23 19:39: `firmware/sdcard/etc/logging/logger_topics.txt` (134 topics: actuator_motors,
nose_lift_output/feedback, debug_vect, sensor_combined and vehicle_angular_velocity every sample; actuator_outputs
100 Hz; the default profile's multicopter topics at their default rates) is on the board's card; the boot log says
"logging 134 topics from logger_topics.txt". Uploaded with `scripts/board_sdcard.py put ... --reboot` (MAVLink FTP
over USB; pymavlink needs a fresh MAVFTP object per transfer and a Windows temp path, see the script). Write load: a
first version (outputs at 200 Hz, accel and setpoints every sample) wrote 180 KiB/s with the logger buffer at 90 %
and 2 dropouts of up to 0.16 s in one of two 10 s tests, fans off; the installed, trimmed version: 147-150 KiB/s (~9 MB a
minute), 0 dropouts in 90 s of tests, buffer peaks 89-93 % (fans off; `logger on` / `logger status` / `logger off`
over the NSH shell). Tight: if flight logs show dropouts, lower actuator_outputs or vehicle_acceleration first. The board kept dropping off USB whenever the port
was opened while the laptop battery was flat and the Pixhawk ran on USB power alone; with the flight battery
connected it worked, but it dropped again at 19:40.
