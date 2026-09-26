# Telemetry radio and the throttle dashboard

## The link

RFD SiK 2.0 (HM-TRP) pair, NETID 25, 57600 baud, 915-928 MHz, MAVLink framing. Ground side: FTDI 0403:6015, COM6 on
the Windows PC. Air side: the Pixhawk 6X's TELEM3 (`/dev/ttyS1`). TELEM3 had no MAVLink instance (MAV_0 = TELEM1,
MAV_2 = Ethernet), so it was silent although the radios were paired: set `MAV_1_CONFIG` 103, `MAV_1_MODE` 0 (Normal),
`MAV_1_RATE` 0; `SER_TEL3_BAUD` was already 57600. Useful throughput ~2 KB/s (a 1 MB log takes ~9 min; USB takes
seconds). `+++` / `ATI5` / `RTI5` / `ATI7` read both radios' settings and link counters (remote RSSI 0 = drone side
unpowered).

## scripts/throttle_dashboard.py

Windows Python (pymavlink, pyserial, pyulog installed with `--user`), not WSL: the radio is a Windows COM port.
`.claude/launch.json` entries `throttle-radio` (COM6, http://127.0.0.1:8095) and `throttle-usb` (COM3, 8096).

- Bars for every output, labelled from `PWM_MAIN/AUX_FUNC`; the nose-lift (front) motors (`NL_MOT_MSK`) large, every other motor (rear foil fans M1-M8) as tall bars below. Mode, armed,
  battery, RSSI, nose-lift state/abort/pitch/command (`DEBUG_VECT` "NLIFT": x = state + 0.01 * abort, y = pitch,
  z = command), last status texts, an Altitude mode button (`MAV_CMD_DO_SET_MODE`).
- Runs: arm to 3 s after disarm, with the 15 s before arming, saved to `results/telemetry_runs/run_<time>.json`
  (messages, nose-lift state changes, mode changes, link drops, peak outputs); listed under Runs on the page.
- Reconnects when the port disappears; forwards everything to udp 14550 so QGC can run alongside.
- Logs tab (added 2026-09-23): each flight's live log from arm to disarm (not the Runs' 15 s before / 3 s after),
  `results/telemetry_runs/log_<time>.csv`, appended and flushed row by row. 10 samples a second of mode, armed,
  link, msg rate, battery, RSSI, ATTITUDE (`px4_*`: PX4's frame, level = hover pitch), nose-lift state / abort /
  pitch / command and outputs 1-16 (µs, headers labelled from PWM_*_FUNC at arm time), plus one row per event
  (status text, mode, nose lift, link, armed). Each row: local time to the ms, epoch, seconds since arming, and the
  board's uptime from ATTITUDE.time_boot_ms (lines up with the ULog). Values older than 1.5 s are blank. The tab
  lists the flights, shows a table (every sample / one per second / events only), downloads the CSV
  (`/logs`, `/logs/<name>`, `/logs/<name>.csv`), and marks the tab while one records. Event rows are written
  after the state they announce is updated, so their columns show the new mode / nose-lift state.
- Forces MAVLink 2 (`MAVLINK20`): this pymavlink defaults to MAVLink 1, whose STATUSTEXT has no chunk id.
- Desk test without the radio: launch `throttle-fake` (the dashboard on `udpin:127.0.0.1:14660`, port 8097, runs
  in /tmp/throttle_fake_runs), then `python scripts/fake_px4_telemetry.py` flies a scripted flight into it.

Quirks it handles, each found the hard way:
- PX4 streams outputs 9-16 only as `SERVO_OUTPUT_RAW_1`, on no link by default, and forgets `mavlink stream` at every
  boot. The dashboard re-adds them through the board's shell (SERIAL_CONTROL) on a new heartbeat, on uptime going
  backwards, and whenever outputs 1-8 arrive but 9-16 do not (a quick reboot hides inside the heartbeat gap).
- PX4 sends no STATUSTEXT (and the SiK no RADIO_STATUS) on a link without a GCS heartbeat: the dashboard sends one at
  1 Hz. Safe only while `NAV_DLL_ACT` is 0; otherwise closing it would count as losing the ground station.
- Long status texts arrive in 50-character chunks (same `id`); joined before recording.

## Logs and QGC

- QGC holds its serial port exclusively; close it (or use its UDP link to the dashboard) before touching COM3/COM6.
  It saves tlogs to `Documents/QGroundControl Daily/Telemetry/`, which is how the first aircraft abort was diagnosed.
- ULog download over MAVLink: `LOG_REQUEST_LIST` for sizes (entries can be lost on the radio: ask for the id
  directly), then `LOG_REQUEST_DATA` with the exact size, re-requesting gaps. Logs are time-ordered, so a partial
  download already holds the start of a run.
