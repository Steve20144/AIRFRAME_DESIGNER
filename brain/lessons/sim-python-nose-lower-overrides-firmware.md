# Lesson: the simulator's own nose-lowering overrides the firmware after touchdown

`design.nose_lower.enabled` (true on `atlas_v3_30_jets_57.5_85_85_park-10.json`) makes `Simulator._auto_nose_lower`
arm a Python `NoseLower` whenever PX4 is armed and the CG is above 1.0 m. After the touchdown it takes EVERY motor
(`override_all`: rear fans cut at once, nose fans on its own loop) and at the end force-disarms PX4. This happens
even with `design.nose_lift.executor: "firmware"`, i.e. on top of the PX4 nose_lift module.
Effect (found 8 Oct): every SITL flight that hovered above ~0.7 m above the ground (all Stabilized-climb flights,
peak 1.7 to 2 m) had its touchdown and nose lowering partly flown by the Python sequence, not the firmware; its
instant motor cut is why those touchdowns looked gentle (3 to 4 deg/s). Flights hovering at 1.0 m CG height did not
arm it and showed the firmware's real touchdown (~15 deg/s with PX4 Land).
Rule: when the firmware runs the nose (executor "firmware"), pass `--set design.nose_lower.enabled=false`
(scripts/sitl_pack.py does since 8 Oct; the live-app airframe `atlas_v3_30_board_winnerC_sitl.json` too).
Results before that in results/sitl_pack/{tune1,tune2,tune3,final,climb1,climb2,j1,j2} are affected for the
touchdown/lowering metrics of high-hover flights.
