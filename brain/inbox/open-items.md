# Open items (unprocessed, 2026-09-21; aircraft items added 2026-09-23)

- **Measure the nose fans' step response** (spin-up, spin-down, dead time) on the bench: the sim does not reproduce
  the aircraft's ~1 Hz fan bursts; fit `tau` / `tau_down`, then re-tune `NL_KQ*` / `NL_LOW_KQ*`.
- **Lowering never fades**: the post-rotation pitch estimate reads ~0.5 deg high, the 0.5 deg fade tolerance is never
  met, thrust bleeds to the 25 s lowering timeout. Fade when the nose stops while the loop still asks it down.
- **App export reverts the board's nose-lift settings** (NL_KQ/KQI to 0.02/0.012, COM_DISARM_PRFLT to 120) and never
  exports NL_LOW_KQ/KQI: put G4 into `design.nose_lift` and export the lowering gains.
- **Compass**: "Strong magnetic interference" after every fan run; fan wiring near the GPS/compass.
- **Vibration**: accel vibration metric 3-5 with the fans on; soft-mount the Pixhawk, balance the fans.
- **SITL arming flake**: twice in ~40 runs PX4 armed while nose_lift stayed "disarmed"; retry passes.
- **Uncommitted** on branch nose-lift-firmware (as of 2026-09-23): both firmware fixes, NL_Q_LPF, sim tau_down and
  vibration, the throttle dashboard, the build-script params fix, these brain notes.

- **Remote switches in HITL for Stabilized.** Wanted: takeoff (rotate + lift) and land (land + rotate back) on
  switches. Exists: `design.nose_lift.rc_channel` (SB down = rotate, `rc_takeoff` chains PX4 Takeoff) and
  `design.nose_lower.enabled` (nose comes down on any touchdown). Missing: a Stabilized takeoff on a switch
  (rotation then a scripted lift while the pilot holds sticks) and a dedicated land switch for Stabilized.
- **Measure km** (fan reaction torque) on a thrust stand; it decides ATLAS_09B controllability.
- **Which geometry is the aircraft**: OG (Fusion-measured stations) vs 09B (nominal). Reconcile against CAD.
- **Hands-off drift acceptance**: ATLAS_09B sits at the 2 m / 12 s criterion (1.8 to 2.2 m with set A) because of a
  yaw-authority roll bias a pilot trims. Consider a position-hold stick option for the scripted pilot, or report the
  number for 09B instead of gating on it. Sweep scores are bimodal run to run; confirm over seeds.
- **Counter-rotating fans**: ten same-spin EDFs leave about 0.3 Nm of yaw authority one way. Alternating spin on the
  foil-fan pairs would cancel the reaction torque and make yaw symmetric. Hardware question for the fan supplier.
- **Sensor noise off** makes PX4 refuse to arm; the `noise: false` option is therefore unusable for diagnostics.
- **Gazebo backend**: one slow readback path unexplained; JSBSim is the cross-check engine in practice.
- **Older sweeps** (`stab_rate_p`, `stab_att_p`, `stab_rate_d_yaw`, `stab_asbuilt_*`) predate per-trial time
  series; their rows in the Tuning tab show metrics only.
- **Foil-fan rotor positions vs CAD**: the eight foil EDF bodies sit 0.1 to 0.3 m ahead of where OG and 09B place
  those rotors (jet exits). Measure the actual jet exit on the CAD and reconcile both models.
