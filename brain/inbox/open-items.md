# Open items (unprocessed, 2026-09-21)

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
