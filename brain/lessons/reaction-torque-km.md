# Lesson: the reaction-torque coefficient decides controllability

All ten ATLAS fans spin the same way, so their reaction torques add: yaw torque = km x total thrust, about
km x 120 N in hover. PX4 cancels it with the canted nose fans' lateral thrust, and the min-norm allocator also
leans on the foil fans' forward thrust components at their lateral arms, which drives one foil fan toward idle.

- km 0.01 (imported ATLAS_09B): the hover mix needs negative thrust on three fans; no gain flies it.
- km 0.004 (validated on the ATLAS_OG board sessions for that model): mix feasible only at one hover pitch with a
  fan at idle; PX4 has no nose-up authority, hovers 1.7 deg nose-down.
- km 0.002 (ATLAS_OG file value): 2.7 Nm pitch authority each way at hover 24 on ATLAS_09B; yaw still only +0.3 Nm.
- Earlier history: committed ATLAS files with km -0.01 crashed; hovers under 0.004, could not leave the ground
  above it.

km is an estimate in every model. HITL cannot check it (the board sees the simulator). Measure it on a thrust
stand before trusting Stabilized-mode results on hardware. Alternating spin directions would roughly double yaw
authority; the aircraft is built with all fans clockwise.
