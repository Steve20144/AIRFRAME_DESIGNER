# Lesson: stands, pivots and lift margin

- The aircraft pivots about its rear feet during the nose lift and the landing. Gravity's moment about that pivot
  is weight times the horizontal distance from the rear feet to the CG at the parked pitch; the nose fans' moment
  is their max thrust times their arm from the same pivot. Ratios seen: ATLAS_OG at +8 comfortable; ATLAS_09B at
  -12: 1.05 (stalls), at +4: 1.19; the older as-built ATLAS could not pivot from -16 (58 vs 58.8 Nm).
- A nose-down park moves the front feet's ground contact aft relative to the CG; on ATLAS_OG's hard points any
  nose-down park puts the CG ahead of the front feet (tips). `legs_for_park_pitch` moves the front feet 8 cm ahead
  of the CG and keeps the rear arm; a stand that still cannot stand raises a clear error.
- At the hover attitude the aircraft rests on the rear feet only (line contact); the ground reaction produces a
  nose-down moment PX4 trims with its integrators while thrust ramps; the release from the ground is where kicks
  appear. A decisive liftoff (1 s ramp) shortens the time spent at unity thrust-to-weight.
- The tricycle stand (ATLAS_09B) lands on the rear legs first with the nose leg 0.7 m in the air: without the nose
  lower hook it falls onto its nose or, when touching down with aft speed, over its tail.
- The scripted pilot's touchdown-speed metric is the full velocity norm: a lateral drift at touchdown inflates it
  and, with the hook taking over, tips the aircraft. Fix the drift, not the descent rate.
