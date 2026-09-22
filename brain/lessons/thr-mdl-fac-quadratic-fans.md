# Lesson: quadratic fans need THR_MDL_FAC 1

The simulated EDFs (and the real ones, thrust proportional to command squared, `thrust_exponent 2`) must be
declared to PX4 with `THR_MDL_FAC 1.0`, and `MPC_THR_HOVER` set to the hover *thrust* fraction (about 0.36 to
0.40 for ATLAS). Without it PX4 mixes as if thrust were linear in command: the allocated torques are wrong by the
mix asymmetry, the rate integrators carry a constant bias that changes with collective, and every liftoff shows a
nose-up kick (5 deg on ATLAS_09B) as the collective ramps through unity thrust-to-weight. ATLAS_OG had it from the
start; the imported ATLAS_09B did not, and its landings crashed until it was added.
