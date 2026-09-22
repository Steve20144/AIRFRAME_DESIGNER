# Lesson: the pseudo-inverse mix and the idle fan

PX4's multirotor allocator takes the minimum-norm solution of the effectiveness matrix (torques and thrust,
including the x and y force rows for tilted axes) and clips to [0, 1]. On ATLAS this matters twice:

- If the min-norm hover mix needs negative thrust on a fan, clipping leaves a residual torque PX4 must trim with
  integrators; if it pins a fan at idle, that fan bounds authority on three half-axes at once (roll one way, pitch
  one way, yaw one way). The aircraft then holds an attitude offset with the setpoint at zero and motors far from
  saturated. PX4's estimate is fine (`*_est_bias_deg` about 0.3 deg): it is the controller that cannot.
- Check feasibility offline before tuning: build the effectiveness from `rotors_in_px4_frame()` and
  `effective_max_thrust()`, take `pinv(B) @ [0,0,0,0,0,-W/ct_ref]`, look at the minimum share and at the torque
  each axis can add before a fan hits 0 or 1. The hover pitch that balances pitch authority is the one to fly
  (ATLAS_OG 26, ATLAS_09B 24); the reaction-torque coefficient sets how much authority is left at all.
- CA_METHOD 1 (sequential desaturation), airmode and integrator limits do not create authority; they redistribute
  saturation. They cured the board wobble on ATLAS_OG (yaw demand saturation) but did nothing for ATLAS_09B's bias.
