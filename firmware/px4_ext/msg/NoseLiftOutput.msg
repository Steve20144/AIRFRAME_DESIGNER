# AIRFRAME_DESIGNER nose lift -> control allocator: the motors the ground sequence owns while it runs.
# Published by the nose_lift module on every cycle while NL_EN is set; the allocator applies it just before it
# publishes actuator_motors, so PX4's kill switch and disarm (which act in the output driver) still cut everything.

uint64 timestamp		# time since system start (microseconds)

bool active			# false: the allocator's commands pass through untouched
uint16 override_mask		# motors (bit i = motor i+1) whose command is replaced by control[i] (NaN = stopped)
uint16 floor_mask		# motors whose command becomes max(allocator, control[i])
float32[12] control		# [-] motor commands in actuator_motors units

uint8 state			# NoseLift::State
uint8 abort_reason		# NoseLift::Abort
float32 pitch_deg		# [deg] structural pitch (PX4's pitch plus NL_HOV_PITCH)
float32 target_deg		# [deg] structural pitch the sequence is driving to
float32 cmd			# [-] lifting-motor command before the roll/yaw split

# Automatic takeoff, hover and landing (NL_AUTO): the throttle the sequence asks for, applied by rc_update in place of
# the pilot's throttle stick while it is finite and fresh (NaN = the stick flies).
float32 throttle		# [-] 0..1
uint8 auto_phase		# NoseLift::AutoPhase

# The aircraft stands on its legs under the ground sequence (raise, hold, handover until it is off its legs, lowering):
# mc_att_control then holds no roll error against the legs (it would load them and release as a roll jolt at lift-off).
bool on_legs
uint8 legs_hold		# NL_LEGS_ATT: which errors are not held against the legs (1 roll, 2 pitch)
float32 legs_fade_s	# NL_LEGS_FADE: the setpoint offset fades out over this long once off the legs
