# Control allocator -> AIRFRAME_DESIGNER nose lift: the allocator's own motor commands before the nose lift was
# applied, so the handover can tell when PX4 drives the lifting motors at least as hard as the hold.

uint64 timestamp		# time since system start (microseconds)
float32[12] allocator_control	# [-] actuator_motors commands as the allocator computed them (NaN = stopped)
