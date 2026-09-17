"""AIRFRAME_DESIGNER: a PX4-in-the-loop aircraft design simulator.

Segments (each a sub-package, usable on its own):
  geometry   the airframe description: mass/CG, rotors (props, ducted fans, jetfoils), wings, legs, body
  aero       force models: strip-theory wings, rotor thrust/torque/ram drag, body drag
  dynamics   6-DOF rigid body about the CG, per-leg ground contact
  sensors    IMU / mag / baro / GPS models -> PX4 HIL messages
  px4        MAVLink link, parameter export, SITL process control, HITL connection management
  sim        the lockstep/real-time loop, scripted scenarios, flight metrics
  analysis   static hover/cruise analysis and the geometric (no-PX4) optimiser
  batch      headless runs, parallel workers, simulation-driven optimisation studies
  server     FastAPI + websocket for the 3D editor UI
"""
__version__ = "0.1.0"
