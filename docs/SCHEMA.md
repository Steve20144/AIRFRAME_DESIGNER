# Airframe JSON schema (version 2)

Everything lives in the **structural frame**: FRD (x forward, y right, z down), metres, origin at the airframe's
reference point (any convenient datum). The centre of gravity is a property of the `mass` segment, so moving the
CG never means re-entering positions. Angles are degrees in JSON.

```jsonc
{
  "schema": 2,
  "name": "ATLAS_08",
  "mass": {
    "mass": 13.0,                 // kg, total
    "cg": [0.0, 0.0, 0.0],        // centre of gravity, structural frame
    "inertia": [0.075, 0.128, 0.185],      // Ixx Iyy Izz about the CG, body axes
    "inertia_products": [0, 0, 0],         // Ixy Ixz Iyz
    "items": [ {"name": "battery", "mass": 2.0, "pos": [0.1, 0, 0.02], "inertia": [0, 0, 0]} ],
    "from_items": false           // true: mass, cg and inertia are computed from items
  },
  "body": {
    "size": [0.14, 0.12, 0.06],   // drawn box, m
    "drag_quadratic": [0.1, 0.1, 0.2],     // N per (m/s)^2 along each body axis
    "drag_angular": [0.005, 0.005, 0.005], // Nm per (rad/s)^2
    "drag_center": [0, 0, 0]      // where the body drag acts
  },
  "rotors": [ {
    "name": "M1", "enabled": true,
    "pos": [-0.3, -0.37, 0.14],   // structural frame
    "axis": [0.26, 0.0, -0.97],   // unit thrust direction (up = [0,0,-1]); the UI edits it as tilt/cant
    "km": 0.01,                   // CA_ROTORn_KM: reaction torque per unit thrust, sign = spin (>0 CCW from above)
    "max_thrust": 36,             // N at full command
    "tau": 0.12,                  // spool time constant, s
    "diameter": 0.08,             // fan/prop diameter, m
    "thrust_exponent": 2.0,       // thrust = max * omega^n
    "kind": "ducted",             // "prop" | "ducted"
    "ram_drag": true,             // momentum drag of the inlet flow (ducted fans)
    "duct_axis": [1, 0, 0],       // jetfoil: physical fan axis; the jet is bent to "axis" (null = fan along the jet)
    "turn_loss": 0.1              // thrust lost at 90 degrees of jet deflection
  } ],
  "wings": [ {
    "name": "main", "enabled": true,
    "pos": [0.42, 0.0, 0.06],     // ROOT LEADING EDGE on the centreline
    "span": 1.075,                // tip to tip (symmetric) or panel length (single panel)
    "root_chord": 0.93, "tip_chord": 0.0,
    "sweep_deg": 60,              // leading-edge sweep, back positive
    "dihedral_deg": 0,            // tips up positive; 90 with symmetric=false is a vertical fin
    "incidence_deg": 10,          // section incidence at the root (each section turned about the span line), LE up
    "pitch_deg": 0,               // whole-wing pitch: rigid rotation about the aircraft pitch axis through pos, nose-up positive
    "twist_deg": 0,               // extra incidence at the tip (washout negative)
    "symmetric": true,            // mirrored left/right halves
    "panels": 6,                  // strips per half for the strip-theory model
    "aspect_ratio": null,         // override the geometric AR
    "aero": {
      "model": "polhamus",        // "linear" (slope + induced drag) | "polhamus" (sharp-edged delta with vortex lift)
                                  // | "polar" (airfoil section data: CL/CD/CM vs alpha and Re from XFOIL or NeuralFoil)
      "airfoil_root": "naca23006", // polar model: NACA 4/5-digit, a file airfoils/<name>.dat, or a UIUC database name
      "airfoil_tip": "naca23006",  // polar model: tip section (blank = root); sections blend linearly along the span
      "polar_source": "auto",      // "auto" (XFOIL if installed, else NeuralFoil) | "xfoil" | "neuralfoil"
      "ncrit": 9.0,                // transition criterion of the polar (9 clean, 5 rough/turbulent)
      "cl_alpha": 6.2832,         // 2-D lift slope, 1/rad (Helmbold gives the 3-D slope from the AR)
      "cl0": 0.0, "cd0": 0.02, "oswald": 0.85,
      "stall_deg": 30, "stall_blend_deg": 15, "cd_flat": 1.2,
      "cm0": 0.0,                 // section pitching moment about the quarter chord
      "vortex_lift": true
    }
  } ],
  "legs": [ {
    "name": "FR", "enabled": true,
    "attach": [0.2, 0.2, 0.0],    // where the leg meets the structure
    "length": 0.28,
    "tilt_deg": 10,               // leg leans forward (+) / back (-)
    "cant_deg": 0,                // leg leans outward (+) / inward (-)
    "foot_radius": 0.0,           // contact starts this far above the foot point
    "stiffness": 3000, "damping": 150, "friction": 0.8
  } ],
  "hover_pitch_deg": 15,          // nose-up pitch PX4 treats as level (geometry exported in that frame, SENS_BOARD_Y_OFF set)
  "landed_pitch_deg": -10,        // attitude when standing on the legs (initial state of a simulation)
  "px4_overrides": {"MPC_THR_HOVER": 0.6},   // PX4 parameters set by hand, exported and seeded with the geometry
  "design": {"cruise_speed_kmh": 50},        // analysis / optimiser settings
  "notes": "",
  "mesh": {"file": "atlas_og.stl", "frame": "frd", "scale": 1.0, "opacity": 0.85}
                                  // optional CAD visual: an STL under airframes/meshes/, served at /meshes/<file>,
                                  // drawn by the 3D view in place of the body box. frame = the mesh coordinates:
                                  // "flu" (Gazebo model frame, x fwd y left z up) or "frd" (this schema's frame);
                                  // same origin as the structural frame, metres after scale. Visual only.
}
```

### What the jetfoil model is, and is not

A rotor with a `duct_axis` is a fan blowing along that axis whose jet a foil bends into `axis`. The simulator
applies **one force: the rotor thrust along `axis`, at `pos`**, so `pos` must be the jet exit on the foil (where
the turned jet leaves), not the fan. The turning reaction on the foil is not modelled separately because, for the
rigid vehicle, fan thrust plus foil reaction equals the exit momentum flux, which is that single vector; the only
explicit turning parameter is `turn_loss`, which scales `max_thrust` by `1 - turn_loss * deflection / 90 deg`.
The ram drag of a ducted rotor (the inlet mass flow `sqrt(rho A T)` times the local airspeed) also acts at `pos`.
Not modelled: any dependence of the deflection angle on airspeed or thrust, jet-induced lift on the foil, or
losses beyond the linear `turn_loss`. The foil as a lifting body in the outside flow is a separate `wings` entry
(strip theory); `airframes/atlas_og.json` carries one with an estimated planform measured on the CAD mesh.

### Fan vibration (`design.vibration`)

Off unless `enabled`. The fans' vibration is added to the simulated IMU samples (`sensors/vibration.py`); it never
enters the rigid-body integration (at hundreds of hertz it moves the vehicle by microns and matters only through
the sensors).

```jsonc
"design": {"vibration": {
  "enabled": true,
  "rpm_max": 30000,          // fan speed at full command, rpm (scalar or one per active rotor)
  "imbalance_gmm": 1.0,      // residual unbalance per fan, g mm: a force U w^2 rotating in the fan's disc plane (1P)
  "blades": 12,              // blade count (blade-pass frequency = blades x fan speed)
  "blade_ripple": 0.0,       // thrust ripple at blade pass, fraction of the fan's thrust, along the thrust axis
  "imu_pos": null,           // flight controller IMU in the structural frame, m (null: at the CG)
  "mount_hz": 0,             // soft-mount natural frequency, Hz (0: hard-mounted); "mount_damping": 0.1
  "frame_modes": [],         // [{"hz": 80, "damping": 0.03, "gain": 0.5}]: frame resonances, peak ~ gain / (2 damping)
  "accel_rectification": [0, 0, 0]  // DC accel bias per body axis, m/s^2 per g^2 of rms vibration
}}
```

Per tone: rigid body about the CG (F/m and I^-1 (r x F) on the IMU's lever arm; the gyro sees its integral), then
the frame modes and the mount, then the sampler: each HIL_SENSOR is the average over its interval, like PX4's
integrated IMU samples, so tones above half the sensor rate alias (a tone at a multiple of the rate averages to
zero). PX4's accel / gyro vibration metrics (its VehicleIMU formula) and clip counts (16 g, 2000 deg/s) are computed
on what is sent; they appear in the status bar, in the metrics (`accel_vibration_mean/max`,
`gyro_vibration_mean/max` per phase, `accel_clipping`, `gyro_clipping`) and in the time series (`vib_acc`,
`vib_gyro`). `airframe-designer vibration --airframe X --motors 9=1,10=0.82 --rate 400 [--target-metric 3.8]` holds
the fans at fixed speeds without PX4 and lists the tones, their aliases and the metric; `--target-metric` scales
`imbalance_gmm` to reproduce a logged metric. The metric depends on the sample rate: compare at the log's rate.

Schema 1 files (the AIRFRAME_SIMULATOR format) load transparently: `Airframe.from_dict()` migrates them (mass number
→ `mass`, `prop_diameter` → `diameter`, generated feet → four `legs`, the area/span delta wing → a `polhamus`
wing positioned by its root leading edge, CG = origin).

## Parameter paths

Any numeric leaf can be addressed by a string, used by `--set`, studies and the API:

| path | meaning |
|---|---|
| `rotors[0].pos[2]` | one element |
| `rotors[0,1].tilt_deg`, `rotors[0:8].cant_deg`, `rotors[*].tau` | several rotors (virtual `tilt_deg` / `cant_deg` write the axis) |
| `rotors[M3].km` | by name |
| `wings[0].incidence_deg`, `wings[0].aero.cd0`, `wings[fin].sweep_deg` | wings |
| `legs[*].length`, `legs[FL].tilt_deg` | legs |
| `mass.cg[0]`, `mass.mass`, `mass.inertia[1]` | mass segment |
| `hover_pitch_deg`, `landed_pitch_deg` | attitudes |
| `px4.MC_PITCHRATE_P` | a PX4 parameter (goes into `px4_overrides`) |
| `design.cruise_speed_kmh`, `design.vibration.mount_hz` | design settings (setting a path creates missing `design` levels) |

`airframe-designer paths --airframe X` prints every path with its current value.
