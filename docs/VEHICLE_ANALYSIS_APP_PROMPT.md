# Build "Wrench": a validated multi-EDF vehicle analysis tool

You are building a new web application from scratch, in an empty repository. Read this whole document before
writing any code. Everything you need is here: the domain, the conventions, the exact formulas, the input data,
the acceptance tests with numerical oracles, and the build order. Do not invent physics that is not specified
here; where this document says a value is estimated or unknown, the app must say so too.

## 0. Mission and the failure you are preventing

The user is redesigning a 12 kg hovering aircraft lifted by ten electric ducted fans (EDFs). A previous tool for
this aircraft was a convincing 3D model whose force and control assumptions were partly wrong or incomplete, and
the errors were invisible because the model looked right and printed "hover: pass". Your app must be the
opposite: **an analysis tool first, a viewer second.** Every number it shows must be traceable to a typed input
with a unit and a provenance tag, every physics step must be a pure, tested function, and the verdict on a
configuration must come from an automatic gate that reports residuals and margins, never a bare pass.

Non-goals for this MVP (do not build them, do not leave stubs for them): foil or airfoil optimisation, PX4
parameter tuning, ground contact and landing gear, polished CAD rendering, mesh import, flight simulation in time.

## 1. The aircraft (domain context)

Name: ATLAS. Ten identical XFLY 80 mm EDF units, each rated 33.3 N at full command (this thrust curve is
**estimated**, never measured on this vehicle). Eight sit inside two "jetfoils" (one per side): each foil duct
points fore-aft and the foil bends the jet downward and aft, so each foil fan's thrust axis is tilted forward of
vertical by 25, 30, 35 or 40 degrees (outer to inner station) and loses 10 percent of its thrust per 90 degrees of
bend. Two nose fans sit on a bracket at the front; as built they are canted 30 degrees left and right.

The total mass comes from weighing (measured); the CG and inertia come from CAD rescaled to the weighed masses
(estimated); rotor positions come from CAD (measured); fan thrust, reaction torque coefficient, jet deflection
angles and turning loss are **estimated**. Power: the ESCs are Flycolor X-Cross HV3 160A units, one per fan
(datasheet limits, measured). The aircraft flies on **6S lithium packs** (owner statement; cell count and nominal
voltage estimated until the pack label is read). Pack capacity, pack discharge rating and the power distribution
board rating are **unknown** until the user enters them.

The aircraft hovers with its body pitched 26 degrees nose-up. That is not a bug: with the fan geometry above, the
summed thrust at the balanced hover allocation points 26.0 degrees forward of the body vertical (the simpler
max-thrust-weighted geometry reference of section 5.1 gives 26.57), so the flight controller's "level" is that
attitude. The app must reproduce this.

Known results from a prior audit (you will use these as test oracles in section 9):

* At the hover allocation, the ten fan thrusts are strongly asymmetric left to right (fan 1 at 2.7 N, fan 7 at
  20.9 N). The cause is the canted, fore-aft staggered nose fans, whose lateral thrust components produce a net
  yaw moment that only the foil fans can cancel. It is not caused by the fans all spinning the same way.
* That idle fan is the binding limit of control authority on three of six half-axes (roll one way, pitch nose-up,
  yaw one way: 0.39 N m).
* A 6 m/s headwind applies about 2.4 N m nose-down and a 6 m/s crosswind about 1.75 N m of yaw, both beyond the
  weak side of the as-built authority.

### The three architectures to compare (section 8)

* **A, as built:** nose fans canted plus and minus 30 degrees; all ten fans spin the same direction (km = -0.002).
* **B, flat brackets:** nose fans pointing straight down in the body frame, axis (0, 0, -1); spin unchanged.
* **C, flat brackets and counter-rotating left foil:** as B, with the four left foil fans (rotors 1, 3, 5, 7)
  reversed, km = +0.002.

The user must be able to add more variants; these three ship as fixtures.

## 2. Conventions (fixed, display them in the UI)

* **Structural (body) frame:** FRD. x forward, y right, z down. Origin is the CAD reference point, not the CG.
  Positions are given in this frame in metres. The CG is an input in this frame.
* **Thrust axis** `a` of a rotor is the unit direction of the force the rotor puts on the vehicle. A rotor that
  lifts the vehicle has axis (0, 0, -1). Normalise every axis on load; reject zero vectors.
* **Reaction torque** on the body from rotor i is `-km_i * T_i * a_i` (N m). `km` is in metres. `km > 0` means the
  rotor turns counter-clockwise seen from above. This is the PX4 `CA_ROTORn_KM` convention.
* **Hover frame:** the structural frame rotated nose-up by the hover pitch `phi` so that the mean thrust is
  vertical. A structural vector `v` becomes `R_h v` with
  `R_h = [[cos phi, 0, sin phi], [0, 1, 0], [-sin phi, 0, cos phi]]`.
  Torque and force balances, authority and gate results are computed and displayed in the hover frame. Positions
  used there are relative to the CG: `r_i = R_h (p_i - cg)`.
* **Moments** are right-handed about the frame axes: roll about x (positive = right side down), pitch about y
  (positive = nose up), yaw about z (positive = nose right). Show these words next to the axes in the UI.
* **Gravity:** `g = 9.80665 m/s^2`, weight `W = m g`, applied once, at the CG, along +z of the level hover frame.
* **Air density** `rho = 1.225 kg/m^3`.
* **Units:** every stored quantity is SI (m, kg, N, N m, m/s, kg m^2, V, A, W, A h). The schema carries the unit
  string with every field, and the UI prints the unit next to every number. Conversions for display live in one
  `units` module and nowhere else.
* **Rotor numbering:** rotor index i (0-based in code) is "Motor i+1" in the UI and in exports.

## 3. Technology and repository layout

pnpm workspace, TypeScript strict mode everywhere, ESLint, Prettier, Vitest.

```
wrench/
  package.json               pnpm workspace root; scripts: build, test, lint, typecheck (all must pass)
  packages/schema/           Zod schemas + inferred types + JSON Schema export + fixtures  (no React, no physics)
  packages/core/             the physics and gate: pure functions only, no DOM, no React, no I/O
  packages/report/           JSON and CSV serialisation of a run, provenance hashing
  apps/web/                  Vite + React 18 + TypeScript; @react-three/fiber for the 3D view; Zustand for state
```

Rules:

* `packages/core` imports only `packages/schema` and a small linear-algebra helper. Use `ml-matrix` for SVD-based
  pseudo-inverse (`pseudoInverse` with a tolerance of 1e-9 on singular values) or implement SVD yourself; do not
  use normal equations, they fail on the rank-deficient cases the gate must detect.
* No `Math.random`, no `Date.now()` inside `core`. Timestamps are injected by the caller of `report`.
* Iterate arrays in index order; never depend on object key order for numerics.
* Every exported function in `core` has a docstring stating its frame, units and formula, and at least one test.
* The UI never computes physics. It calls `core` and renders the returned objects. If you find yourself writing
  a cross product in a React component, stop and move it.
* `MODEL_VERSION` (semver string) lives in `core`, `SCHEMA_VERSION` in `schema`. Bump `MODEL_VERSION` whenever a
  formula changes. Both are written into every report.

## 4. Schemas (`packages/schema`)

Use Zod. Every numeric physical input is a **tracked quantity**: an object
`{ value: number, unit: <literal unit string>, provenance: Provenance, source: string }` produced by a helper
`tracked("m")`, `tracked("N")`, etc., so the unit and the provenance are part of the type and are serialised.
Vectors are `{ value: [number, number, number], unit, provenance, source }`. **Provenance is per quantity, never
grouped:** mass, CG, each inertia vector, each rotor's position, axis, thrust, km, diameter, and so on each carry
their own tag, because a CG from CAD and a mass from a scale have different provenance and the report must be
able to say so. `source` is free text ("weighing 2026-09-14", "CAD PHASE_0_V4", "Flycolor datasheet",
"assumed"); it may be empty only when provenance is `unknown`. Provenance `derived` is reserved for a value the
app itself computed from other tracked inputs and the user accepted with one click (only `hoverPitchDeg` can
receive it in this MVP, see section 5.1); a value the user typed is `estimated` or `measured`. Provide
`.default()`s only for non-physical fields such as names.

**Two declared exceptions to tracking.** (a) `GateSettings` are analysis thresholds and test cases authored by
the user, not properties of the vehicle; they are plain numbers with the unit fixed in the field name and shown
in the UI, and they are saved verbatim with every run and included in `inputHash`, so a run is still fully
reproducible. (b) Counts (`esc.count`, `panels`) are plain integers.

```ts
Provenance = z.enum(["measured", "estimated", "unknown", "derived"])
Tracked<U>  = { value: number, unit: U, provenance: Provenance, source: string }
Tracked3<U> = { value: [number, number, number], unit: U, provenance: Provenance, source: string }

Rotor = {
  name: string,
  position: Tracked3<"m">,            // force application point in the structural frame (for a foil fan: the jet exit)
  axis: Tracked3<"unitless">,         // thrust direction, normalised on parse
  ductAxis: Tracked3<"unitless"> | null,   // physical fan axis if a foil bends the jet; null = same as axis
  turnLoss: Tracked<"unitless">,      // fraction of thrust lost at 90 deg of jet deflection
  maxThrust: Tracked<"N">,            // at full command, before turn loss
  thrustExponent: Tracked<"unitless">,// T = maxThrust_eff * cmd^exponent, cmd in [0,1]
  km: Tracked<"m">,                   // reaction torque per newton, signed (spin direction)
  diameter: Tracked<"m">,             // disc diameter, for momentum theory power and ram drag
  spin: z.enum(["cw", "ccw"]),        // must agree with the sign of km; validate with a refinement
  currentModel: CurrentModel,         // thrust -> electrical current
  enabled: boolean
}

CurrentModel = discriminated union on "kind" (two kinds only; a polynomial model is NOT in the MVP because its
coefficients would need units of A/N^k per term):
  { kind: "momentumTheory", efficiency: Tracked<"unitless"> }                     // I = P_ideal / (eta * V_pack)
  { kind: "table", points: { thrust: Tracked<"N">, current: Tracked<"A"> }[] }   // >= 2 points, strictly increasing
                                                                                  // thrust; linear interpolation, clamp at ends

MassProperties = {
  mass: Tracked<"kg">,
  cg: Tracked3<"m">,                                   // structural frame
  inertia: Tracked3<"kg m^2">,                         // Ixx, Iyy, Izz about the CG, structural axes
  inertiaProducts: Tracked3<"kg m^2">                  // Pxy, Pxz, Pyz = integral of xy dm etc.
                                                       // tensor off-diagonals are MINUS these: I_xy = -Pxy
}

Body = {  // bluff-body drag, quadratic per axis
  dragQuadratic: Tracked3<"N s^2/m^2">,    // F_i = -c_i * v_i * |v_i| in the structural frame
  dragCenter: Tracked3<"m">
}

Foil = {  // a lifting surface in the outside flow; MVP stores it but only reports it
  name: string,
  planform: {
    span: Tracked<"m">, rootChord: Tracked<"m">, tipChord: Tracked<"m">,
    sweepDeg: Tracked<"deg">, dihedralDeg: Tracked<"deg">, incidenceDeg: Tracked<"deg">
  },
  liftSlopePerRad: Tracked<"1/rad">, cd0: Tracked<"unitless">, stallDeg: Tracked<"deg">,
  forceProvenance: Provenance          // provenance of the foil FORCE as a whole; if not "measured", the gate
                                       // lists the foil as "not modelled as real"
}

Power = {
  pack: { cells: Tracked<"unitless">, nominalVoltage: Tracked<"V">, capacity: Tracked<"A h">, maxContinuousCurrent: Tracked<"A"> },
  esc: { model: string, maxContinuousCurrentPerChannel: Tracked<"A">, burstCurrentPerChannel: Tracked<"A">,
         minCells: Tracked<"unitless">, maxCells: Tracked<"unitless">, massEach: Tracked<"kg">, count: number },
         // one ESC per rotor; refine: count == number of rotors, and pack.cells lies in [minCells, maxCells]
  pdb: { maxContinuousCurrent: Tracked<"A"> }
}

Vehicle = {
  schemaVersion: literal(SCHEMA_VERSION),
  name: string,
  description: string,
  mass: MassProperties,
  body: Body,
  rotors: Rotor[] (min 1, max 32),
  foils: Foil[],
  power: Power,
  hoverPitchDeg: Tracked<"deg">,       // the hover frame rotation phi; drives the physics (section 5.1)
  notes: string
}

GateSettings = {  // user-authored analysis settings (tracking exception (a)); units are in the field names;
                  // all defaults below; saved verbatim with every run
  hoverMaxUtilisation: 0.85,                              // unitless
  authorityFloorNm: { roll: 2.0, pitch: 2.0, yaw: 1.0 },  // N m
  authorityMarginFactor: 1.5,          // unitless; authority on each half-axis must exceed factor * worst disturbance on that axis
  cgOffsetsM: [ [+0.02,0,0], [-0.02,0,0], [0,+0.02,0], [0,-0.02,0], [0,0,+0.01], [0,0,-0.01] ],  // m, structural frame
  cgOffsetMaxUtilisation: 0.90,                           // unitless
  failureMaxUtilisation: 1.0,                             // unitless
  windCases: [ {name:"headwind 6", windNedMps:[-6,0,0]}, {name:"tailwind 6", windNedMps:[6,0,0]},
               {name:"from right 6", windNedMps:[0,-6,0]}, {name:"from left 6", windNedMps:[0,6,0]} ],  // m/s, air-mass velocity
  windMaxUtilisation: 1.0,                                // unitless
  residualToleranceN: 1e-6,                               // N,   per newton of lift
  residualToleranceNm: 1e-6,                              // N m, per newton of lift
  hoverPitchWarningDeg: 1.0                               // deg; see section 5.1
}

Run = { id, createdAt, modelVersion, schemaVersion, inputHash, vehicle, gateSettings, results: GateResult }
```

Validation refinements to implement: axes non-zero; `spin` matches the sign of `km` (km = 0 allowed with either);
`turnLoss` in [0, 1]; `maxThrust > 0`; `mass > 0`; inertia diagonal positive; `thrustExponent` in [1, 3];
`hoverPitchDeg` in [-90, 90]; `windCases` non-empty; at least one enabled rotor; `source` non-empty unless
provenance is `unknown`; `esc.count` equals the number of rotors; `pack.cells` within the ESC cell range.

Export a JSON Schema from the Zod schema at build time to `packages/schema/dist/vehicle.schema.json`. **The JSON
Schema describes structure only** (fields, types, unit literals, enums, ranges expressible as min/max). It cannot
express cross-field refinements (spin versus km sign, cell range, count equality, source rules) or the axis
normalisation transform. Runtime Zod parsing is the authoritative validator; the JSON Schema exists for editors
and external tools, and the README must say so.

## 5. Physics core (`packages/core`)

All functions are pure, take plain numbers and arrays (already unwrapped from the tracked-quantity objects by a single
`toSI(vehicle)` adapter), and return plain objects. Suggested modules and the exact formulas.

### 5.1 Geometry
* `hoverRotation(phiRad): Mat3` as given in section 2.
* `deflectionRad(rotor)`: angle between `axis` and `ductAxis` (0 if `ductAxis` is null).
* `effectiveMaxThrust(rotor) = maxThrust * max(0, 1 - turnLoss * deflectionDeg / 90)`; 0 if disabled.
* `rotorsInHoverFrame(vehicle)`: for each enabled rotor, `r_i = R_h (p_i - cg)` and `a_i = R_h a_i`.
* **Which hover pitch drives the physics:** the stored `vehicle.hoverPitchDeg` does, always. The app also
  computes a geometric reference, `referenceHoverPitchDeg(vehicle)`, defined as the direction of the
  max-thrust-weighted sum of the structural thrust axes:
  `m = sum_i Tmax_eff_i * a_i` (structural frame, enabled rotors), `phi_ref = atan2(m_x, -m_z)` in degrees.
  This definition does not depend on the allocation, so it is not circular. The reference is displayed beside
  the input with the label "geometry reference"; if `|hoverPitchDeg - phi_ref| > hoverPitchWarningDeg` the UI
  shows a warning (not a gate failure) and offers a one-click "use reference", which writes `phi_ref` into the
  vehicle with provenance `derived`. The reference is never used silently.

### 5.2 Effectiveness and allocation
* `effectiveness(vehicle): number[6][n]`; column i is `[ r_i x a_i - km_i a_i ; a_i ]` (torque rows 0..2, force
  rows 3..5), in the hover frame, per newton of thrust.
* `allocate(E, wrench, options?)`: minimum-norm solution `u = pinv(E) w` (SVD tolerance 1e-9). Option
  `ignoreYaw` deletes row 2 before solving (diagnostic only).
* `hoverAllocation(vehicle)`: `u = pinv(E) [0,0,0,0,0,-1]`, then scale `u` so that `-(E[5] . u) = W`. Return the
  per-rotor thrusts, utilisation `u_i / Tmax_eff_i`, residual wrench `E u - [0,0,0,0,0,-W]`, total thrust,
  `W / total` (cosine efficiency), and the list of rotors with negative demand. If the residual force or
  roll/pitch torque exceeds `residualToleranceN` / `residualToleranceNm` per newton of lift the hover is **unbalanced**: the controller
  would have to lean, and the gate fails hover.
* `thrustToCommand(T, rotor) = (T / Tmax_eff)^(1/exponent)`, clamped to [0, 1].

### 5.3 Wrench of a thrust vector
* `wrench(vehicle, thrusts): { force[3], moment[3] }` in the hover frame: `F = sum T_i a_i`, `M = sum T_i (r_i x
  a_i - km_i a_i)`. Include gravity as a separate term `{ force: [0,0,+W], moment: 0 }` and expose
  `netWrench = thrust + gravity + disturbance`. The UI shows all four (thrust, gravity, disturbance, net).
* Per-rotor contributions are returned as an array so the table in section 7 is just a render of this result.

### 5.4 Control authority (linear, about the hover trim)
For each axis k in (roll, pitch, yaw) and sign s in (+1, -1):
```
d = pinv(E) . (s * e_k)          // thrust change per N m of pure torque, other wrench components held at zero
for each rotor i: limit_i = d_i > 0 ? (Tmax_eff_i - u_i) / d_i : d_i < 0 ? (0 - u_i) / d_i : +inf
tau(k, s) = min_i limit_i;  binding rotor = argmin
alpha(k, s) = tau / I_h[k][k]    where I_h = R_h I R_h^T, I built from the tensor convention in section 4
```
Also report `climbHeadroom`: with `d = pinv(E) [0,0,0,0,0,-1]`, the extra lift before the first rotor hits its
maximum, and `thrustToWeight = sum Tmax_eff / W`. Document in the UI that this is a linear estimate about the
trim; PX4's allocator can redistribute and do somewhat better, but not on a half-axis whose binding rotor is at
zero.

### 5.5 Disturbances (steady wind, vehicle at rest at the hover attitude)
**Definition:** `wind` is the **air-mass velocity** in NED (m/s), the direction the air moves. A "headwind" for
a vehicle facing north is `wind = (-6, 0, 0)`: air moving south. It is not the apparent airflow. The vehicle is
at rest and level in the hover frame with yaw zero, so a wind given in NED is the same vector in the hover frame.
The apparent air velocity felt by the vehicle in the structural frame is `v_air = R_h^T (v_vehicle - wind)` with
`v_vehicle = 0`, i.e. `v_air = R_h^T (-wind)`. All drag formulas below use `v_air`.
* **Ram drag of ducted rotors:** `mdot_i = sqrt(rho * A_i * T_i)` with `A_i = pi (d_i/2)^2`, `F_i = -mdot_i *
  v_air` applied at `p_i`. Use the hover allocation thrusts for `T_i`.
* **Body drag:** `F = -c (.) v_air (.) |v_air|` (element-wise) at `dragCenter`.
* **Foils:** in this MVP do not compute a foil force. Report each foil in the results with its
  `forceProvenance` and the text "not included in the balance" unless it is `measured`, in which case still do
  not include it (the strip model is out of scope) but flag "measured data present, model not implemented".
  Never let a foil silently contribute zero as if it were known.
Sum the forces and moments about the CG, rotate to the hover frame, and return them as the disturbance wrench.
Then `trimUnderDisturbance = hover thrusts + pinv(E) (-disturbanceWrench)`; report feasibility (all thrusts in
[0, Tmax_eff]) and max utilisation.

### 5.6 Power and battery
* Ideal ducted-fan power: `P_i = T_i^1.5 / (2 sqrt(rho A_i))` (momentum theory for a duct). Open propeller would
  be `T^1.5 / sqrt(2 rho A)`; all ATLAS rotors are ducted.
* Current per rotor from its `currentModel`; for `momentumTheory`, `I_i = P_i / (efficiency * V_pack)`.
* Cases: hover; hover plus the worst wind trim; every rotor at maximum. For each: per-rotor current versus ESC
  limit, total versus PDB and pack limits, pack C-rate, hover endurance `capacity / I_total` (hours).
* **Unknown-value propagation, exactly these rules:**
  - Currents are always computed, using the pack voltage even when its provenance is `estimated`, and are
    labelled "indicative" in the UI whenever the voltage is not `measured`.
  - A comparison against a limit whose provenance is `unknown` has status `unknown`; its placeholder number is
    never compared, never shown as a limit (show "unknown" in its place), and never influences pass/fail.
  - If `pack.capacity` is `unknown`, endurance and C-rate are `unknown` and displayed as such.
  - The power check status is `pass` only if every comparison it contains is `pass`; `fail` if any comparison
    fails against a limit whose provenance is `measured` or `estimated`; otherwise `unknown`.
  - The same three rules apply to any future limit: an `unknown` input can make a result `unknown`, it can
    never make it `pass` or `fail`.

### 5.7 Failure and CG cases
* **Single actuator failure:** for each enabled rotor j, set `Tmax_eff_j = 0` (remove its column), re-run
  `hoverAllocation`. Feasible if balanced and all remaining thrusts within [0, Tmax_eff] and utilisation below
  `failureMaxUtilisation`.
* **CG offsets:** for each offset in `gateSettings.cgOffsetsM`, shift the CG in the structural frame and re-run
  `hoverAllocation`; feasible if balanced, non-negative, utilisation below `cgOffsetMaxUtilisation`.

## 6. The control-authority gate

`runGate(vehicle, settings): GateResult` runs every check below and returns a structured result. Each check has
`status: "pass" | "fail" | "unknown"`, a one-sentence `reason`, the numbers it used, and the rotor indices that
were binding. **Verdict precedence, in this order and no other:**

1. if any check is `fail`, the verdict is `"not viable"` (a failure is a failure even when other inputs are unknown);
2. otherwise, if any check is `unknown`, the verdict is `"unknown"`;
3. otherwise `"viable"`.

Show the verdict with the list of failing checks first and unknown checks second; never show a green badge alone.

| check | pass condition |
|---|---|
| hover balance | residual force and roll/pitch/yaw torque below tolerance, no negative thrust |
| hover margin | max utilisation <= `hoverMaxUtilisation` |
| authority, six half-axes | each `tau(k, s) >= max(floor_k, marginFactor * worst disturbance moment on axis k across wind cases)`; two-sided means both signs pass for every axis |
| saturation under wind | every wind case trims with all rotors in [0, Tmax_eff] and utilisation <= `windMaxUtilisation` |
| CG offsets | every offset case feasible |
| single failure | every rotor-out case feasible (report which rotors are survivable even when the check fails) |
| power | all currents within limits for hover and worst wind; `unknown` if any limit is unknown |
| provenance | `unknown` if any input used by the checks above (mass, CG, rotor geometry, thrust, km) is `unknown`; this check cannot pass a vehicle, it can only downgrade the verdict, and it lists estimated inputs as warnings |

Also compute and show, outside the verdict, the **hover pitch window**: sweep `hoverPitchDeg` from -10 to +50
degrees in 0.5 degree steps. A pitch value is **feasible** when the hover allocation at that pitch satisfies all
three of: residual force and torque below `residualToleranceN` and `residualToleranceNm` per newton of lift; every thrust `>= 0`; every
thrust `<= Tmax_eff` (utilisation `<= 1.0`). Authority is **not** considered in the window. Report the feasible
interval containing the vehicle's value (or "none"), the sub-interval where max utilisation `<= hoverMaxUtilisation`,
and the pitch of minimum utilisation. This tells the user how much attitude margin the geometry gives.

## 7. Visualisation (`apps/web`)

Layout: left panel inputs (a form generated from the schema with unit labels, provenance selectors and
validation errors inline), centre 3D view, right panel results. Bottom: run history and comparison.

3D view (react-three-fiber, orthographic and perspective toggle):
* Axis triad labelled "x fwd / y right / z down (FRD)" with a second, dimmer triad for the hover frame, and a
  toggle to display everything in either frame.
* CG as a sphere; reference origin as a small cross.
* Each rotor: a disc at its application point, a thrust arrow along its axis with length proportional to the
  hover thrust (scale slider, with the N per metre printed), colour by utilisation (grey 0, green mid, red above
  0.85), and a thin line from the CG to the application point (the moment arm). Hovering a rotor shows its
  contribution numbers. Reaction torque as a small curved arrow, colour by spin.
* Gravity arrow at the CG. Net residual force arrow at the CG (should vanish in a balanced hover, which is the
  point of drawing it). Disturbance arrows when a wind case is selected.
* Body drag centre marker and, if foils exist, their planform outline drawn as a wireframe with the label
  "estimated, not in balance" or "unknown".

Right panel:
* Per-rotor table: name, position relative to CG (hover frame), axis (hover frame), effective max thrust, hover
  thrust, command, utilisation, force vector, moment vector (all six components with units), km, spin,
  provenance flags. Sortable, exportable as CSV.
* Wrench summary: thrust, gravity, disturbance, net, each as force and moment with units, plus the residual.
* Gate result as the table of section 6 with reasons and binding rotors; the hover pitch window; authority in
  N m and deg/s^2 per half-axis with the binding rotor named.
* Power table.
* Buttons: "Export JSON report", "Export CSV tables", "Save run".

Every number displayed comes from a `GateResult` or `Vehicle` object; the component receives it as a prop.

## 8. Runs, provenance and scenario comparison (`packages/report`, `apps/web`)

* `inputHash = sha256(canonicalJson(vehicle) + canonicalJson(gateSettings))` where `canonicalJson` sorts keys
  and formats numbers with 15 significant digits. Use the Web Crypto API in the browser, Node's `crypto` in
  tests.
* Saving a run stores the full `Run` object (section 4) in IndexedDB (use `idb`) and offers the JSON download.
  Runs are immutable; editing a vehicle and re-running creates a new run.
* Comparison view: pick up to four saved runs. Refuse to compare (with a clear message) unless `modelVersion`,
  `schemaVersion` and `gateSettings` are identical, so variants are always compared under identical assumptions;
  offer "re-run all with current model" to fix it. Show side by side: verdict, per-check status, total hover
  thrust, max utilisation, six authority values, worst wind utilisation, failure survivability count, power
  status, and a diff of the vehicle inputs (which fields differ, old and new values with units).
* Ship the three architectures of section 1 as fixtures in `packages/schema/fixtures/` and a "Load fixture"
  menu. The as-built fixture is in section 10.
* CSV export: one file per table (rotors, gate, authority, wind cases, failure cases, power), with a header row
  that includes the unit in each column name, e.g. `hover_thrust_N`, and a first comment line
  `# run <id> model <MODEL_VERSION> schema <SCHEMA_VERSION> hash <inputHash>`.

## 9. Acceptance tests (must all pass before the UI is started)

Write these first in `packages/core` against the as-built fixture of section 10 (architecture A). Tolerances:
thrust 0.1 N, moment 0.05 N m, angle 0.05 degrees, unless stated.

1. **Axes normalised:** rotor 9 and 10 axes parse to (0, ±0.5, -0.866025) from the raw (0, ±0.005, -0.00866).
2. **Effective max thrust:** rotors 1..8 = 30.89, 31.08, 31.26, 31.45 N (pairs), rotors 9, 10 = 33.3 N.
3. **Hover allocation A:** thrusts `[2.70, 15.84, 13.21, 9.40, 19.08, 6.65, 20.89, 7.24, 15.91, 15.91]` N; total
   126.8 N; weight 118.59 N; max utilisation 0.66 at rotor 7; residual wrench all components below 1e-6.
4. **Hover frame consistency:** the summed thrust vector in the *structural* frame is (51.98, 0, -106.58) N, which
   is 26.00 degrees from the structural -z axis, equal to the hover pitch. The geometry reference
   `referenceHoverPitchDeg` (max-thrust-weighted axes, section 5.1) is 26.57 degrees, so no warning is raised at
   the default 1.0 degree threshold.
5. **Asymmetry diagnosis:** with `ignoreYaw` the allocation is `[9.6, 9.1, 11.5, 11.1, 13.0, 12.7, 14.2, 14.0,
   15.9, 15.9]` (tolerance 0.15 N). With all `km = 0` the left/right split is still 54.1 N / 41.0 N (foil fans
   only, tolerance 0.2 N). Both facts must hold; they prove the asymmetry is geometric.
6. **Architecture B hover:** `[8.1, 11.0, 11.7, 10.9, 14.1, 11.4, 15.5, 12.5, 12.4, 15.0]` N, total 122.6 N
   (tolerance 0.15 N). **Architecture C hover:** `[9.3, 9.7, 11.4, 11.2, 13.0, 12.5, 14.3, 13.8, 12.7, 14.7]` N.
7. **Authority A (N m):** roll +16.46 / -2.79, pitch +1.70 / -9.45, yaw +0.39 / -2.35; binding rotors: roll+ 2,
   roll- 1, pitch+ 1, pitch- 8, yaw+ 1, yaw- 2, all hitting zero. Climb headroom 60.0 N. Thrust-to-weight 2.66.
   **Authority B:** roll 11.36 / 8.60, pitch 6.42 / 12.50, yaw 1.27 / 1.60; climb headroom 122.4 N.
8. **Hover pitch window A** (criteria of section 6: residual below tolerance, all thrusts in [0, Tmax_eff]):
   feasible for hover pitch in [24.0, 26.5] degrees on the 0.5 degree grid; infeasible at 22.0 (rotor 8 negative)
   and at 27.0 (rotor 1 negative); no rotor exceeds Tmax_eff anywhere in the feasible interval; minimum
   utilisation at 25.0 degrees, value 0.61 (tolerance 0.01).
9. **Disturbance A.** All arrays are hover-frame FRD `[x fwd, y right, z down]` for forces (N) and `[roll,
   pitch, yaw]` for moments (N m), using the hover thrusts of test 3 for the ram drag. Tolerances 0.05.

   | case | wind NED (m/s) | ram force | ram moment | body force | body moment |
   |---|---|---|---|---|---|
   | headwind | (-6, 0, 0) | [-20.96, 0, 0] | [0, -2.70, 0.07] | [-8.70, 0, -0.64] | [0, 0.63, 0] |
   | from the right | (0, -6, 0) | [0, -20.96, 0] | [2.70, 0, 0.83] | [0, -12.17, 0] | [-0.69, 0, -2.58] |

   Sign check: a headwind pushes the vehicle backwards (negative x), the ram drag on the fans, which sit below
   and behind the CG in the hover frame, pitches it nose-down (negative pitch); the body drag centre is ahead of
   the CG so body drag pitches nose-up (positive). Body force z is -0.64, i.e. 0.64 N upward, because the hover
   frame's z axis points down.
10. **Gate verdict A:** `not viable` by precedence rule 1; the authority check fails on yaw+, pitch+ and roll-
    with reasons naming rotor 1; saturation under the headwind case may pass or fail. **Gate verdict B:** `not
    viable`; roll and pitch authority pass the floors, yaw fails the 1.5 x crosswind margin (1.27 and 1.60 N m
    against about 2.6 N m). **Power check** is `unknown` for all three fixtures because the pack and PDB limits
    are unknown even though the ESC limit is measured; with every other check passing this would give
    `unknown`, but A and B have failures so they are `not viable`. The per-ESC comparison must still be computed
    and shown: at the hover allocation with the momentum-theory model at 0.55 efficiency and 22.2 V, the busiest
    rotor draws about 39 A and at full thrust about 78 A, both below the 160 A ESC limit.
11. **Properties:** mirroring a vehicle in y (negate y of positions and axes, negate km) mirrors the hover
    allocation exactly; translating the reference origin (all positions and the CG by the same vector) changes
    nothing; gravity appears exactly once in the net wrench (net = thrust + gravity + disturbance, and with zero
    thrust the net force equals +W in z).
12. **Schema:** a vehicle with a zero axis, with `spin: "ccw"` and `km < 0`, with a missing unit, with a
    physical quantity missing its `provenance`, with a non-empty-source rule broken (provenance `measured` or
    `estimated` but empty `source`), or with a pack cell count outside the ESC range fails to parse with a
    message naming the field. All three fixtures parse. JSON Schema is emitted and validates the fixtures.
13. **Report:** the same vehicle and settings produce the same `inputHash` regardless of key order; the CSV has
    unit suffixes in every numeric column header.

## 10. Fixture: architecture A, as built (schemaVersion "1.0.0")

Positions and axes in the structural frame (m, unitless). Rotors 1 to 8 have `ductAxis` (1, 0, 0), `turnLoss`
0.1; rotors 9 and 10 have `ductAxis` null. Per-quantity provenance for every rotor: `position` measured
("CAD PHASE_0_V4, jet exit measured on the mesh" for rotors 1 to 8, "CAD PHASE_0_V4" for 9 and 10); `axis`
estimated ("assumed jet deflection 25/30/35/40 deg" for 1 to 8, "CAD bracket angle" measured for 9 and 10);
`ductAxis` measured ("CAD"); `turnLoss` 0.1 estimated ("assumed"); `maxThrust` 33.3 N estimated ("vendor
figure, not measured on this vehicle"); `thrustExponent` 2.0 estimated ("assumed quadratic"); `km` -0.002 m
estimated ("assumed; never measured"); `diameter` 0.1032 m measured ("datasheet"); `currentModel`
momentumTheory with efficiency 0.55 estimated ("assumed"); `spin` "cw"; `enabled` true.

| # | name | position (x, y, z) | axis (x, y, z) |
|---|---|---|---|
| 1 | EDF_01_FOIL_LEFT_OUTER | (-0.402, -0.392, 0.365) | (0.422618, 0, -0.906308) |
| 2 | EDF_02_FOIL_RIGHT_OUTER | (-0.402, 0.392, 0.365) | (0.422618, 0, -0.906308) |
| 3 | EDF_03_FOIL_LEFT_MID_OUTER | (-0.373, -0.309, 0.310) | (0.5, 0, -0.866025) |
| 4 | EDF_04_FOIL_RIGHT_MID_OUTER | (-0.373, 0.309, 0.310) | (0.5, 0, -0.866025) |
| 5 | EDF_05_FOIL_LEFT_MID_INNER | (-0.346, -0.214, 0.256) | (0.573576, 0, -0.819152) |
| 6 | EDF_06_FOIL_RIGHT_MID_INNER | (-0.346, 0.214, 0.256) | (0.573576, 0, -0.819152) |
| 7 | EDF_07_FOIL_LEFT_INNER | (-0.318, -0.132, 0.204) | (0.642788, 0, -0.766044) |
| 8 | EDF_08_FOIL_RIGHT_INNER | (-0.318, 0.132, 0.204) | (0.642788, 0, -0.766044) |
| 9 | EDF_09_FRONT_AFT | (0.395, -0.01397, 0.0306) | (0, 0.005, -0.00866) raw, normalise |
| 10 | EDF_10_FRONT_FORWARD | (0.505, 0.01397, 0.0306) | (0, -0.005, -0.00866) raw, normalise |

Mass properties, each with its own provenance: `mass` 12.092 kg, measured ("weighing 2026-09-14"); `cg`
(-0.078364, -0.000109, 0.122775) m, estimated ("CAD part positions with CAD densities rescaled to the weighed
part masses; not measured by balancing"); `inertia` (0.645693, 0.861915, 1.294571) kg m^2, estimated ("CAD,
rescaled"); `inertiaProducts` (Pxy, Pxz, Pyz) = (-0.002557, 0.343564, 0.001129) kg m^2, estimated ("CAD,
rescaled"; tensor off-diagonals are the negatives of these).

Body: `dragQuadratic` (0.2592, 0.3381, 0.6347) N s^2/m^2, estimated ("bluff-body guess"); `dragCenter`
(0.1373, 0, 0.1646) m, estimated ("CAD volume centroid").

Foils (estimated planform, force provenance "estimated"): one entry, name "JET_FOIL_V1 pair", span 0.888 m, root
chord 0.31 m, tip chord 0.34 m, sweep 10 deg, dihedral -28.5 deg (tips down), incidence 0, lift slope 6.2832
1/rad, cd0 0.08, stall 12 deg.

Power. ESC, every field measured ("Flycolor datasheet"): model "Flycolor X-Cross HV3 160A 5-12S", one per
rotor (count 10), 160 A continuous and 180 A burst per channel, 5 to 12 cells, 0.132 kg each. Pack: the aircraft
flies on **6S** packs (`cells` 6, `nominalVoltage` 22.2 V, both estimated, source "owner statement 2026-09-21,
pack label not yet read"); `capacity` 20 A h and `maxContinuousCurrent` 200 A are **unknown** placeholders whose
values must not be trusted. PDB `maxContinuousCurrent` 300 A, unknown placeholder. Do not use 12S anywhere in
the fixtures; the ESC accepts 6S, so the cell-range refinement passes. The UI must show the pack capacity, pack
current and PDB as unknown, the cell count and voltage as estimated, and the ESC as measured.

`hoverPitchDeg` 26.0, provenance `estimated`, source "chosen in closed-loop flight tests 2026-09-17; the
geometry reference of section 5.1 is 26.57 deg". The stored 26.0 drives the physics; the app shows 26.57 as the
reference and no warning, because the difference is below `hoverPitchWarningDeg`.

Fixtures B and C keep every value and provenance of A except the fields named in section 1.

Architecture B: same, with rotors 9 and 10 `axis` (0, 0, -1). Architecture C: as B with rotors 1, 3, 5, 7 `km`
+0.002 and `spin` "ccw".

## 11. Build order and definition of done

Work in this order and do not start a step before the previous one's tests pass:

0. **Preflight consistency check, before any application code.** Write a standalone script
   (`tools/preflight.ts`, plain TypeScript run with `tsx`, or a Python script; it must not import from the
   packages you are about to write) that types in fixture A from section 10 and computes, from the formulas of
   section 5 alone, every oracle number in section 9: effective max thrusts, hover thrusts for A, B and C, the
   `ignoreYaw` and `km = 0` diagnostics, the six authority values with binding rotors, climb headroom, the pitch
   window, and the four disturbance wrenches. Print each computed value next to the oracle and the difference.
   **If any value is outside its tolerance, stop.** Do not proceed to step 1, do not change a formula or an
   oracle, and report the mismatch (which value, computed versus expected, and your reading of the formula) as
   your final output. Only when every value matches, save the script as `tools/preflight.ts` (or `.py`) and its
   printed comparison as `tools/preflight-report.txt` in the repository, and continue. This script becomes the
   first test in `packages/core`.
1. `packages/schema`: Zod schemas, tracked-quantity helper, refinements, fixtures A/B/C, JSON Schema emit, tests 12.
2. `packages/core`: geometry, effectiveness, allocation, wrench; tests 1 to 6 and 11.
3. `packages/core`: authority, pitch window, disturbances, failure and CG cases, power; tests 7 to 9.
4. `packages/core`: gate; test 10. `packages/report`: hashing, JSON, CSV; test 13.
5. `apps/web`: schema-driven input form with units and provenance, results tables, gate panel, export, run
   history in IndexedDB.
6. `apps/web`: 3D view with arrows, moment arms, frame triads, per-rotor hover details.
7. `apps/web`: comparison view for saved runs with identical assumptions.
8. README: how to run, the conventions of section 2 verbatim, what is estimated and unknown in the fixtures, and
   the statement that a `viable` verdict requires measured inputs and known power limits.

Definition of done: `pnpm typecheck && pnpm lint && pnpm test && pnpm build` all pass; every acceptance test in
section 9 is implemented and green; loading fixture A in the UI shows the same hover thrusts as test 3, a red
"not viable / unknown" verdict, and arrows whose net residual vanishes; the comparison view shows A, B and C side
by side from three saved runs; exporting a run and re-importing it reproduces the same `inputHash`.

## 12. Rules on mismatches

* If you are unsure about a formula, use the one written here.
* **Never adjust a formula, a tolerance, a fixture value or an oracle to make a test pass.** An oracle mismatch
  means either your implementation or this document is wrong, and you cannot tell which from inside the code.
  Stop, report the mismatch (value, expected, computed, the formula as you read it), and wait for the user.
* If two statements in this document contradict each other, stop and report both, quoting them. Do not choose.
* Tests marked with a tolerance must use that tolerance exactly; do not widen it.
* Do not add physics that is not specified here (no foil forces, no ground effect, no motor dynamics), even as
  a disabled option.
