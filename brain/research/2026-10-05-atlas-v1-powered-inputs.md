# Powered forward flight: remaining inputs

2026-10-05T22:17:30.754170+00:00

Confirmed user requirement: **fans operate during forward flight**. Keep actual span3.805m, nose+Y, pilot canopy and intentional head/tail-light gaps. Existing isolated-section XFOIL runs do not include powered flow; this confirmation does not retrospectively change their meaning.

## Verified evidence

- Correct source blender/atlas_v1.stp, SHA256 unchanged. source_inventory.json records one root, three OPEN_SHELLs,420 faces, zero solids. Direct STEP metadata contains a generic PRODUCT('Document','Document') and no identified fan assemblies. No analytic CIRCLE/CYLINDRICAL_SURFACE entities were found; the surfaces are not a labelled catalogue of rotor disks.
- Existing exact-STEP mesh views show approximately circular openings in the lower shell, including three per side. They are candidate flow openings, not verified fan disks, blade geometry, fan count or operating axes. The broad source is body/wing/canopy skin geometry; no separate identifiable fan/rotor assembly or actuator-surface mapping has been verified. We cannot conclude every possible fan-related surface is absent merely because labels/solids are absent.
- Searches of project brain, airframe JSON and documentation found no explicit performance/placement mapping to atlas_v1.stp or this full-scale aircraft. Consequently no numerical centres, directions, rotor diameter/count, RPM, thrust, mass flow or pressure jump are approved inputs yet.
- brain/architecture/atlas-v3-small.md explicitly belongs to SMALL_SCALE_V3.step: nine fans, six foil fans with106mm ducts/96mm discs and three nose XFLY80; these are NOT full-scale inputs. The V3_30 CFD point3x36N,14.4kg mass and jet angles belong to another geometry. brain/architecture/vibration-model.md labels30000rpm as a default and actual RPM as unknown. None is transferred.

## Smallest useful questions for the user

1. Which fan assembly/specification belongs to this full-scale atlas_v1? A referenced assembly or table giving count, centres, shaft/flow directions and rotor diameters is sufficient; identify which existing openings each fan feeds. This resolves geometry mapping without guessing from the small prototype.
2. What operating point should the first powered forward-flight case use per fan or symmetric group? Provide a calibrated thrust/flow/pressure-rise value, or the selected fan model plus RPM and its performance curve. A throttle percentage alone, or RPM without a fan characteristic, does not define a flow boundary.

For a bounded first actuator-disk approximation, rotor diameter/location/axis and a defined loading law or operating thrust can establish an explicitly approximate momentum source; blade-resolved modelling is not required by default. A pressure-jump-versus-flow curve or calibrated operating flow is better for interaction with inlet speed. Rotation/swirl or torque data become necessary if swirl is represented; do not invent them or quietly claim powered fidelity from a uniform jet assumption.

These two groups are the essential powered-input blockers. A small speed/incidence matrix, computational boundaries and turbulence sensitivity can then be proposed for review. Do not request mass/CG merely to calculate aerodynamic force coefficients; those become necessary for trim or flight dynamics. Preserve intentional lighting gaps and canopy while defining their physical CFD boundary treatment. NoCFD, new AgentA handoff, CAD change or upload was performed.
