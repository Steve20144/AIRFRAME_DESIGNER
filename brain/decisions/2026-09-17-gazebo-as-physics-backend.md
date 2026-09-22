# Decision: Gazebo attaches as a physics backend, not via PX4's gz_bridge

Date: 2026-09-17
Status: accepted

## Context

A sibling project (`Utopia Labs/mujoco-vibe`, actually `atlas-sim`: Gazebo Harmonic + PX4 SITL, CAD pipeline,
C++ aero plugin, React dashboard) models the same ten-EDF ATLAS from the other end. Both should merge. Across all
36 recorded runs of the real CAD aircraft in atlas-sim every EDF shows zero command: its motors never turned.
AIRFRAME_DESIGNER flies the same aircraft once PX4 parameter seeding is fixed.

## Decision

Gazebo becomes an alternative physics engine under AIRFRAME_DESIGNER's existing HIL MAVLink link, the same seam
`dynamics/jsbsim_backend.py` uses. Gazebo supplies 6-DOF integration, mesh contacts and visuals; AIRFRAME_DESIGNER
keeps its aero, rotor and sensor models, scenarios, metrics and PX4 coupling. Vehicle truth is the CAD figure
(11.19 kg then, 12.09 kg weighed later), not `atlas_08.json`'s 13 kg. A new repository takes AIRFRAME_DESIGNER as
the base and grafts atlas-sim's history in.

## Reasoning

Design search first, then an evidence path a chosen design must pass before hardware. The Python engine is fast
enough for search; Gazebo adds fidelity where it matters (contacts, visuals) without owning the loop.

## Alternatives Considered

- PX4's `gz_bridge`: gives Gazebo the loop and loses the scenario runner, metrics and headless speed.
- Continue two projects: duplicated aircraft models that already disagree.

## Consequences

`dynamics/gazebo_backend.py` exists as a prototype; stepping from Python is fast, one slow readback path remains
unexplained. JSBSim is the cross-check engine used in practice (`compare` subcommand).
