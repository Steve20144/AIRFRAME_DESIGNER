# Product Vision

## Problem

ATLAS is a 12 kg aircraft lifted by ten 80 mm electric ducted fans: eight inside two jetfoils that bend the jets
down and aft, two on a nose bracket. It hovers pitched about 25 degrees nose-up and cannot be armed level: every
flight starts with a rotation on its stand and ends with the reverse. Conventional multirotor tooling assumes none
of this. A design change (fan station, bracket cant, stand geometry, CG) or a PX4 parameter change had no way to be
judged before a risky hardware test.

## Target User

The Utopia Labs ATLAS team: the engineer who designs the airframe, the pilot who flies it indoors with a remote,
and AI agents asked to run studies on it.

## Vision

A simulator with the real flight controller software in the loop, where the exact geometry drawn in 3D is what PX4
flies (rotor positions and axes exported 1:1 into the control allocator), running interactively for a pilot or
headless and unthrottled for parameter searches, and connectable to the real Pixhawk so the same flight can be
repeated in hardware-in-the-loop before the aircraft leaves its stand.

## Core Value Proposition

- One geometry, three ways to fly it: live SITL with a USB remote, headless batches at 5 to 7x real time per
  worker, and HITL on the board.
- Ground sequences (nose lift, nose lower, stands) are first-class, because on ATLAS they are the dangerous part.
- Every flight leaves a time series with PX4's own setpoints and estimates next to the truth, so tuning and HITL
  comparisons argue from data.
- Physics that can be cross-checked (Python rigid body vs JSBSim) and analysis that says why a configuration fails
  (authority per axis, allocator feasibility, lift margin), not just pass or fail.

## Non-Goals

- Photorealistic rendering or CAD editing (the STEP lives in Fusion; the app shows a parametric or STL view).
- Replacing PX4's estimator or controllers with our own.
- Modelling the indoor environment (walls, ground effect, GPS denial) beyond a simulated GPS and wind.
- A general multirotor design tool: presets exist, but every feature earns its place through ATLAS.
