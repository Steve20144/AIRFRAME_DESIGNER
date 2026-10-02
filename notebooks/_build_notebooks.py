"""Regenerates notebooks/*.ipynb (theory, plain-language notes, runnable cells on the real code).

Run: .venv/bin/python notebooks/_build_notebooks.py, then execute them in Jupyter (kernel "AIRFRAME_DESIGNER (.venv)").
Edit the cell text here rather than in the .ipynb files, or the next rebuild overwrites the edits."""
import pathlib
import nbformat as nbf

OUT = pathlib.Path(__file__).resolve().parent
OUT.mkdir(exist_ok=True)
KERNEL = {"name": "airframe-designer", "display_name": "AIRFRAME_DESIGNER (.venv)", "language": "python"}

SETUP = r'''# Setup: make the repository importable and load the V3 airframe used throughout these notebooks
import sys, math, pathlib
ROOT = pathlib.Path.cwd().resolve()
if ROOT.name == "notebooks":
    ROOT = ROOT.parent
sys.path.insert(0, str(ROOT))

import numpy as np
import matplotlib.pyplot as plt
from airframe_designer.geometry.airframe import Airframe

np.set_printoptions(precision=4, suppress=True)
plt.rcParams.update({"figure.figsize": (7, 3.6), "axes.grid": True, "grid.alpha": 0.3})

AF_PATH = ROOT / "airframes" / "atlas_v3_v34_foils_50_65_50_tuned.json"
af = Airframe.load(AF_PATH)
af.resolve_mass()
G = 9.80665
print(f"{af.name}: {af.mass.mass:.2f} kg, {len(af.active_rotors())} fans, hover pitch {af.hover_pitch_deg} deg, parked {af.landed_pitch_deg} deg")'''


def md(text):
    return nbf.v4.new_markdown_cell(text.strip("\n"))


def code(text):
    return nbf.v4.new_code_cell(text.strip("\n"))


def note(title, text):
    return md(f"> **Note: {title}**\n>\n" + "\n".join("> " + line if line else ">" for line in text.strip("\n").splitlines()))


def save(name, cells):
    nb = nbf.v4.new_notebook()
    nb.cells = cells
    nb.metadata["kernelspec"] = KERNEL
    nb.metadata["language_info"] = {"name": "python"}
    nbf.write(nb, OUT / name)
    print("wrote", name)


# =====================================================================================================================
save("00_index.ipynb", [
md(r'''
# ATLAS simulator: theory notebooks

These notebooks walk through the physics inside `airframe_designer/`, the simulator that flies the ATLAS V3 model against PX4 (SITL and HITL). Each notebook covers one aspect. Every important code cell has the theory above it and a short **Note** in plain words, and the code calls the simulator's real functions on the real V3 airframe, so the numbers you see are the ones the simulator uses.

| Notebook | Aspect | Main code it uses |
|---|---|---|
| [01_frames](01_frames.ipynb) | World/inertial frame, structural, hover, rotor and sensor frames, pitch rotations | `geometry/airframe.py`, `geometry/frames.py` |
| [02_quaternions](02_quaternions.ipynb) | Unit quaternions, Hamilton product, attitude kinematics, the 90° singularity | `dynamics/quaternion.py` |
| [03_equations_of_motion](03_equations_of_motion.ipynb) | State vector, Newton and Euler equations, inertia and the parallel-axis theorem, fan spool lag, the integrator | `dynamics/rigid_body.py`, `geometry/mass.py`, `aero/rotor_aero.py` |
| [04_ground_contact](04_ground_contact.ipynb) | Legs as algebraic penalty springs, friction, parking angle | `dynamics/contact.py`, `geometry/gear.py` |
| [05_sensors](05_sensors.ipynb) | Specific force, gyro, noise, the H-FLOW model and its EKF2 settings | `sensors/models.py`, `sensors/flow.py` |
| [06_fans_jetfoils_coanda](06_fans_jetfoils_coanda.ipynb) | Fan thrust, jetfoil turning, turning loss, ram drag, ducted-fan momentum theory, Coanda flow, the PX4 mixer export | `geometry/propulsion.py`, `aero/rotor_aero.py`, `geometry/airframe.py` |
| [07_jsbsim_and_chrono](07_jsbsim_and_chrono.ipynb) | JSBSim as a second physics backend through Python (`FGFDMExec`), and where Project Chrono would fit | `dynamics/jsbsim_backend.py` |

**Recommended order:** 01 → 02 → 03 → 04 → 05 → 06 → 07. Each one can also be run on its own.

## How to open and run them

- In **VS Code**: open a notebook and pick the kernel **AIRFRAME_DESIGNER (.venv)**.
- In **Jupyter Lab**: `uv pip install --python .venv/bin/python jupyterlab`, then `.venv/bin/jupyter lab notebooks/`.

Every notebook starts with the same setup cell. It adds the repository to the Python path and loads `airframes/atlas_v3_v34_foils_50_65_50_tuned.json` (the V3 model with 50 / 65 / 50° foils and the tuned gains on the board).

The companion report with the same material in long form is `docs/ATLAS_Simulator_Physics.pdf`.
'''),
code(SETUP),
md(r'''
## Conventions used everywhere

| Symbol | Meaning |
|---|---|
| $N$ superscript | components in the world NED frame (North, East, Down) |
| $B$ superscript | components in the body (structural, FRD) frame |
| $\mathbf q = [w, x, y, z]$ | attitude quaternion, scalar first, Hamilton product, body → NED |
| $R(\mathbf q)$ | rotation matrix, $\mathbf v^N = R\,\mathbf v^B$ |
| $R_h$ | structural → hover (PX4 body) frame, a pitch of $\theta_h$ |
| $\boldsymbol\omega^B = (p, q, r)$ | body angular velocity, rad/s |
| $\bar\Omega_i$ | fan $i$ speed as a fraction of full, 0 to 1 |
| $\Delta t$ | physics step, 1 ms |
'''),
])

# =====================================================================================================================
save("01_frames.ipynb", [
md(r'''
# 01 · Frames: world, structural, hover, rotor and sensor

Every vector in the simulator is a list of three numbers, and those numbers only mean something once you know which axes they are measured along. This notebook goes through each frame the simulator uses and the rotations between them.
'''),
code(SETUP),
md(r'''
## 1. The world frame and why it can be treated as inertial

**Theory.** Newton's law $\mathbf F = m\mathbf a$ holds as written only in an inertial frame. Seen from a frame $B$ that rotates at $\boldsymbol\omega$, the rate of change of any vector $\mathbf b$ picks up an extra term (the transport theorem):

$$\left.\frac{d\mathbf b}{dt}\right|_I = \left.\frac{d\mathbf b}{dt}\right|_B + \boldsymbol\omega\times\mathbf b .$$

Applied to motion over the rotating Earth, the acceleration gains a Coriolis and a centripetal term:

$$\mathbf a_I = \mathbf a_E + 2\,\boldsymbol\Omega_E\times\mathbf v_E + \boldsymbol\Omega_E\times(\boldsymbol\Omega_E\times\mathbf r),\qquad \Omega_E = 7.29\times10^{-5}\ \text{rad/s}.$$

The simulator uses a **flat, non-rotating North-East-Down** frame with its origin on the floor at the home point, and drops both terms. The cell below shows how small they are for ATLAS.
'''),
note("what \"inertial\" means here", r'''
An inertial frame is one that is not accelerating or spinning. The room is not quite one, because the Earth spins once a day. But the Earth spins so slowly that, for an aircraft moving at walking speed for a few minutes, the effect is tens of thousands of times smaller than gravity. So the simulator pretends the room is perfectly still and flat, which keeps the equations simple.'''),
code(r'''
Omega_E, R_E = 7.2921e-5, 6.371e6
for v in (0.5, 2.0, 5.0):
    cor = 2 * Omega_E * v
    print(f"Coriolis at {v:3.1f} m/s: {cor:.2e} m/s^2 = {cor / G:.1e} g")
cent = Omega_E**2 * R_E
print(f"Centripetal at the equator: {cent:.3f} m/s^2 = {100 * cent / G:.2f} % of g (constant, absorbed into g)")
'''),
md(r'''
## 2. The frames that move with the aircraft

| Frame | Axes and origin | Used for |
|---|---|---|
| **World** (NED) | North, East, Down; home point on the floor | position, velocity, gravity, wind, floor |
| **Structural** = simulator body frame | Forward, Right, Down (FRD), fixed to the CAD; moments about the CG | rotor geometry, inertia, forces, angular velocity, IMU |
| **Hover** = PX4 body frame | structural frame pitched nose-up by $\theta_h$ | what PX4 calls "level"; told with `SENS_BOARD_Y_OFF` |
| **Rotor** | position $\mathbf r_i$ and unit push direction $\hat{\mathbf a}_i$ of fan $i$, in structural axes | thrust and reaction torque; exported to PX4 in hover axes |
| **Sensor** | IMU in structural axes; H-FLOW fixed to the frame, tilted 20.4° | what each sensor reports |
'''),
note("the hover frame", r'''
When ATLAS hovers still, its structure is tilted nose-up by 23.85°, because that is where the nine tilted jets balance. PX4 wants "level" to mean "hovering still", so it uses axes tilted along with the aircraft: in a hover, the hover frame's x axis is horizontal and its z axis points straight down. Stick centred, roll 0 and pitch 0 all refer to this frame.'''),
md(r'''
## 3. The pitch rotation between structural and hover axes

**Theory.** A rotation about the y (wing) axis by $\theta_h$ leaves $y$ alone and mixes $x$ and $z$:

$$\mathbf v_{\text{hover}} = R_h\,\mathbf v_{\text{struct}},\qquad R_h = \begin{bmatrix}\cos\theta_h & 0 & \sin\theta_h\\ 0&1&0\\ -\sin\theta_h & 0 & \cos\theta_h\end{bmatrix},\qquad R_h^{-1} = R_h^{\mathsf T}.$$

`Airframe.hover_rotation()` builds exactly this matrix.
'''),
code(r'''
Rh = af.hover_rotation()
print("R_h =\n", Rh)
print("orthonormal (R_h R_h^T = I):", np.allclose(Rh @ Rh.T, np.eye(3)), "  det =", round(np.linalg.det(Rh), 6))
print("structural forward axis (1,0,0) in hover axes:", Rh @ [1, 0, 0])
'''),
note("reading the result", r'''
The structural nose axis comes out as (0.915, 0, −0.404) in hover axes. Negative z means up, so in a hover the nose points 0.40 upward for every 0.915 forward: that is the 23.85° nose-up attitude seen from PX4's level frame.'''),
md(r'''
### A measurement from the board (30 Sep, HITL)

Parked at −16°, the Pixhawk's raw accelerometer (structural axes) read $(-2.71, 0.04, -9.43)\ \text{m/s}^2$, and after PX4 applied its 23.85° board rotation it reported $(-6.26, 0.13, -7.52)$. The same rotation done by hand:
'''),
code(r'''
raw_struct = np.array([-2.71, 0.04, -9.43])
print("R_h @ raw   =", Rh @ raw_struct)
print("board said = [-6.26  0.13 -7.52]")
'''),
md(r'''
## 4. The rotor frame: where each fan pushes, and in which direction

**Theory.** Fan $i$ is described by the point where its jet force acts, $\mathbf r_i$, and a unit push direction $\hat{\mathbf a}_i$. Its jetfoil turns the jet by the exit angle $\delta$, so the push is $\hat{\mathbf a} = (\cos\delta, 0, -\sin\delta)$ in structural axes (notebook 06). PX4 receives both in **hover axes, relative to the CG**: $R_h(\mathbf r_i - \mathbf r_{CG})$ and $R_h\hat{\mathbf a}_i$. `Airframe.rotors_in_px4_frame()` does that conversion.
'''),
code(r'''
cg = af.cg
print(f"{'fan':10s} {'exit deg':>8s}  {'push, structural':>24s}  {'push, hover':>24s}  {'up share':>8s}")
for r, (p_h, a_h) in zip(af.active_rotors(), af.rotors_in_px4_frame()):
    a_s = np.asarray(r.axis, float) / np.linalg.norm(r.axis)
    print(f"{r.name[:10]:10s} {r.deflection_deg():8.1f}  {str(np.round(a_s, 3)):>24s}  {str(np.round(a_h, 3)):>24s}  {-a_h[2]:8.3f}")
'''),
code(r'''
# Side view in hover axes (x forward, z down, so plot -z as up): each fan's position and push direction
fig, ax = plt.subplots(figsize=(7, 4))
for r, (p_h, a_h) in zip(af.active_rotors(), af.rotors_in_px4_frame()):
    ax.plot(p_h[0], -p_h[2], "o", color="C0")
    ax.annotate("", xy=(p_h[0] + 0.12 * a_h[0], -p_h[2] - 0.12 * a_h[2]), xytext=(p_h[0], -p_h[2]),
                arrowprops=dict(arrowstyle="->", color="C1"))
ax.plot(0, 0, "k+", ms=14, mew=2); ax.text(0.01, 0.01, "CG")
ax.set_xlabel("x hover, forward (m)"); ax.set_ylabel("up (m)"); ax.set_aspect("equal")
ax.set_title("Fan force points and push directions in PX4's hover frame")
plt.show()
'''),
note("what PX4 does with these numbers", r'''
PX4's motor mixer is given these hover-axes positions and directions as `CA_ROTORi_PX/PY/PZ` and `CA_ROTORi_AX/AY/AZ`. From them it works out how much each fan contributes to roll, pitch, yaw and lift, and inverts that to decide which fans to speed up for a given command. If these numbers are wrong (for example in structural instead of hover axes), every command goes partly to the wrong axis.'''),
md(r'''
## 5. A frame mismatch found on 30 Sep: the flow gyro

**Theory.** A pure yaw rotation at rate $r$ about the **hover** vertical is the vector $(0, 0, r)$ in hover axes. In structural axes it is $R_h^{\mathsf T}(0, 0, r) = (-r\sin\theta_h,\ 0,\ r\cos\theta_h)$. PX4's optical-flow module, when the flow sensor sends no gyro, integrates the raw gyro, which is in structural axes, and uses its x and y parts as roll and pitch rates.
'''),
code(r'''
yaw_rate = np.array([0, 0, 1.0])        # 1 rad/s about the hover vertical
seen_raw = Rh.T @ yaw_rate
print("yaw 1 rad/s seen by the raw gyro (structural axes):", np.round(seen_raw, 3))
print(f"=> {abs(seen_raw[0]) * 100:.0f} % of every yaw rate looks like a roll rate to the flow compensation")
'''),
note("why it matters and what fixes it", r'''
The camera sees the floor move when the aircraft slides and also when it rotates. PX4 subtracts the rotation part using a gyro, and the gyro must use the same axes as the camera. With the raw structural gyro, 40 % of each yaw turn is subtracted as if it were roll, so every turn looks like a small sideways slide. The simulated H-FLOW now sends its own gyro reading; on the real aircraft, `EKF2_OF_GYR_SRC = 1` makes the estimator use its own, correctly rotated gyro. (This was a real mismatch, but the main cause of the heading jitter seen that day was the flow noise; see notebook 05.)'''),
md(r'''
## 6. Where the round Earth still appears: GPS

**Theory.** The GPS model converts NED metres to latitude, longitude and altitude with a small-offset (equirectangular) approximation around the home point:

$$\varphi = \varphi_0 + \frac{n}{R_E},\qquad \lambda = \lambda_0 + \frac{e}{R_E\cos\varphi_0},\qquad h = h_0 - d .$$
'''),
code(r'''
from airframe_designer.sensors.models import Home
home = Home()
n, e, d = 100.0, 50.0, -2.0           # 100 m north, 50 m east, 2 m up
lat = home.lat + math.degrees(n / R_E)
lon = home.lon + math.degrees(e / (R_E * math.cos(math.radians(home.lat))))
print(f"home {home.lat:.6f}, {home.lon:.6f}, {home.alt} m  ->  {lat:.6f}, {lon:.6f}, {home.alt - d} m")
'''),
])

# =====================================================================================================================
save("02_quaternions.ipynb", [
md(r'''
# 02 · Quaternions

The simulator stores the aircraft's attitude as a unit quaternion and moves it forward 1000 times a second. This notebook explains what the four numbers mean, how they multiply, how they are propagated, and why Euler angles are avoided for this job.
'''),
code(SETUP),
code(r'''
from airframe_designer.dynamics.quaternion import q_from_euler, q_to_rotmat, q_to_euler, q_deriv, q_normalize
'''),
md(r'''
## 1. Body-to-NED unit quaternion

**Theory.** A rotation by angle $\vartheta$ about the unit axis $\hat{\mathbf n}$ is

$$\mathbf q = \begin{bmatrix} w\\ x\\ y\\ z\end{bmatrix} = \begin{bmatrix}\cos(\vartheta/2)\\ \hat{\mathbf n}\sin(\vartheta/2)\end{bmatrix},\qquad w^2+x^2+y^2+z^2 = 1.$$

The code stores it scalar first, and it rotates **body (structural FRD) vectors into NED**: $\mathbf v^N = R(\mathbf q)\,\mathbf v^B$, with

$$R(\mathbf q) = \begin{bmatrix} 1-2(y^2+z^2) & 2(xy - wz) & 2(xz + wy)\\ 2(xy + wz) & 1-2(x^2+z^2) & 2(yz - wx)\\ 2(xz - wy) & 2(yz + wx) & 1-2(x^2+y^2)\end{bmatrix}.$$
'''),
note("what the four numbers are", r'''
Any orientation can be reached from "level, facing north" by one single turn about some axis. The quaternion stores that turn: the first number is the cosine of half the angle, the other three are the axis scaled by the sine of half the angle. **Unit** means the four squares add up to 1, which makes it a pure rotation. **Body-to-NED** means it takes an arrow given in the aircraft's own axes and tells you where it points in the room.'''),
code(r'''
cases = {
    "level, facing north": (0, 0, 0),
    "hover, 23.85 deg nose-up": (0, af.hover_pitch_deg, 0),
    "parked, 16 deg nose-down": (0, -16.0, 0),
    "level, facing east": (0, 0, 90),
}
for name, (r_, p_, y_) in cases.items():
    q = q_from_euler(math.radians(r_), math.radians(p_), math.radians(y_))
    nose = q_to_rotmat(q) @ [1, 0, 0]
    print(f"{name:26s} q = {np.round(q, 3)}   |q| = {np.linalg.norm(q):.6f}   nose in NED = {np.round(nose, 3)}")
'''),
note("reading the hover line", r'''
A 23.85° turn about the right wing gives $w = \cos 11.9^\circ = 0.978$ and $y = \sin 11.9^\circ = 0.207$. The nose then points 0.404 "up" (negative down) for every 0.915 north. Note that $\mathbf q$ and $-\mathbf q$ describe the same attitude, a side effect of the half angles.'''),
md(r'''
## 2. The Hamilton product

**Theory.** Writing each quaternion as a number plus an arrow, $\mathbf a = (a_0, \vec a)$ and $\mathbf b = (b_0, \vec b)$:

$$\mathbf a\otimes\mathbf b = \big(a_0 b_0 - \vec a\cdot\vec b,\ \ a_0\vec b + b_0\vec a + \vec a\times\vec b\big).$$

It chains rotations ($\mathbf q_1\otimes\mathbf q_2$ is "turn by $\mathbf q_1$, then by $\mathbf q_2$ in the already-turned axes") and rotates vectors: $(0, \vec v^N) = \mathbf q\otimes(0, \vec v^B)\otimes\mathbf q^*$, with $\mathbf q^* = (w, -x, -y, -z)$. The code does not need a general product function; the cell defines one so it can be checked against the code's own matrix.
'''),
code(r'''
def qmul(a, b):
    """Hamilton product, scalar first."""
    a0, av = a[0], np.asarray(a[1:]); b0, bv = b[0], np.asarray(b[1:])
    return np.concatenate([[a0 * b0 - av @ bv], a0 * bv + b0 * av + np.cross(av, bv)])

def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

q = q_from_euler(math.radians(10), math.radians(23.85), math.radians(40))
v_body = np.array([1.0, 0.2, -0.3])
via_product = qmul(qmul(q, np.r_[0, v_body]), qconj(q))[1:]
via_matrix = q_to_rotmat(q) @ v_body
print("q (x) v (x) q*  =", via_product)
print("R(q) v          =", via_matrix, "  same:", np.allclose(via_product, via_matrix))
'''),
note("order matters, and the convention matters", r'''
Because of the cross product, $\mathbf a\otimes\mathbf b \neq \mathbf b\otimes\mathbf a$: yaw a book 90° then pitch it 90° and it ends up somewhere different than pitching first (next cell). There are two conventions in use: Hamilton's ($i\,j = k$, used here and by PX4) and the "JPL" one ($i\,j = -k$) found in some spacecraft software. Mixing them silently reverses chained rotations. This simulator and PX4 both use Hamilton, scalar first, so quaternions pass between them unchanged.'''),
code(r'''
yaw90 = q_from_euler(0, 0, math.radians(90))
pitch90 = q_from_euler(0, math.radians(90), 0)
for name, qq in (("yaw then pitch", qmul(yaw90, pitch90)), ("pitch then yaw", qmul(pitch90, yaw90))):
    print(f"{name}: nose points {np.round(q_to_rotmat(qq) @ [1, 0, 0], 3)} in NED")
'''),
md(r'''
## 3. How the attitude moves: the kinematic equation

**Theory.** With body rates $\boldsymbol\omega^B = (p, q, r)$,

$$\dot{\mathbf q} = \tfrac12\,\mathbf q\otimes\begin{bmatrix}0\\ \boldsymbol\omega^B\end{bmatrix},$$

the quaternion form of $\dot R = R\,[\boldsymbol\omega^B]_\times$. `q_deriv` implements it; the cell checks it against the Hamilton product.
'''),
code(r'''
w = np.array([0.3, -0.2, 0.5])
print("q_deriv(q, w)          =", q_deriv(q, w))
print("0.5 * q (x) [0, w]     =", 0.5 * qmul(q, np.r_[0, w]))
'''),
md(r'''
**Theory, discrete step.** Each 1 ms substep takes one explicit step, then divides by the norm:

$$\mathbf q_{k+1} = \frac{\mathbf q_k + \dot{\mathbf q}\,\Delta t}{\lVert \mathbf q_k + \dot{\mathbf q}\,\Delta t\rVert}.$$
'''),
note("why the rescaling is needed", r'''
A straight-line step leaves the unit sphere a little every time, so without rescaling the quaternion slowly grows and stops being a pure rotation (it would start to stretch vectors). The cell spins at 3 rad/s for 10 s with and without rescaling.'''),
code(r'''
dt, T, w = 1e-3, 10.0, np.array([0.0, 0.0, 3.0])
qa = np.array([1.0, 0, 0, 0]); qb = qa.copy()
for _ in range(int(T / dt)):
    qa = qa + q_deriv(qa, w) * dt
    qb = q_normalize(qb + q_deriv(qb, w) * dt)
print(f"after {T:.0f} s at 3 rad/s: |q| without rescaling = {np.linalg.norm(qa):.6f}, with = {np.linalg.norm(qb):.6f}")
exact_yaw = (3.0 * T) % (2 * math.pi)
print(f"yaw reached {q_to_euler(qb)[2] % (2 * math.pi):.6f} rad, exact {exact_yaw:.6f} rad")
'''),
md(r'''
## 4. The 90° singularity of Euler angles (gimbal lock)

**Theory.** Euler angles (yaw $\psi$, pitch $\theta$, roll $\phi$, ZYX order) evolve as

$$\dot\phi = p + (q\sin\phi + r\cos\phi)\tan\theta,\qquad \dot\theta = q\cos\phi - r\sin\phi,\qquad \dot\psi = \frac{q\sin\phi + r\cos\phi}{\cos\theta},$$

which divide by $\cos\theta$ and blow up at $\theta = \pm 90^\circ$.
'''),
note("what goes wrong at 90°", r'''
Euler angles are three turns in a row: yaw about the vertical, pitch about the new wing axis, roll about the new nose axis. Pitch the nose straight up and the nose (roll axis) lines up with the vertical (yaw axis): rolling and yawing become the same motion, and one degree of freedom vanishes. Near it, a small body rate needs a huge Euler-angle rate, and errors are multiplied by $1/\cos\theta$. A quaternion has no such point.'''),
code(r'''
th = np.linspace(0, 89.5, 400)
plt.plot(th, 1 / np.cos(np.radians(th)))
for t in (24, 60, 85, 89):
    plt.annotate(f"{t} deg: x{1 / math.cos(math.radians(t)):.0f}", (t, 1 / math.cos(math.radians(t))),
                 textcoords="offset points", xytext=(-70, 8))
plt.yscale("log"); plt.xlabel("pitch (deg)"); plt.ylabel("yaw-rate gain 1/cos(theta)")
plt.title("How much a 1 deg/s body yaw rate is amplified in the Euler yaw rate"); plt.show()
'''),
code(r'''
# Climb the nose through 90 deg while yawing slowly: Euler-angle integration vs the quaternion
def euler_rates(phi, theta, p, q, r):
    return (p + (q * math.sin(phi) + r * math.cos(phi)) * math.tan(theta),
            q * math.cos(phi) - r * math.sin(phi),
            (q * math.sin(phi) + r * math.cos(phi)) / math.cos(theta))

dt = 1e-3; rates = (0.0, math.radians(30), math.radians(5))        # pitch up at 30 deg/s, small yaw, for 4 s
phi = theta = psi = 0.0; qq = np.array([1.0, 0, 0, 0]); peak = 0.0
for k in range(int(4.0 / dt)):
    dphi, dth, dpsi = euler_rates(phi, theta, *rates)
    peak = max(peak, abs(dphi), abs(dpsi))
    phi, theta, psi = phi + dphi * dt, theta + dth * dt, psi + dpsi * dt
    qq = q_normalize(qq + q_deriv(qq, np.array(rates)) * dt)
nose_euler = q_to_rotmat(q_from_euler(phi, theta, psi)) @ [1, 0, 0]
nose_quat = q_to_rotmat(qq) @ [1, 0, 0]
err = math.degrees(math.acos(max(-1.0, min(1.0, float(nose_euler @ nose_quat)))))
print(f"largest Euler-angle rate needed: {math.degrees(peak):.0f} deg/s for body rates of 30 and 5 deg/s")
print("nose after 4 s, Euler integration:", np.round(nose_euler, 3))
print("nose after 4 s, quaternion       :", np.round(nose_quat, 3), f"  -> Euler path is off by {err:.1f} deg")
'''),
md(r'''
## 5. Hover-frame angles from the quaternion

**Theory.** PX4's attitude is the structural attitude followed by the fixed hover rotation: $R^N_{\text{PX4}} = R(\mathbf q)\,R_h^{\mathsf T}$. In a perfect hover this is the identity, so PX4 reads roll = pitch = 0. The simulator's `RigidBody.hover_frame_euler` does this conversion for the tuning metrics.
'''),
code(r'''
q_hover = q_from_euler(0, math.radians(af.hover_pitch_deg), 0)
R_px4 = q_to_rotmat(q_hover) @ af.hover_rotation().T
print("R(q) R_h^T in a perfect hover =\n", np.round(R_px4, 6))
'''),
])

# =====================================================================================================================
save("03_equations_of_motion.ipynb", [
md(r'''
# 03 · State vector and equations of motion

How the simulator moves the aircraft: what it remembers from one step to the next (the state), the equations that change it, the inertia that resists rotation, the fan spool lag, and the integrator that steps it all forward 1000 times a second.
'''),
code(SETUP),
code(r'''
from airframe_designer.dynamics.rigid_body import RigidBody
from airframe_designer.dynamics.quaternion import q_to_euler
rb = RigidBody(af)
'''),
md(r'''
## 1. The state vector

**Theory.** For V3 the state is

$$\mathbf x = \big[\ \mathbf p^N\ (3)\ \ \mathbf v^N\ (3)\ \ \mathbf q\ (4)\ \ \boldsymbol\omega^B\ (3)\ \ \bar\Omega_1\dots\bar\Omega_9\ (9)\ \big],\qquad 22\ \text{numbers},\ 21\ \text{degrees of freedom}.$$
'''),
note("state versus everything else", r'''
A state is a quantity that needs its own history: next step's velocity is this step's velocity plus the acceleration. Everything else (leg forces, the accelerometer reading, total thrust) is recomputed from the state every step. The quaternion has 4 numbers but only 3 degrees of freedom, because of the unit-length constraint.'''),
code(r'''
state = {"p (NED, m)": rb.pos, "v (NED, m/s)": rb.vel, "q (body->NED)": rb.q, "omega (body, rad/s)": rb.rates,
         "fan speeds (0..1)": rb.omega}
n = 0
for k, v in state.items():
    n += len(v); print(f"{k:22s} {len(v):2d}  {np.round(v, 4)}")
print("total state length:", n)
print("parked pitch from q:", round(math.degrees(q_to_euler(rb.q)[1]), 2), "deg;  CG height:", round(-rb.pos[2], 3), "m")
'''),
md(r'''
## 2. Equations of motion

**Theory.**

$$\begin{aligned}
\dot{\mathbf p}^N &= \mathbf v^N\\
m\,\dot{\mathbf v}^N &= R(\mathbf q)\,\mathbf F^B + m g\,\hat{\mathbf e}_3 + \mathbf F^N_{\text{contact}}\\
\dot{\mathbf q} &= \tfrac12\,\mathbf q\otimes[0,\ \boldsymbol\omega^B]\\
I\,\dot{\boldsymbol\omega}^B &= \mathbf M^B - \boldsymbol\omega^B\times I\boldsymbol\omega^B\\
\dot{\bar\Omega}_i &= (u_i - \bar\Omega_i)/\tau_i
\end{aligned}$$

with $\hat{\mathbf e}_3 = (0, 0, 1)$ pointing down. The fourth line is Euler's equation for a rigid body.
'''),
note("\"translation is written in NED, rotation in the body frame\"", r'''
Writing an equation in a frame means choosing which axes list the components of its arrows. **Translation** (how the CG moves) is written in NED, because there Newton's law has its plain form, gravity is the same everywhere, and the floor is simply z = 0. **Rotation** (how the body spins) is written in body axes, because every part is bolted to the airframe, so the inertia tensor is a fixed set of numbers there; in NED it would change every time the aircraft turns, $I^N = R\,I^B R^{\mathsf T}$. The price is the extra $\boldsymbol\omega\times I\boldsymbol\omega$ term. The two halves are joined by $R(\mathbf q)$: forces are computed in body axes, where the fans are, and turned into NED.'''),
md(r'''
## 3. The inertia tensor and the parallel-axis theorem

**Theory.** For each part $k$ with own inertia $I_k$ about its own centre, mass $m_k$ and position $\mathbf r_k$ from the CG,

$$I = \sum_k \Big(I_k + m_k\big(\lVert\mathbf r_k\rVert^2 E - \mathbf r_k\mathbf r_k^{\mathsf T}\big)\Big),\qquad \text{scalar form: } I = I_{\text{own}} + m d^2 .$$

The off-diagonal entries are the products of inertia, $I_{xz} = \sum m_k x_k z_k$ and so on (with the minus sign convention in the tensor).
'''),
code(r'''
I = af.mass.tensor()
print("V3 inertia tensor about the CG (kg m^2), structural axes:\n", I)
vals, vecs = np.linalg.eigh(I)
print("principal moments:", np.round(vals, 4))
print("principal axes (columns):\n", np.round(vecs, 3))
'''),
note("parallel-axis theorem with one battery", r'''
Inertia depends on how far mass sits from the axis, squared. Each part has a small inertia about its own centre; moving it a distance $d$ away adds $m d^2$. The next cell does this for one of V3's six batteries (0.77 kg, about 15 × 5 cm) placed 0.25 m ahead of the CG: its own pitch inertia is tiny, the parallel-axis part is about 30 times bigger, and that single battery adds about 5 % to the whole aircraft's pitch inertia. That is why moving batteries changes how quickly ATLAS can pitch and yaw.'''),
code(r'''
m_b, Lx, Lz, d = 0.77, 0.15, 0.05, 0.25
own = m_b * (Lx**2 + Lz**2) / 12
shift = m_b * d**2
print(f"own pitch inertia {own:.4f}, parallel-axis term {shift:.4f} ({shift / own:.0f}x), "
      f"share of the aircraft's Iyy {100 * (own + shift) / I[1, 1]:.1f} %")

# The same with the code's own routine, for two point masses
from airframe_designer.geometry.mass import MassProperties, MassItem
mp = MassProperties(from_items=True, items=[MassItem("battery", m_b, [d, 0, 0], [m_b * Lz**2 / 12, own, m_b * Lx**2 / 12]),
                                             MassItem("rest", 11.0, [-d * m_b / 11.0, 0, 0])]).resolve()
print("MassProperties.resolve: cg", mp.cg, " inertia", mp.inertia)
'''),
md(r'''
## 4. Euler's equation in action: the intermediate-axis instability

**Theory.** With no moments, $I\dot{\boldsymbol\omega} = -\boldsymbol\omega\times I\boldsymbol\omega$. Spin about the axis with the smallest or largest principal moment is stable; spin about the middle one is not. This is the gyroscopic term doing its work.
'''),
note("why this matters for a drone", r'''
The $\boldsymbol\omega\times I\boldsymbol\omega$ term couples the axes: spinning about one axis creates angular acceleration about the others. At ATLAS's normal rates it is small, but it is why roll, pitch and yaw are not fully independent, and why the products of inertia (here $I_{xz}$) matter for the mixer and the gains. The cell spins the V3 inertia about its middle principal axis with a tiny disturbance.'''),
code(r'''
Ip = np.diag(vals); Ip_inv = np.linalg.inv(Ip)
dt, T = 1e-3, 20.0
for axis, label in ((0, "smallest"), (1, "middle"), (2, "largest")):
    w = np.zeros(3); w[axis] = 2.0; w += 1e-3       # 2 rad/s plus a small disturbance
    hist = []
    for _ in range(int(T / dt)):
        w = w + Ip_inv @ (-np.cross(w, Ip @ w)) * dt
        hist.append(w.copy())
    hist = np.array(hist)
    plt.plot(np.arange(len(hist)) * dt, hist[:, axis], label=f"spin about {label} axis")
plt.xlabel("time (s)"); plt.ylabel("rate about the spin axis (rad/s)"); plt.legend(); plt.title("Torque-free spin, V3 principal inertias")
plt.show()
'''),
md(r'''
## 5. Fan spool lag: first order with $\tau = 0.12$ s

**Theory.** $\dot{\bar\Omega} = (u - \bar\Omega)/\tau$, whose step response is $\bar\Omega(t) = u + (\bar\Omega_0 - u)\,e^{-t/\tau}$. Thrust is $T = T_{\text{eff}}\,\bar\Omega^2$. `RotorSet.spool` implements the first line with separate spin-up and spin-down constants.
'''),
note("what spool and lag mean", r'''
"Spool" is jet-engine language for the fan's rotational speed. A fan cannot change speed instantly, because its rotor has inertia and the motor's current is limited. The first-order model says: the speed moves towards the command at a rate proportional to the remaining gap. After one time constant τ it has done 63 % of the change, after three 95 %. Because thrust goes with speed squared, thrust lags even more at first. The 0.12 s value is a default for ducted fans and has not been measured on the ATLAS fans.'''),
code(r'''
rs = rb.rotors
print("tau (s):", rs.tau, " tau_down (s):", rs.tau_down)
dt, T = 1e-3, 1.0
omega = np.zeros(rs.n); cmd = np.full(rs.n, 1.0); t = np.arange(0, T, dt); speed, thrust = [], []
for _ in t:
    omega = rs.spool(omega, cmd, dt)
    speed.append(omega[0]); thrust.append(rs.thrust(omega)[0] / rs.tmax[0])
tau = rs.tau[0]
plt.plot(t, speed, label="fan speed (simulator)")
plt.plot(t, 1 - np.exp(-t / tau), "k--", lw=1, label="1 - exp(-t/tau)")
plt.plot(t, thrust, label="thrust / max")
for k in (1, 3):
    plt.axvline(k * tau, color="gray", lw=0.8); plt.text(k * tau + 0.01, 0.1, f"{k} tau")
plt.xlabel("time after a 0 -> 100 % step (s)"); plt.ylabel("fraction"); plt.legend(); plt.show()
f = 2.0
print(f"phase lag of the fans for a {f} Hz wobble: {math.degrees(math.atan(2 * math.pi * f * tau)):.0f} deg")
'''),
md(r'''
## 6. The integrator: semi-implicit Euler at 1 ms

**Theory.**

$$\mathbf v_{k+1} = \mathbf v_k + \mathbf a_k\Delta t,\quad \mathbf p_{k+1} = \mathbf p_k + \mathbf v_{k+1}\Delta t,\quad \boldsymbol\omega_{k+1} = \boldsymbol\omega_k + \dot{\boldsymbol\omega}_k\Delta t,\quad \mathbf q_{k+1} = \mathcal N\big(\mathbf q_k + \dot{\mathbf q}(\mathbf q_k, \boldsymbol\omega_{k+1})\Delta t\big).$$

Using the **new** velocity for the position (and the new rate for the attitude) is what makes it semi-implicit. For an oscillator it is stable while $\omega_n\Delta t < 2$.
'''),
note("why the order of the updates matters", r'''
Take one leg as a spring: $k = 1500$ N/m on about 3.9 kg gives a natural frequency near 20 rad/s. With plain explicit Euler (old velocity for the position) every 1 ms step multiplies the bounce amplitude by $\sqrt{1 + (\omega_n\Delta t)^2}$, a tiny 0.02 %, which over one second of 1000 steps adds about 20 % of bounce out of nothing. The semi-implicit order keeps it bounded. The cell compares the two.'''),
code(r'''
wn, dt, T = math.sqrt(1500 / (af.mass.mass / 3)), 1e-3, 5.0
def run(semi):
    x, v, out = 0.02, 0.0, []
    for _ in range(int(T / dt)):
        a = -wn**2 * x
        if semi:
            v = v + a * dt; x = x + v * dt
        else:
            x, v = x + v * dt, v + a * dt
        out.append(x)
    return np.array(out)
t = np.arange(int(T / dt)) * dt
plt.plot(t, run(False) * 100, label="explicit Euler")
plt.plot(t, run(True) * 100, label="semi-implicit Euler (simulator)")
plt.xlabel("time (s)"); plt.ylabel("leg deflection (cm)"); plt.legend(); plt.title(f"Undamped leg spring, {wn:.1f} rad/s, 1 ms steps"); plt.show()
'''),
md(r'''
## 7. The real body settling on its legs

**Theory.** Put together, `RigidBody.step(dt)` evaluates the forces, applies Newton and Euler, spools the fans and integrates. With the motors off the aircraft should settle on its legs, a little below the angle the legs were solved for (the springs compress under the weight).
'''),
code(r'''
rb = RigidBody(af)
rb.set_motor_commands(np.zeros(rb.rotors.n))
t, pitch, height = [], [], []
for k in range(3000):
    rb.step(1e-3, detail=False)
    if k % 10 == 0:
        t.append(rb.t); pitch.append(math.degrees(q_to_euler(rb.q)[1])); height.append(-rb.pos[2])
fig, ax = plt.subplots(1, 2, figsize=(10, 3.2))
ax[0].plot(t, pitch); ax[0].set_ylabel("pitch (deg)"); ax[0].set_xlabel("time (s)")
ax[1].plot(t, height); ax[1].set_ylabel("CG height (m)"); ax[1].set_xlabel("time (s)")
plt.suptitle(f"Motors off: rests at {pitch[-1]:.2f} deg (legs solved for {af.landed_pitch_deg} deg)"); plt.tight_layout(); plt.show()
'''),
])

# =====================================================================================================================
save("04_ground_contact.ipynb", [
md(r'''
# 04 · Ground contact: legs as algebraic springs

How the floor holds ATLAS up: each foot is a penalty spring-damper with friction, worked out fresh every step. This notebook shows the force law, why it is algebraic, and how the parked angle is set.
'''),
code(SETUP),
code(r'''
from airframe_designer.dynamics.rigid_body import RigidBody
from airframe_designer.dynamics.quaternion import q_to_rotmat, q_to_euler
rb = RigidBody(af)
legs = rb.legs
for l in af.active_legs():
    print(f"{l.name:12s} attach {np.round(l.attach, 3)}  foot {np.round(l.foot(), 3)}  k {l.stiffness:.0f} N/m  c {l.damping:.0f} N s/m  mu {l.friction}")
print("static check:", af.leg_static())
'''),
md(r'''
## 1. The contact force law

**Theory.** For a foot with penetration $\delta$ into the floor (positive when below $z = 0$) and penetration rate $\dot\delta$:

$$f_n = \max\big(k\,\delta + c\,\max(\dot\delta, 0),\ 0\big),\qquad \mathbf f_t = -\mu\,f_n\,\frac{\mathbf v_h}{\max(\lVert\mathbf v_h\rVert,\ 0.2\ \text{m/s})}.$$

The floor can only push (the max with 0), damping acts only in compression, and friction is Coulomb above 0.2 m/s and viscous below. The moment about the CG is $\mathbf r\times\mathbf f$ for each foot.
'''),
note("leg contact is algebraic", r'''
"Algebraic" means the leg force is just a formula of the current state, with no memory. Each step the code works out where each foot is from the position and attitude, checks whether it is below the floor and by how much, and sets the force from that. So the legs behave like massless springs that respond instantly, no compression is stored between steps, and friction has no "stuck" memory (a parked aircraft can creep very slowly instead of sticking). A multibody engine with each leg as its own body would turn the legs into states.'''),
code(r'''
# Push the parked body down step by step and read the algebraic force each time
R = q_to_rotmat(rb.q)
pos0 = rb.pos.copy()
sink = np.linspace(-0.005, 0.04, 60)
fz, feet = [], []
for s in sink:
    F, M, on_ground, n = legs.forces(pos0 + [0, 0, s], np.zeros(3), R, np.zeros(3))
    fz.append(-F[2]); feet.append(n)
plt.plot(sink * 100, fz, label="total floor force (N, up)")
plt.axhline(af.mass.mass * G, color="k", lw=0.8, ls="--", label="weight")
plt.xlabel("extra sink below the rest position (cm)"); plt.ylabel("N"); plt.legend(); plt.title("Leg force is a function of position only"); plt.show()
print("feet in contact across the sweep:", sorted(set(feet)))
'''),
note("spring constant, sink and damping", r'''
11.8 kg on 3 feet means each foot carries about 38.7 N. With this airframe's $k = 1500$ N/m per leg that is about 2.6 cm of static sink, and $c = 80$ N s/m gives a damping ratio of about 0.5 (the `static check` line above). `Airframe.auto_leg_constants` can resize them for a chosen sink and damping ratio (by default 2 cm and 0.8).'''),
md(r'''
## 2. Why the damping is clamped

**Theory.** For a foot at lever arm $\mathbf r$ with contact normal $\mathbf n$, the effective mass the foot "feels" is $m_{\text{eff}} = 1/\big(1/m + (\mathbf r\times\mathbf n)^{\mathsf T} I^{-1}(\mathbf r\times\mathbf n)\big)$. A damper $c$ integrated explicitly over one step is stable only while $c\,\Delta t / m_{\text{eff}}$ stays small, so the code limits the damping force to what can remove at most half of the foot's velocity in one step ($c \le 0.5\,m_{\text{eff}}/\Delta t$).
'''),
code(r'''
m_eff = legs.effective_mass(af.mass.mass, np.linalg.inv(af.mass.tensor()), R)
print("effective mass per foot (kg):", np.round(m_eff, 2))
print("damping limit at 1 ms (N s/m):", np.round(0.5 * m_eff / 1e-3, 0), " actual:", [round(l.damping) for l in af.active_legs()])
'''),
md(r'''
## 3. Setting the parked angle

**Theory.** `Airframe.with_attitude(park_pitch_deg=...)` re-solves the legs under the same hard points so that every foot lies on the floor at the requested pitch, keeping the mean leg length. If that would make a leg shorter than 3 cm (steep nose-down parks), it lowers the floor plane instead so the shortest leg is 3 cm and the others grow. It also checks the stand does not tip (the CG must project inside the feet).
'''),
note("why the aircraft rests below the solved angle", r'''
The legs are solved for a geometric angle, but under 11.8 kg each spring compresses about 2 cm, and the front feet compress a little differently from the rear ones. The aircraft therefore settles about 1.85° further nose-down. To rest at −16°, the legs are solved for −14.15°.'''),
code(r'''
for park in (-6.2, -14.15, -15.2):
    a2 = af.with_attitude(park_pitch_deg=park)
    b = RigidBody(a2); b.set_motor_commands(np.zeros(b.rotors.n))
    for _ in range(4000):
        b.step(1e-3, detail=False)
    print(f"legs solved for {park:6.2f} deg -> rests at {math.degrees(q_to_euler(b.q)[1]):6.2f} deg")
'''),
])

# =====================================================================================================================
save("05_sensors.ipynb", [
md(r'''
# 05 · Sensors: from the true state to what PX4 sees

PX4 never sees the simulator's true state, only sensor readings made from it. This notebook builds those readings: the accelerometer's specific force, the gyro, noise, and the H-FLOW optical-flow and range sensor, including the two bugs fixed on 1 Oct.
'''),
code(SETUP),
code(r'''
from airframe_designer.dynamics.rigid_body import RigidBody
from airframe_designer.dynamics.quaternion import q_to_rotmat, q_from_euler
from airframe_designer.sensors.models import SensorSuite
rb = RigidBody(af); rb.set_motor_commands(np.zeros(rb.rotors.n))
for _ in range(3000):
    rb.step(1e-3, detail=False)
'''),
md(r'''
## 1. The accelerometer measures specific force

**Theory.**

$$\mathbf f^B = R(\mathbf q)^{\mathsf T}\Big(\frac{\mathbf v_{k+1}-\mathbf v_k}{\Delta t} - g\,\hat{\mathbf e}_3\Big).$$

The simulator derives it from the motion it actually integrated, so the reading is exactly consistent with the trajectory.
'''),
note("what an accelerometer feels", r'''
An accelerometer cannot feel gravity. It feels everything that stops it from falling. Sitting on the floor it reads "pushed up by 1 g", $(0, 0, -g)$ in level axes; in free fall it reads zero. Because the reading is in the sensor's own axes, a tilted aircraft at rest shows part of that 1 g on its x axis.'''),
code(r'''
R = q_to_rotmat(rb.q)
print("simulator accel_body at rest      :", rb.accel_body)
print("R^T (0, 0, -g), the expected value :", R.T @ [0, 0, -G])
q_h = q_from_euler(0, math.radians(af.hover_pitch_deg), 0)
print("in a perfect hover (structural axes):", q_to_rotmat(q_h).T @ [0, 0, -G], " (x feels part of the thrust)")
'''),
md(r'''
## 2. Gyro and the full HIL_SENSOR message

**Theory.** $\boldsymbol\omega_{\text{gyro}} = \boldsymbol\omega^B + \mathbf b_g + \mathbf n_g + \boldsymbol\omega_{\text{vib}}$: the true rate plus a constant bias, white noise, and the fan vibration model. The magnetometer is $R^{\mathsf T}\mathbf m^N$ plus noise; the barometer is ISA pressure at the current altitude plus noise.
'''),
code(r'''
ss = SensorSuite(seed=1); ss.set_airframe(af)
acc, gyr = [], []
for k in range(2000):
    m = ss.hil_sensor(rb, k * 4000)
    acc.append([m["xacc"], m["yacc"], m["zacc"]]); gyr.append([m["xgyro"], m["ygyro"], m["zgyro"]])
acc, gyr = np.array(acc), np.array(gyr)
print("accel mean", acc.mean(0), " std", acc.std(0))
print("gyro  mean", gyr.mean(0), " std", gyr.std(0))
print("one message:", {k: (round(v, 3) if isinstance(v, float) else v) for k, v in m.items()})
'''),
note("truth versus estimate", r'''
PX4's estimator (EKF2) builds its own state from these readings: attitude, velocity, position, gyro and accelerometer biases, magnetic fields and wind, about 24 numbers. Almost every HITL problem on 30 Sep was a gap between that estimate and the simulator's truth (a stored accelerometer offset, noisy flow), not a problem with the true motion. Comparing estimate with truth is the most useful diagnostic the simulator offers.'''),
md(r'''
## 3. The H-FLOW: optical flow and range

**Theory.** A downward camera at height $d$ (along its optical axis) over a flat floor sees the image centre move by

$$\text{pixel}_x = \Delta\theta_x - \frac{v_y\,\Delta t}{d},\qquad \text{pixel}_y = \Delta\theta_y + \frac{v_x\,\Delta t}{d},$$

with $\mathbf v$ the lens velocity and $\Delta\boldsymbol\theta$ the rotation over the window, both in the sensor frame. PX4 subtracts the rotation using a gyro and recovers $v = d\cdot(\text{flow} - \text{gyro})/\Delta t$.

The as-built mount: fixed to the frame, tilted 20.4°, right below the Pixhawk, so in the 23.85° hover it looks 3.45° off vertical.
'''),
code(r'''
from airframe_designer.sensors.flow import FlowSensor, ekf2_params
cfg = dict(af.design["flow_sensor"]); cfg["enabled"] = True
print("flow sensor config:", {k: cfg[k] for k in ("pos", "mount", "extra_pitch_deg", "rate", "flow_noise", "range_noise")})
print("EKF2 settings that match it:")
for k, v in ekf2_params(af, cfg).items():
    print(f"   {k:18s} {v}")
'''),
note("the two flow bugs fixed on 1 Oct", r'''
1. **Noise 7 times too large.** `flow_noise` is the noise of one reading (0.02 rad/s). The code used to scale it by $\sqrt{\Delta t}$ as if it were a noise density, which at 50 readings a second made each one 0.14 rad/s: about 0.37 m/s of fake motion at 2.6 m. EKF2 kept nudging its heading to explain that noise. Now it is `noise * dt`.
2. **Gyro in the wrong frame.** The message now carries the sensor's own gyro integral, so PX4 does not fall back to the raw, unrotated board gyro (notebook 01, section 5).

The cell measures both: the decoded noise per reading, and a pure turn on the tilted mount. The lens sits about 8 cm from the CG, so a turn really moves it ($\boldsymbol\omega\times\mathbf r$); the decoded velocity must equal exactly that lens motion, with no rotation leaking in.'''),
code(r'''
from types import SimpleNamespace
fs = FlowSensor(af, cfg, np.random.default_rng(3))
def fake_sim(vel_ned, rates, pitch_deg, height):
    Rm = q_to_rotmat(q_from_euler(0, math.radians(pitch_deg), 0))
    return SimpleNamespace(rotmat=Rm, rates=np.asarray(rates, float), vel=np.asarray(vel_ned, float), pos=np.array([0, 0, -height]))
dt = 1 / fs.rate
rates = []
for _ in range(3000):
    s = fake_sim([0, 0, 0], [0, 0, 0], af.hover_pitch_deg, 1.5)
    fs.step(s, dt); of, ds = fs.messages(s, 0, dt)
    rates.append(of["integrated_x"] / dt)
print(f"flow noise per reading: {np.std(rates):.4f} rad/s (configured {cfg['flow_noise']})")

fs2 = FlowSensor(af, dict(cfg, flow_noise=0, range_noise=0), np.random.default_rng(1))
s = fake_sim([0, 0, 0], [0.2, -0.1, 0.5], af.hover_pitch_deg, 1.5)
fs2.step(s, dt); of, ds = fs2.messages(s, 0, dt)
flow = -np.array([of["integrated_x"], of["integrated_y"]]); gyro = -np.array([of["integrated_xgyro"], of["integrated_ygyro"]])
comp = flow - gyro; d = of["distance"]
decoded = np.array([-d * comp[1] / dt, d * comp[0] / dt])
lens = (fs2.C.T @ np.cross([0.2, -0.1, 0.5], fs2.r))[:2]          # the lens's own motion from the lever arm, sensor axes
print("pure turn, decoded lens velocity :", np.round(decoded, 5), "m/s   range", round(d, 3), "m")
print("lens motion from omega x r       :", np.round(lens, 5), "m/s   match:", np.allclose(decoded, lens, atol=1e-6))
'''),
])

# =====================================================================================================================
save("06_fans_jetfoils_coanda.ipynb", [
md(r'''
# 06 · Fans, jetfoils, Coanda flow and jet enclosure aerodynamics

Where ATLAS's lift comes from: nine ducted fans whose jets are turned downward by curved jetfoils. This notebook covers the momentum theory of a fan in a duct, how the simulator models the turned jet and its losses, ram drag, what Coanda flow is and what the model leaves out, and how all of it reaches PX4's mixer.
'''),
code(SETUP),
code(r'''
from airframe_designer.aero.rotor_aero import RotorSet
rs = RotorSet(af.active_rotors(), af.cg)
print(f"{'fan':10s} {'kind':7s} {'D (mm)':>6s} {'Tmax':>5s} {'exit':>6s} {'T_eff':>6s} {'km':>6s} {'tau':>5s} ram")
for r in af.active_rotors():
    print(f"{r.name[:10]:10s} {r.kind:7s} {r.diameter * 1000:6.0f} {r.max_thrust:5.1f} {r.deflection_deg():6.1f} {r.effective_max_thrust():6.2f} {r.km:6.3f} {r.tau:5.2f} {r.ram_drag}")
'''),
md(r'''
## 1. Jet enclosure: momentum theory for a ducted fan

**Theory.** Treat the fan as an actuator disc of area $A$ adding momentum to air drawn from rest. Thrust is the momentum flux leaving the system, $T = \dot m V_e$ with $\dot m = \rho A_e V_e$. With exit area ratio $\sigma = A_e/A$:

$$T = \rho\,\sigma A\,V_e^2,\qquad P_{\text{ideal}} = \tfrac12\dot m V_e^2 = \frac{T^{3/2}}{2\sqrt{\rho\,\sigma A}},\qquad \text{open rotor: } P_{\text{ideal}} = \frac{T^{3/2}}{\sqrt{2\rho A}}.$$
'''),
note("what the duct buys", r'''
An open propeller's wake contracts to half the disc area, so the air leaves faster than it needs to and wastes energy. A duct with an exit as big as the disc ($\sigma = 1$) stops that contraction: for the same thrust it needs about 71 % of the power, or for the same power it gives about 26 % more thrust. Part of that extra thrust is suction on the duct lip. In the simulator thrust is specified per fan, so this formula is only used as a power diagnostic.'''),
code(r'''
rho = 1.225
r0 = af.active_rotors()[0]; A = r0.disc_area
T = np.linspace(1, r0.max_thrust, 50)
P_open = T**1.5 / np.sqrt(2 * rho * A); P_duct = T**1.5 / (2 * np.sqrt(rho * A))
plt.plot(T, P_open, label="open rotor"); plt.plot(T, P_duct, label="ducted, sigma = 1")
plt.xlabel("thrust per fan (N)"); plt.ylabel("ideal power (W)"); plt.legend(); plt.title(f"{r0.diameter * 1000:.0f} mm fan, momentum theory"); plt.show()
T_h = af.mass.mass * G / len(af.active_rotors())
print(f"at ~{T_h:.1f} N per fan: open {T_h**1.5 / np.sqrt(2 * rho * A):.0f} W, ducted {T_h**1.5 / (2 * np.sqrt(rho * A)):.0f} W (ideal); exit speed {math.sqrt(T_h / (rho * A)):.0f} m/s")
'''),
md(r'''
## 2. The turned jet: direction, loss and point of action

**Theory.** The jetfoil turns the jet by its exit angle $\delta$. The simulator models:

$$\hat{\mathbf a} = (\cos\delta,\ 0,\ -\sin\delta),\qquad \text{lean} = 90^\circ - \delta,\qquad T_{\text{eff}} = T_{\max}\,\max\Big(0,\ 1 - k_{\text{turn}}\frac{\delta}{90^\circ}\Big),\ \ k_{\text{turn}} = 0.1,$$

acting at the jet exit line $\mathbf r = \mathbf r_{\text{TE}} + h\hat{\mathbf n}$ with $h = 15$ mm. The exit angle comes from the CAD: rays cast through the foil every 2.5 mm, slope of the last 5 mm of wall.
'''),
note("why one force at the exit is enough", r'''
Draw a box around the fan, duct and foil. Air enters from rest and leaves as a jet along the trailing edge. By momentum balance, the total force on everything inside the box equals the jet's momentum flux, pointing opposite to where the jet leaves. So the fan's own thrust and the foil's reaction never need to be modelled separately: only their sum, the turned jet, acting where the jet leaves.'''),
code(r'''
d = np.linspace(0, 90, 91)
plt.plot(d, 1 - 0.1 * d / 90)
for r in af.active_rotors():
    plt.plot(r.deflection_deg(), r.effective_max_thrust() / r.max_thrust, "o", color="C1")
plt.xlabel("jetfoil exit angle delta (deg)"); plt.ylabel("T_eff / T_max"); plt.title("Linear turning loss, k_turn = 0.1 (assumed)"); plt.show()
for r in af.active_rotors()[:3]:
    dl = r.deflection_deg()
    print(f"{r.name[:10]:10s} delta {dl:5.1f} deg  lean {90 - dl:5.1f} deg  axis {np.round(np.asarray(r.axis) / np.linalg.norm(r.axis), 3)}  loss {100 * (1 - r.effective_max_thrust() / r.max_thrust):.1f} %")
'''),
md(r'''
## 3. Coanda flow: how a curved wall turns a jet

**Theory.** For streamlines to curve around a wall of radius $R$, the pressure must drop towards the wall. A radial momentum balance across a jet of thickness $b$ and speed $U$ gives

$$\frac{\partial p}{\partial n} = \frac{\rho U^2}{R}\quad\Rightarrow\quad \Delta p \approx \frac{\rho U^2 b}{R}.$$

The jet stays attached while that suction survives wall friction and mixing with the surrounding air. Thin, fast jets on gentle curves (large $R/b$) turn further before they separate.
'''),
note("what the simulator assumes about Coanda flow", r'''
The simulator assumes the jet stays fully attached and leaves exactly along the trailing edge, at every throttle and airspeed. If the real jet separates early, it leaves at a smaller angle than the wall: more forward thrust, less lift, a different hover pitch. Separation, jet spreading, entrainment, jet-jet interaction and ground effect are not modelled. The numbers in the next cell are an order-of-magnitude illustration with an assumed wall radius, not a measurement.'''),
code(r'''
U = math.sqrt(T_h / (rho * A))          # jet speed from momentum theory at hover thrust
b = 2 * 0.015                            # jet thickness ~ nozzle height (twice the 15 mm half-height used in the model)
for R_wall in (0.05, 0.10, 0.20):        # assumed wall radii
    print(f"R = {R_wall:.2f} m, R/b = {R_wall / b:4.1f}: suction at the wall ~ {rho * U**2 * b / R_wall:5.0f} Pa  (jet speed {U:.0f} m/s)")
'''),
md(r'''
## 4. Ram drag: the enclosure in a crossflow

**Theory.** Air arriving at the fan with velocity $\mathbf V$ must be turned into the fan's own jet, which costs momentum flux $\dot m\mathbf V$:

$$\mathbf f_{\text{ram},i} = -\dot m_i\mathbf V_i,\qquad \dot m_i = \sqrt{\rho A_i T_i},\qquad \mathbf V_i = \mathbf v_{\text{air}} + \boldsymbol\omega\times\mathbf r_i .$$
'''),
note("why ram drag also slows yaw", r'''
Ram drag grows with the air speed seen by each fan, including the part caused by the aircraft's own rotation ($\boldsymbol\omega\times\mathbf r$). So it resists translation and rotation alike. At 2 m/s it is about 0.7 N per fan, roughly 6 N in total against 116 N of weight; it also damps yaw, which is part of why ATLAS's yaw is sluggish.'''),
code(r'''
omega_h = np.full(rs.n, math.sqrt(T_h / rs.tmax.mean()))
F0, M0, _, _ = rs.forces(omega_h, np.zeros(3), np.zeros(3))
for v in (1.0, 2.0, 4.0):
    F, M, _, _ = rs.forces(omega_h, np.array([v, 0, 0]), np.zeros(3))
    print(f"forward airspeed {v:.0f} m/s: ram drag change {np.round(F - F0, 2)} N")
F, M, _, _ = rs.forces(omega_h, np.zeros(3), np.array([0, 0, 0.5]))
print("yaw rate 0.5 rad/s: yaw moment change", round((M - M0)[2], 3), "N m")
'''),
md(r'''
## 5. Into PX4: the mixer export and the hover trim

**Theory.** PX4's control allocation uses, per fan, the moment and force per unit thrust in hover axes: the effectiveness column $[\mathbf p\times\hat{\mathbf a} - k_m\hat{\mathbf a};\ \hat{\mathbf a}]$, and a relative thrust coefficient `CA_ROTORi_CT` $\propto T_{\text{eff},i}$. The hover pitch $\theta_h$ is chosen by scanning for the angle that leaves the most control headroom, and `MPC_THR_HOVER` is the throttle that balances the weight.
'''),
code(r'''
p = af.px4_params()
for i in range(3):
    print({k: p[k] for k in p if k.startswith(f"CA_ROTOR{i}_")})
print("SENS_BOARD_Y_OFF =", p["SENS_BOARD_Y_OFF"], "  hover throttle fraction =", round(af.hover_thrust_fraction(), 3))
B = af.effectiveness()
print("effectiveness matrix (rows: roll, pitch, yaw moment; x, y, z force), columns = fans:\n", np.round(B, 3))
print("trim hover pitch from the scan:", af.trim_hover_pitch(), "deg")
'''),
note("what is still an assumption", r'''
Fan maximum thrust (36 N), reaction torque coefficient $k_m = 0.002$, spool time constant (0.12 s), turning loss $k_{\text{turn}} = 0.1$ and nozzle height (15 mm) are all unmeasured. A thrust stand with a 6-axis load cell, one fan with its foil, swept in throttle, would replace all five in one setup.'''),
])

# =====================================================================================================================
save("07_jsbsim_and_chrono.ipynb", [
md(r'''
# 07 · JSBSim through Python, and where Project Chrono would fit

JSBSim is an optional second physics backend: an independent C++ engine that integrates the same aircraft, used to cross-check the Python integrator, frames and ground contact. Project Chrono is not used in this repository; the second half of this notebook explains where it would fit.
'''),
code(SETUP),
md(r'''
## 1. What JSBSim does and what Python still does

| Task | Done by |
|---|---|
| 6-DOF integration, gravity, Earth model (rotating WGS84) | JSBSim |
| Atmosphere | JSBSim (the Python backend uses constant density) |
| Leg contact (feet as `STRUCTURE` contacts) | JSBSim |
| Moment arms of the fan forces | JSBSim, from each force's location |
| Fan spool, thrust, reaction torque, ram drag, body drag | Python, injected every step |
| Wings, if any | JSBSim, from tables Python generates |
'''),
note("why a second engine is useful", r'''
If two independently written integrators fly the same aircraft with the same forces and agree, the frames, signs, contact model and integration are probably right. JSBSim does not check the aerodynamics, because both backends use the same fan and drag code.'''),
md(r'''
## 2. Generating the JSBSim aircraft file

**Theory.** `generate_model` writes an aircraft XML: mass and inertia (slugs, slug·ft²), each foot as a contact, one `external_reactions` force per fan in the BODY frame at the fan's position, and aerodynamic tables. JSBSim's structural frame is x aft, y right, z up, in inches, so every point maps as $\mathbf r_{\text{JSB}} = (-x,\ y,\ -z)\times 39.37$.
'''),
code(r'''
from airframe_designer.dynamics.jsbsim_backend import generate_model, JSB_ROOT
mdir = generate_model(af, "notebook_demo")
xml = (mdir / "notebook_demo.xml").read_text()
print("written to", mdir)
print(xml[:1800])
'''),
code(r'''
# The unit and axis conversion, by hand, for the first fan
r0 = af.active_rotors()[0]
p = np.asarray(r0.pos, float)
print("fan position, ours (FRD, m):", p, " -> JSBSim (aft, right, up, in):", np.round(np.array([-p[0], p[1], -p[2]]) * 39.37, 3))
print("mass", af.mass.mass, "kg =", round(af.mass.mass / 14.5939, 4), "slug;  Ixx", af.mass.inertia[0], "kg m^2 =",
      round(af.mass.inertia[0] / 1.35582, 4), "slug ft^2")
'''),
md(r'''
## 3. FGFDMExec: driving JSBSim from Python

**Theory.** `FGFDMExec` ("Flight Dynamics Model Executive") is JSBSim's main object. One instance is one complete simulated aircraft. It reads the XML, owns all the sub-models (mass, propulsion, aerodynamics, ground reactions, external forces, atmosphere, Earth model, integrator), runs them in order every step, and keeps a **property tree** of named values such as `velocities/p-rad_sec`, whose suffix gives the unit.
'''),
note("the four calls the backend makes", r'''
1. `fdm = jsbsim.FGFDMExec(root)` creates the executive.
2. `fdm.load_model(name)` and `fdm.set_dt(dt)` load the aircraft and set the step.
3. `fdm["ic/theta-deg"] = pitch` (and latitude, height, heading), then `fdm.run_ic()` applies the initial conditions.
4. Every step: write each fan's thrust into `external_reactions/rotor<i>/magnitude`, call `fdm.run()`, read back properties such as `velocities/v-north-fps`.

The cell runs this only if the `jsbsim` package is installed; on this Mac it is not, so it prints how to install it.'''),
code(r'''
try:
    import jsbsim
except ImportError:
    jsbsim = None
    print("jsbsim is not installed in this environment. Install it with:")
    print("    uv pip install --python .venv/bin/python jsbsim")
if jsbsim is not None:
    fdm = jsbsim.FGFDMExec(str(JSB_ROOT))
    fdm.set_debug_level(0)
    assert fdm.load_model("notebook_demo")
    fdm.set_dt(1e-3)
    fdm["ic/h-agl-ft"] = 0.5; fdm["ic/theta-deg"] = af.landed_pitch_deg
    fdm.run_ic()
    for _ in range(2000):
        fdm.run()
    print("after 2 s at rest: theta", round(fdm["attitude/theta-deg"], 2), "deg, h-agl", round(fdm["position/h-agl-ft"] * 0.3048, 3), "m")
    # the same through the simulator's backend class
    from airframe_designer.dynamics.jsbsim_backend import JSBSimBody
    jb = JSBSimBody(af); jb.set_motor_commands(np.zeros(len(af.active_rotors())))
    for _ in range(2000):
        jb.step(1e-3)
    print("JSBSimBody pitch", round(math.degrees(jb.euler[1]), 2), "deg")
'''),
md(r'''
## 4. Project Chrono: what it is and where it would fit

**Theory.** Project Chrono is an open-source multibody dynamics library (C++, Python bindings called PyChrono). It is built around many connected bodies and their contacts:

- rigid bodies with joints (revolute, prismatic, springs and dampers), solved as a constrained system;
- contact as penalty forces with Hertz or Hunt-Crossley laws (SMC), $f_n = k\delta^{n} + c\,\delta^{n}\dot\delta$, or as hard complementarity constraints (NSC);
- flexible bodies through finite elements (beams, shells);
- add-ons for vehicles, granular terrain and fluid-structure interaction.

It has no aircraft aerodynamics: the fan and drag forces would still come from this project's Python code, as with JSBSim.

| Area | Today | With Chrono |
|---|---|---|
| Ground contact | point feet, linear spring-damper, regularised friction | real foot shapes, Hunt-Crossley contact, stick-slip friction |
| Nose-lift rotation | pivots on penalty springs at the rear feet | a pivot held as a constraint, no spring bounce |
| Legs | rigid lines, re-solved per parked angle | articulated or compliant legs with their own mass |
| Structure | analytic vibration modes feeding the IMU only | flexible booms and mounts whose modes feed back into the flight |
'''),
note("is it worth adding", r'''
Only if the ground phases (park, fan-driven rotation, touchdown, nose-lowering) turn out to be the limiting factor. In the air Chrono adds nothing over the current rigid-body integrator. A practical route mirrors the JSBSim bridge: one Chrono body with the vehicle's mass and inertia, foot contact shapes, the Python `RotorSet` applying forces through Chrono's force accumulators each step, and `DoStepDynamics` at 1 ms; then fly the same scenarios on both and compare the ground phases. The sketch below shows the shape of that code; it only runs if `pychrono` is installed.'''),
code(r'''
try:
    import pychrono as chrono
except ImportError:
    chrono = None
    print("pychrono is not installed (it is distributed through conda: conda install -c projectchrono pychrono).")
if chrono is not None:
    sys_ = chrono.ChSystemSMC()
    sys_.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -G))      # Chrono is usually z-up
    body = chrono.ChBody()
    body.SetMass(af.mass.mass)
    body.SetInertiaXX(chrono.ChVector3d(*af.mass.inertia))
    sys_.Add(body)
    print("Chrono body created; next steps: foot contact shapes, force accumulators fed by RotorSet, DoStepDynamics(1e-3)")
'''),
])
print("done")
