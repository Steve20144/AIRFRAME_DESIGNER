"""Gazebo physics backend: the same airframe, integrated by gz-sim instead of this project's rigid body.

The split is the one the JSBSim backend already uses. Gazebo does what a general-purpose engine does well and this
project has to approximate: 6-DOF integration, gravity, and contact against real collision geometry. Everything
aircraft-specific stays here, because Gazebo knows nothing about ducted fans whose jets are turned by a foil:
rotor spool-up, the thrust curve, reaction torque, intake ram drag and body aerodynamics are all the project's own
models, summed into a single force and moment about the centre of gravity and handed to Gazebo each step.

So a Python-versus-Gazebo difference points at integration, contact or frames, not at the force models, which are
shared by construction. That is the whole point of having a third engine.

Frames
------
This project works in world NED with a body FRD; Gazebo works in world ENU with a model FLU. Both conversions
are their own inverse and both happen exactly once, in ``_ned_enu`` and ``_frd_flu``. A sign error in either is
silent and produces a plausible-looking flight, so they are unit-tested rather than trusted.

Lockstep
--------
The server is started paused and advanced by an explicit ``multi_step`` request, so simulated time only moves
when this loop says so. Measured at 0.2 ms per request regardless of the batch size, which is about 13x real time
at a 1 ms step, and verified against the clock rather than inferred from the reply.

UNFINISHED: reading the state back
----------------------------------
Everything here works except ``_read_state``, and it does not work for a reason worth writing down.

A gz-transport subscription from Python delivers correct data, but only while a stepping request is blocked.
Make the stepping fast and the subscriber receives nothing at all; let the stepping time out and every message
arrives. That inverse behaviour survived separate Nodes, a shared Node, and three yield strategies, so it is
neither the GIL nor the publish rate. Polling ``/world/<w>/state`` instead is fast (1.1 ms per request) but its
payload uses gz-sim's own component serialisation rather than plain protobuf, so the pose cannot be decoded
without reproducing version-specific internals.

The way out is a small Gazebo system plugin that reads the link's pose and twist in-process and publishes them
as one plain message, which is the pattern the sibling Gazebo platform already uses. Until that exists this
backend generates a correct world and can step it, but cannot fly.
"""
from __future__ import annotations

import math
import shutil
import subprocess
import tempfile
import threading
import time
from pathlib import Path

import numpy as np

from ..aero import BodyAero, RotorSet, WingSet
from ..geometry.airframe import Airframe
from .quaternion import q_from_euler, q_normalize, q_to_euler, q_to_rotmat

G = 9.80665
#: FRD <-> FLU and NED <-> ENU. Each is an involution, so one matrix serves both directions.
_FRD_FLU = np.diag([1.0, -1.0, -1.0])
_NED_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


def _frd_flu(v) -> np.ndarray:
    """Body FRD to model FLU, and back."""
    return _FRD_FLU @ np.asarray(v, float)


def _ned_enu(v) -> np.ndarray:
    """World NED to world ENU, and back."""
    return _NED_ENU @ np.asarray(v, float)


def _rot_ned_frd(R_enu_flu: np.ndarray) -> np.ndarray:
    """A body-to-world rotation expressed ENU-from-FLU, re-expressed NED-from-FRD."""
    return _NED_ENU @ R_enu_flu @ _FRD_FLU


def _inertia_flu(I_frd: np.ndarray) -> np.ndarray:
    """The inertia tensor carried into the model frame: S I S with S = diag(1, -1, -1)."""
    return _FRD_FLU @ I_frd @ _FRD_FLU


def _q_from_rotmat(R: np.ndarray) -> np.ndarray:
    """Quaternion (w, x, y, z) of a rotation matrix. Shortest-arc branch, as the rest of the project uses."""
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    if w < 1e-9:                       # 180 degrees away from identity: recover from the largest diagonal term
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        t = math.sqrt(max(1e-12, 1.0 + R[i, i] - R[j, j] - R[k, k])) / 2.0
        q = np.zeros(4)
        q[0] = (R[k, j] - R[j, k]) / (4.0 * t)
        q[1 + i], q[1 + j], q[1 + k] = t, (R[j, i] + R[i, j]) / (4.0 * t), (R[k, i] + R[i, k]) / (4.0 * t)
        return q_normalize(q)
    return q_normalize(np.array([w, (R[2, 1] - R[1, 2]) / (4 * w),
                                 (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w)]))


def _num(v: float) -> str:
    return f"{float(v):.9g}"


def generate_world(af: Airframe, name: str, out_dir: Path) -> Path:
    """Write a world holding this airframe as one rigid link, and return the file.

    The link frame is put at the centre of gravity. That makes the inertia tensor the one this project already
    carries, lets the feet be placed straight from ``Leg.foot()``, and means the single wrench applied each step
    acts exactly where the force models computed it. Nothing in the file is hand-edited; it is a pure function of
    the airframe, so regenerating it is always safe.
    """
    af.mass.resolve()
    cg = np.asarray(af.cg, float)
    I = _inertia_flu(af.mass.tensor())
    feet = [(np.asarray(l.foot(), float) - cg, max(l.foot_radius, 0.02), l.friction) for l in af.active_legs()]
    body = np.asarray(af.body.size, float)

    collisions = []
    for i, (p, r, mu) in enumerate(feet):
        q = _frd_flu(p)
        collisions.append(
            f'      <collision name="foot{i}"><pose>{_num(q[0])} {_num(q[1])} {_num(q[2])} 0 0 0</pose>'
            f'<geometry><sphere><radius>{_num(r)}</radius></sphere></geometry>'
            f'<surface><friction><ode><mu>{_num(mu)}</mu><mu2>{_num(mu)}</mu2></ode></friction></surface>'
            f"</collision>"
        )
    # A visual only, so a running world can be looked at; it carries no physics.
    visual = (f'      <visual name="hull"><geometry><box><size>{_num(body[0])} {_num(body[1])} '
              f"{_num(body[2])}</size></box></geometry></visual>")

    return _write(out_dir, name, f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{name}">
    <physics name="lockstep" type="dartsim">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <gravity>0 0 -{_num(G)}</gravity>
    <model name="ground_plane"><static>true</static><link name="link">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>2000 2000</size></plane></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
    </link></model>
    <model name="{name}">
      <pose>0 0 1 0 0 0</pose>
      <link name="base_link">
        <inertial>
          <mass>{_num(af.mass.mass)}</mass>
          <inertia>
            <ixx>{_num(I[0, 0])}</ixx><iyy>{_num(I[1, 1])}</iyy><izz>{_num(I[2, 2])}</izz>
            <ixy>{_num(I[0, 1])}</ixy><ixz>{_num(I[0, 2])}</ixz><iyz>{_num(I[1, 2])}</iyz>
          </inertia>
        </inertial>
{chr(10).join(collisions)}
{visual}
      </link>
      <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
        <odom_frame>world</odom_frame><robot_base_frame>base_link</robot_base_frame>
        <odom_publish_frequency>1000</odom_publish_frequency><dimensions>3</dimensions>
      </plugin>
    </model>
  </world>
</sdf>
""")


def _write(out_dir: Path, name: str, text: str) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{name}.sdf"
    path.write_text(text)
    return path


class GazeboBody:
    """Drop-in replacement for dynamics.RigidBody backed by a gz-sim server."""

    backend = "gazebo"

    def __init__(self, airframe: Airframe, log=None, start_timeout: float = 30.0):
        from gz.transport13 import Node          # imported here so the rest of the project runs without Gazebo

        self._Node = Node
        self.log = log or (lambda s: None)
        self.wind_ned = np.zeros(3)
        self.start_timeout = float(start_timeout)
        self._proc: subprocess.Popen | None = None
        self._dir = Path(tempfile.mkdtemp(prefix="airframe_gz_"))
        self._lock = threading.Lock()
        self._state: dict = {}
        self.set_airframe(airframe)
        self.reset()

    # ------------------------------------------------------------------ configuration
    def set_airframe(self, airframe: Airframe) -> None:
        self.af = airframe
        airframe.mass.resolve()
        self.mass = float(airframe.mass.mass)
        self.I = airframe.mass.tensor()
        cg = airframe.cg
        rotors = airframe.active_rotors()
        self.rotors = RotorSet(rotors, cg)
        self.wings = WingSet(airframe.active_wings(), cg)
        self.body = BodyAero(airframe.body, cg)
        n = len(rotors)
        if not hasattr(self, "omega") or len(self.omega) != n:
            self.omega = np.zeros(n)
            self.cmd = np.zeros(n)
            self.thrust = np.zeros(n)
        self.name = "af_" + "".join(c if c.isalnum() else "_" for c in airframe.name.lower())[:40]
        self.world = generate_world(airframe, self.name, self._dir)
        self._start_server()
        self.breakdown = {}

    def _start_server(self) -> None:
        self.close()
        self._proc = subprocess.Popen(["gz", "sim", "-s", "-v", "1", str(self.world)],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # One Node drives the clock, another listens. Sharing them lets the subscriber's callback thread starve
        # the stepping reply, so every step waits out its timeout while the stepping itself succeeds.
        self._req = self._Node()
        self._sub = self._Node()
        self._ctrl = f"/world/{self.name}/control"
        deadline = time.time() + self.start_timeout
        while time.time() < deadline:
            if self._ctrl in self._req.service_list():
                break
            if self._proc.poll() is not None:
                raise RuntimeError("gz sim exited during start-up")
            time.sleep(0.05)
        else:
            raise RuntimeError(f"gz sim did not offer {self._ctrl} within {self.start_timeout:g} s")

        from gz.msgs10.odometry_pb2 import Odometry
        self._Odometry = Odometry

        def on_odom(msg) -> None:
            p, o = msg.pose.position, msg.pose.orientation
            t = msg.twist
            with self._lock:
                self._state = {
                    "pos_enu": np.array([p.x, p.y, p.z]),
                    "quat_enu_flu": np.array([o.w, o.x, o.y, o.z]),
                    "vel_body_flu": np.array([t.linear.x, t.linear.y, t.linear.z]),
                    "rate_body_flu": np.array([t.angular.x, t.angular.y, t.angular.z]),
                }

        self._sub.subscribe(Odometry, f"/model/{self.name}/odometry", on_odom)
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_wrench_pb2 import EntityWrench
        from gz.msgs10.world_control_pb2 import WorldControl
        self._Boolean, self._EntityWrench, self._WorldControl = Boolean, EntityWrench, WorldControl
        self._wrench_pub = self._req.advertise(f"/world/{self.name}/wrench", EntityWrench)
        time.sleep(0.3)

    def close(self) -> None:
        if self._proc is not None and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        self._proc = None

    def __del__(self):
        try:
            self.close()
            shutil.rmtree(self._dir, ignore_errors=True)
        except Exception:
            pass

    # ------------------------------------------------------------------ state
    def _read_state(self) -> None:
        with self._lock:
            s = dict(self._state)
        if not s:
            return
        self.pos = _ned_enu(s["pos_enu"])
        w, x, y, z = s["quat_enu_flu"]
        R = _rot_ned_frd(q_to_rotmat(np.array([w, x, y, z])))
        self.q = _q_from_rotmat(R)
        self.rates = _frd_flu(s["rate_body_flu"])
        self.vel = R @ _frd_flu(s["vel_body_flu"])

    def _step_server(self, n: int) -> None:
        req = self._WorldControl()
        req.pause = True
        req.multi_step = int(n)
        self._req.request(self._ctrl, req, self._WorldControl, self._Boolean, 2000)

    # ------------------------------------------------------------------ the RigidBody interface
    def set_rotor_health(self, scales) -> None:
        s = np.ones(self.rotors.n)
        for i, v in enumerate(scales[: self.rotors.n]):
            s[i] = float(v)
        self.rotors.scale = s

    def set_motor_commands(self, cmd) -> None:
        self.cmd = np.clip(np.asarray(cmd, float)[: self.rotors.n], 0.0, 1.0)

    def reset(self, yaw: float = 0.0, pos_ned=None) -> None:
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.pose_pb2 import Pose
        self.t = 0.0
        pitch = math.radians(float(self.af.landed_pitch_deg))
        q = q_from_euler(0.0, pitch, yaw)
        R = q_to_rotmat(q)
        feet = np.array([l.foot() for l in self.af.active_legs()], float).reshape(-1, 3) - self.af.cg
        h = float(np.max((feet @ R.T)[:, 2])) if len(feet) else 0.0
        pos = np.asarray(pos_ned, float) if pos_ned is not None else np.array([0.0, 0.0, -(h + 0.002)])

        enu = _ned_enu(pos)
        R_enu_flu = _NED_ENU @ R @ _FRD_FLU
        qw, qx, qy, qz = _q_from_rotmat(R_enu_flu)
        msg = Pose()
        msg.name = self.name
        msg.position.x, msg.position.y, msg.position.z = (float(v) for v in enu)
        msg.orientation.w, msg.orientation.x = float(qw), float(qx)
        msg.orientation.y, msg.orientation.z = float(qy), float(qz)
        self._req.request(f"/world/{self.name}/set_pose", msg, Pose, Boolean, 2000)

        self.omega = np.zeros(self.rotors.n)
        self.cmd = np.zeros(self.rotors.n)
        self.thrust = np.zeros(self.rotors.n)
        self.pos = pos.copy()
        self.vel = np.zeros(3)
        self.q = np.asarray(q, float)
        self.rates = np.zeros(3)
        self.accel_body = np.array([0.0, 0.0, -G])
        self.breakdown = {"thrust": 0.0, "lift": 0.0, "wing_drag": 0.0, "ram_drag": 0.0,
                          "body_drag": 0.0, "power": 0.0, "airspeed": 0.0, "alpha": [], "stalled": False}
        self._step_server(1)
        self._read_state()

    def step(self, dt: float, detail: bool = True) -> None:
        R = q_to_rotmat(self.q)
        v_air = R.T @ (self.vel - self.wind_ned)
        self.omega = np.clip(self.omega + (self.cmd - self.omega) / self.rotors.tau * dt, 0.0, 1.0)
        F_r, M_r, thrust, ram = self.rotors.forces(self.omega, v_air, self.rates, detail)
        F_b, M_b, body_drag = self.body.forces(v_air, self.rates, detail)
        F_w, M_w, wb = (self.wings.forces(v_air, self.rates, detail) if self.wings.n
                        else (np.zeros(3), np.zeros(3), {"lift": 0.0, "drag": 0.0, "alpha": [], "stalled": False}))
        F = F_r + F_b + F_w
        M = M_r + M_b + M_w

        msg = self._EntityWrench()
        msg.entity.name = f"{self.name}::base_link"
        msg.entity.type = 3                            # LINK
        f, m = _frd_flu(F), _frd_flu(M)
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = (float(v) for v in f)
        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = (float(v) for v in m)
        self._wrench_pub.publish(msg)

        vel_before = self.vel.copy()
        self._step_server(max(1, int(round(dt / 0.001))))
        self._read_state()
        self.thrust = thrust
        dv = (self.vel - vel_before) / dt
        dv[2] -= G
        self.accel_body = q_to_rotmat(self.q).T @ dv
        self.t += dt
        if detail:
            self.breakdown = {"thrust": float(thrust.sum()), "lift": wb["lift"], "wing_drag": wb["drag"],
                              "ram_drag": ram, "body_drag": body_drag, "alpha": wb["alpha"],
                              "stalled": wb["stalled"], "airspeed": float(np.sqrt(v_air @ v_air)),
                              "power": self.rotors.ideal_power(thrust), "wing_forces": wb.get("wings", [])}

    # ------------------------------------------------------------------ convenience (identical to RigidBody)
    def is_sane(self) -> bool:
        return bool(np.all(np.isfinite(self.pos)) and np.all(np.isfinite(self.vel))
                    and np.all(np.isfinite(self.q)) and abs(self.pos[2]) < 1e5)

    def euler(self) -> tuple[float, float, float]:
        return q_to_euler(self.q)

    @property
    def rotmat(self) -> np.ndarray:
        return q_to_rotmat(self.q)

    @property
    def tilt_deg(self) -> float:
        up = self.rotmat @ np.array([0.0, 0.0, -1.0])
        return math.degrees(math.acos(max(-1.0, min(1.0, -up[2]))))

    def hover_frame_euler(self) -> tuple[float, float, float]:
        """Attitude as PX4 sees it: its level is the structural frame pitched by ``hover_pitch_deg``."""
        Rp = self.rotmat @ self.af.hover_rotation().T
        return q_to_euler(_q_from_rotmat(Rp))
