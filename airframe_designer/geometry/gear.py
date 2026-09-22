"""Landing gear segment: individual legs, each with its own attachment point, length, angles and contact
properties. The foot is where the leg meets the ground; the simulator applies a spring-damper with friction
there, so an asymmetric or too-short set of legs tips the vehicle over as it would in reality."""
from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict

import numpy as np


@dataclass
class Leg:
    name: str = "leg"
    attach: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])   # where the leg meets the structure
    length: float = 0.2           # m, attachment to foot
    tilt_deg: float = 0.0         # lean of the leg forward (+) / backward (-) from straight down
    cant_deg: float = 0.0         # lean outward (+) from the centreline / inward (-)
    foot_radius: float = 0.0      # m, contact starts this far above the foot point (a ball or skid tube)
    stiffness: float = 3000.0     # N/m
    damping: float = 150.0        # N/(m/s)
    friction: float = 0.8         # Coulomb coefficient at the foot
    enabled: bool = True

    @property
    def side(self) -> float:
        return -1.0 if self.attach[1] < 0 else 1.0

    def direction(self) -> np.ndarray:
        """Unit vector from the attachment to the foot, structural frame (down when both angles are zero)."""
        t, c = math.radians(self.tilt_deg), math.radians(self.cant_deg) * self.side
        return np.array([math.sin(t), math.sin(c) * math.cos(t), math.cos(c) * math.cos(t)])

    def foot(self) -> list[float]:
        return (np.asarray(self.attach, float) + self.length * self.direction()).tolist()

    @classmethod
    def from_points(cls, attach, foot, name: str = "leg", **kw) -> "Leg":
        """A leg that reaches from ``attach`` to ``foot``."""
        a, f = np.asarray(attach, float), np.asarray(foot, float)
        d = f - a
        length = float(np.linalg.norm(d))
        leg = cls(name=name, attach=a.tolist(), length=length, **kw)
        if length > 1e-9:
            u = d / length
            tilt = math.degrees(math.asin(max(-1.0, min(1.0, u[0]))))
            cant = math.degrees(math.atan2(u[1], u[2])) if math.hypot(u[1], u[2]) > 1e-9 else 0.0
            leg.tilt_deg = round(tilt, 3)
            leg.cant_deg = round(cant * leg.side, 3)
        return leg

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "Leg":
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


def legs_for_park_pitch(legs: list[Leg], park_pitch_deg: float, min_length: float = 0.03, cg=None,
                        front_margin: float = 0.08, old_park_pitch_deg: float | None = None) -> list[Leg]:
    """A stand that rests the structure at ``park_pitch_deg`` nose-up, built under the same hard points: every foot
    lies on the ground plane and the mean leg length is kept. With ``cg`` given, the stand is also made to *stand*:
    the front feet are moved forward along the ground until they sit ``front_margin`` ahead of the CG's ground
    projection whenever they would be closer (a nose-down park otherwise puts the CG ahead of them), and the rear
    feet keep the horizontal arm they had behind the CG at ``old_park_pitch_deg`` (that arm is the lever the nose
    lift has to work against, so it is not allowed to grow with the tilt). Legs are rebuilt attachment-to-foot, so
    their lean changes; contact properties are kept."""
    active = [l for l in legs if l.enabled]
    if not active:
        return [Leg.from_dict(l.to_dict()) for l in legs]
    phi = math.radians(park_pitch_deg)
    n = np.array([-math.sin(phi), 0.0, math.cos(phi)])          # world "down" seen in the structural frame
    ex = np.array([math.cos(phi), 0.0, math.sin(phi)])          # world forward (horizontal) in the structural frame
    a = np.array([l.attach for l in active], float)
    d = np.array([l.direction() for l in active], float)
    nd = d @ n
    if np.any(nd < 0.2):
        raise ValueError("a leg is nearly parallel to the ground at that parked pitch")
    na = a @ n
    mean_len = float(np.mean([l.length for l in active]))
    # feet on the plane n.f = c, lengths L_i = (c - n.a_i) / (n.d_i), mean(L) = mean_len
    c = (mean_len + float(np.mean(na / nd))) / float(np.mean(1.0 / nd))
    lengths = (c - na) / nd
    if np.any(lengths < min_length):
        raise ValueError(f"parked pitch {park_pitch_deg:g} deg needs a leg shorter than {min_length} m")
    feet = a + lengths[:, None] * d
    if cg is not None:
        cg = np.asarray(cg, float)
        fx = (feet - cg) @ ex                                   # horizontal distance of each foot ahead of the CG
        front = fx > np.median(fx) if len(fx) % 2 else fx >= np.median(fx)   # the forward leg(s): one on a tricycle, a pair on four
        if fx[front].min() < front_margin:
            feet[front] += (front_margin - fx[front].min()) * ex   # slide the front feet forward on the ground
        if old_park_pitch_deg is not None:
            # the rear feet keep their old horizontal arm behind the CG (the nose lift pivots about them)
            phi0 = math.radians(old_park_pitch_deg)
            ex0 = np.array([math.cos(phi0), 0.0, math.sin(phi0)])
            f0 = np.array([l.foot() for l in active], float)
            arm0 = ((f0 - cg) @ ex0)[~front].mean()
            arm = fx[~front].mean()
            feet[~front] += (arm0 - arm) * ex
    out = []
    k = 0
    for l in legs:
        if not l.enabled:
            out.append(Leg.from_dict(l.to_dict())); continue
        props = {q: getattr(l, q) for q in ("foot_radius", "stiffness", "damping", "friction", "enabled")}
        out.append(Leg.from_points(l.attach, feet[k], name=l.name, **props))
        k += 1
    return out


def generate_legs(height: float, spread_x: float, spread_y: float, landed_pitch_deg: float = 0.0,
                  attach_z: float = 0.0, cg=(0.0, 0.0, 0.0), mass: float | None = None, **kw) -> list[Leg]:
    """Four legs whose feet lie on a plane perpendicular to gravity when the airframe stands at
    ``landed_pitch_deg`` nose-up, ``height`` below the CG along the landed 'down'. Attachment points sit at the
    body corners (±spread) at ``attach_z``. This reproduces the schema-1 generated feet."""
    phi = math.radians(landed_pitch_deg)
    c, s = math.cos(phi), math.sin(phi)
    cg = np.asarray(cg, float)
    legs = []
    if mass is not None and "stiffness" not in kw:
        # size the springs for 2 cm of static sink and a damping ratio of 0.8 (no bouncing)
        w_leg = mass * 9.80665 / 4.0
        kw["stiffness"] = round(w_leg / 0.02, 1)
        kw["damping"] = round(2.0 * 0.8 * math.sqrt(kw["stiffness"] * mass / 4.0), 1)
    for name, (x, y) in zip(("FR", "FL", "RR", "RL"), ((spread_x, spread_y), (spread_x, -spread_y), (-spread_x, spread_y), (-spread_x, -spread_y))):
        foot = cg + np.array([c * x - s * height, y, s * x + c * height])   # landed frame (x, y, h) -> structural
        attach = cg + np.array([x, y, attach_z])
        legs.append(Leg.from_points(attach, foot, name=name, **kw))
    return legs
