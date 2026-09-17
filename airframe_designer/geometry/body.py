"""Body segment: the fuselage/central structure - its drawn size and the bluff-body drag it produces."""
from __future__ import annotations

from dataclasses import dataclass, field, asdict


@dataclass
class Body:
    size: list[float] = field(default_factory=lambda: [0.16, 0.16, 0.06])              # visual box, m (x, y, z)
    drag_quadratic: list[float] = field(default_factory=lambda: [0.10, 0.10, 0.20])    # N/(m/s)^2 per body axis
    drag_angular: list[float] = field(default_factory=lambda: [0.005, 0.005, 0.005])   # Nm/(rad/s)^2
    drag_center: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])          # where the body drag acts, structural frame

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "Body":
        return cls(**{k: v for k, v in (d or {}).items() if k in cls.__dataclass_fields__})
