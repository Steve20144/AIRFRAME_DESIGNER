"""The vehicle segment: the CAD-derived description of a real aircraft, and its conversion to an airframe.

An airframe (``geometry/airframe.py``) is a design: every number in it is editable, and nothing records where it
came from. That is exactly right for exploring a design space and exactly wrong for claiming a real aircraft will
fly. This segment holds the other end: the documents produced by measuring and importing an actual aircraft, with
provenance on every value, and the conversion into an airframe the rest of the project can fly.

The conversion is one-way and regenerable. Editing the generated airframe is how you explore; editing the CAD
documents is how you record reality. Keeping them apart is what lets a result say which it rests on.
"""
from .manifest import CadVehicle, Sourced, load_vehicle
from .to_airframe import airframe_from_cad

__all__ = ["CadVehicle", "Sourced", "load_vehicle", "airframe_from_cad"]
