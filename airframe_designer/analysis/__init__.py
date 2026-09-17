"""Static (no PX4 in the loop) analysis: hover allocation margins, cruise trim, momentum-theory power, and the
fast geometric optimiser that searches the same force model."""
from .static import VehicleModel, analyse
from .geometric_optimiser import optimise, default_groups, Variable

__all__ = ["VehicleModel", "analyse", "optimise", "default_groups", "Variable"]
