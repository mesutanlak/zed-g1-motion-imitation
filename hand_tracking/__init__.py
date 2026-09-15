"""Operator-locked hand perception and simulation-only Dex3 targets.

The package deliberately has no Unitree DDS imports.  Importing it can never
open a physical robot transport.
"""

from .contracts import HAND_LANDMARK_NAMES, HandObservation, validate_hand_packet
from .pipeline import HandSourcePipeline

__all__ = [
    "HAND_LANDMARK_NAMES",
    "HandObservation",
    "HandSourcePipeline",
    "validate_hand_packet",
]
