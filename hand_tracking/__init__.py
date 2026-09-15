"""Operator-locked hand perception and simulation-only Dex3 targets.

The package deliberately has no Unitree DDS imports.  Importing it can never
open a physical robot transport.
"""

from .contracts import HAND_LANDMARK_NAMES, HandObservation, validate_hand_packet
from .pipeline import HandSourcePipeline
from .retargeting import dex3_control_contract

__all__ = [
    "HAND_LANDMARK_NAMES",
    "HandObservation",
    "HandSourcePipeline",
    "dex3_control_contract",
    "validate_hand_packet",
]
