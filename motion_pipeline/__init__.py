"""Safety-oriented ZED BODY_38 to Unitree G1 motion pipeline primitives.

The package is deliberately transport and robot-API agnostic.  It processes
perception/retargeting data but never opens Unitree DDS topics.
"""

from .calibration import CalibrationManager, CalibrationResult
from .operator_selector import OperatorSelection, OperatorSelector
from .safety import FeasibilityResult, G1FeasibilityFilter, SafetyLevel

__all__ = [
    "CalibrationManager",
    "CalibrationResult",
    "FeasibilityResult",
    "G1FeasibilityFilter",
    "OperatorSelection",
    "OperatorSelector",
    "SafetyLevel",
]
