"""Compatibility helpers for ZED Fusion configuration files on Windows."""

from __future__ import annotations

from pathlib import Path
import shutil
import tempfile
from typing import Any

import pyzed.sl as sl


def read_fusion_configuration_file(
    path: Path,
    coordinate_system: Any,
    coordinate_units: Any,
) -> list[Any]:
    """Read a Fusion JSON even when its Windows path contains non-ASCII text.

    ZED SDK 5.4.1's Python reader can return an empty list for an otherwise
    valid JSON when given an absolute path containing characters such as the
    Turkish ``ü`` in ``Masaüstü``.  Staging only the read-only JSON in an ASCII
    temporary directory avoids changing or duplicating the user's source.
    """
    source = path.expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(source)
    try:
        str(source).encode("ascii")
    except UnicodeEncodeError:
        with tempfile.TemporaryDirectory(prefix="zed_fusion_") as directory:
            staged = Path(directory) / "fusion.json"
            shutil.copyfile(source, staged)
            return list(sl.read_fusion_configuration_file(
                str(staged), coordinate_system, coordinate_units
            ))
    return list(sl.read_fusion_configuration_file(
        str(source), coordinate_system, coordinate_units
    ))
