"""Native-library compatibility helpers for the vision runtime."""

from __future__ import annotations

import ctypes
import os
from pathlib import Path
import sys


def preload_environment_libstdcpp() -> bool:
    """Load the active Python environment's C++ runtime before native modules.

    OpenCV/Open3D can otherwise load Ubuntu's older libstdc++ first.  The
    locally built PointNet2 extension then fails because the already-loaded
    library does not provide the ABI version it was linked against.
    """

    candidate = Path(sys.prefix) / "lib" / "libstdc++.so.6"
    if not candidate.is_file():
        return False
    ctypes.CDLL(str(candidate), mode=ctypes.RTLD_GLOBAL)
    return True


def configure_opencv_qt() -> bool:
    """Use the Linux OpenCV wheel's own plugins before creating any window."""
    if not sys.platform.startswith("linux"):
        return False
    import cv2

    plugins = Path(cv2.__file__).resolve().parent / "qt" / "plugins"
    if not (plugins / "platforms" / "libqxcb.so").is_file():
        return False
    # Conda can supply Qt6 plugins while the OpenCV wheel links to Qt5.
    os.environ["QT_PLUGIN_PATH"] = str(plugins)
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(plugins)
    return True
