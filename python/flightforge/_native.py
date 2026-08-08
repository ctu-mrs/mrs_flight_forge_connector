"""Locates the compiled pybind11 modules (flight_forge_drone / flight_forge_game_mode).

The native modules are built by the connector's CMake:
    cmake -S . -B build -DBUILD_PYTHON_LIB=ON && cmake --build build -j

Search order:
  1. normal import (already on sys.path / installed next to this package)
  2. $FLIGHTFORGE_NATIVE_PATH
  3. the connector's ``build/`` directory relative to this checkout
"""

import importlib
import os
import sys
from pathlib import Path


def _candidate_dirs():
    env = os.environ.get("FLIGHTFORGE_NATIVE_PATH")
    if env:
        yield Path(env)

    here = Path(__file__).resolve()
    # <repo>/python/flightforge/_native.py -> <repo>/build
    repo = here.parent.parent.parent
    yield repo / "build"
    yield repo / "build" / "Release"


def _load(name):
    try:
        return importlib.import_module(name)
    except ImportError:
        pass

    for directory in _candidate_dirs():
        if directory.is_dir() and str(directory) not in sys.path:
            sys.path.insert(0, str(directory))
            try:
                return importlib.import_module(name)
            except ImportError:
                continue

    raise ImportError(
        f"cannot import the native module '{name}'. Build it with "
        f"'cmake -S . -B build -DBUILD_PYTHON_LIB=ON && cmake --build build -j' "
        f"in the connector checkout, or point FLIGHTFORGE_NATIVE_PATH at the "
        f"directory containing the compiled modules."
    )


drone = _load("flight_forge_drone")
game_mode = _load("flight_forge_game_mode")
