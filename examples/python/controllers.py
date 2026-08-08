"""Compatibility shim: the RL examples import their symbols from `controllers`.

Build the bindings first:
    cmake -S . -B build -DBUILD_PYTHON_LIB=ON && cmake --build build -j
then either install the resulting modules on PYTHONPATH or run the examples
from a directory containing them (e.g. `PYTHONPATH=../../build python3 ...`).
"""

from flight_forge_drone import (  # noqa: F401
    CameraEvent,
    CameraExposure,
    CameraLensEffects,
    Coordinates,
    DeviceInfo,
    DroneController,
    EventCameraConfig,
    FisheyeCameraConfig,
    LidarConfig,
    LidarData,
    LidarIntData,
    LidarSegData,
    RgbCameraConfig,
    Rotation,
    SensorInfo,
    SensorType,
    StereoCameraConfig,
)
from flight_forge_game_mode import (  # noqa: F401
    CameraCaptureModeEnum,
    GameModeController,
)
