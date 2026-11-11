"""FlightForge Python API (simulator API 0.14).

Layers:
  - the raw pybind bindings (re-exported here): ``RobotClient``,
    ``SimulatorClient`` and the config/data types
  - :class:`flightforge.Simulator` / :class:`flightforge.Robot` — a thin
    pythonic wrapper with numpy image decoding
  - ``flightforge.gym_envs`` — Gymnasium environments (``pip install
    flightforge[gym]``), registered under the ``FlightForge/`` namespace
  - :class:`flightforge.MujocoBridge` — MuJoCo owns the dynamics, FlightForge
    renders (``pip install flightforge[mujoco]``)
"""

from ._native import drone as _drone_module
from ._native import game_mode as _game_mode_module

# raw binding re-exports
CameraCaptureModeEnum = _game_mode_module.CameraCaptureModeEnum
SimulatorClient = _game_mode_module.SimulatorClient

CameraEvent = _drone_module.CameraEvent
CameraExposure = _drone_module.CameraExposure
CameraLensEffects = _drone_module.CameraLensEffects
Coordinates = _drone_module.Coordinates
DeviceInfo = _drone_module.DeviceInfo
RobotClient = _drone_module.RobotClient
EventCameraConfig = _drone_module.EventCameraConfig
FisheyeCameraConfig = _drone_module.FisheyeCameraConfig
LidarConfig = _drone_module.LidarConfig
LidarData = _drone_module.LidarData
LidarIntData = _drone_module.LidarIntData
LidarSegData = _drone_module.LidarSegData
RgbCameraConfig = _drone_module.RgbCameraConfig
Rotation = _drone_module.Rotation
SensorInfo = _drone_module.SensorInfo
SensorType = _drone_module.SensorType
StereoCameraConfig = _drone_module.StereoCameraConfig

from .client import Robot, EventBatch, Simulator  # noqa: E402

try:  # optional: mujoco
    from .mujoco_bridge import MujocoBridge  # noqa: E402
except ImportError:  # pragma: no cover - mujoco not installed
    MujocoBridge = None

__all__ = [
    "CameraCaptureModeEnum",
    "CameraEvent",
    "CameraExposure",
    "CameraLensEffects",
    "Coordinates",
    "DeviceInfo",
    "Robot",
    "RobotClient",
    "EventBatch",
    "EventCameraConfig",
    "FisheyeCameraConfig",
    "SimulatorClient",
    "LidarConfig",
    "LidarData",
    "LidarIntData",
    "LidarSegData",
    "MujocoBridge",
    "RgbCameraConfig",
    "Rotation",
    "SensorInfo",
    "SensorType",
    "Simulator",
    "StereoCameraConfig",
]
