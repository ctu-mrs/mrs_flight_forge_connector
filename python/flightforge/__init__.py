"""FlightForge Python client, API 1.0: pure Python over the FlatBuffers wire.

Everything is in the ground frame (x forward, y left, z up, origin on the floor under the level's UAV PlayerStart),
metres, unit quaternions (x, y, z, w) and rad/s.
"""

from .client import API_MAJOR, ROBOT_NAMES, WORLD_LEVELS, Robot, Simulator
from .data import Collision, EventBatch, Frame, LidarScan, RobotInfo, RobotState, SensorInfo, StateResult, TimeInfo, decode_image
from .geometry import Pose, quat_conjugate, quat_from_rpy, quat_multiply, quat_rotate, rpy_from_quat
from .messages import Message, message
from .transport import DEFAULT_PORT, Connection, FlightForgeConnectionError, FlightForgeError
from .wire.CaptureMode import CaptureMode
from .wire.Compression import Compression
from .wire.Eye import Eye
from .wire.FisheyeLens import FisheyeLens
from .wire.GraphicsQuality import GraphicsQuality
from .wire.JointType import JointType
from .wire.PixelFormat import PixelFormat
from .wire.Projection import Projection
from .wire.RobotKind import RobotKind
from .wire.SensorType import SensorType
from .wire.Weather import Weather

try:
    from .mujoco_bridge import MujocoBridge
except ImportError:
    MujocoBridge = None

__version__ = "1.0.0"

__all__ = [
    "API_MAJOR",
    "CaptureMode",
    "Collision",
    "Compression",
    "Connection",
    "DEFAULT_PORT",
    "EventBatch",
    "Eye",
    "FisheyeLens",
    "FlightForgeConnectionError",
    "FlightForgeError",
    "Frame",
    "GraphicsQuality",
    "JointType",
    "LidarScan",
    "Message",
    "MujocoBridge",
    "PixelFormat",
    "Pose",
    "Projection",
    "ROBOT_NAMES",
    "Robot",
    "RobotInfo",
    "RobotKind",
    "RobotState",
    "SensorInfo",
    "SensorType",
    "Simulator",
    "StateResult",
    "TimeInfo",
    "WORLD_LEVELS",
    "Weather",
    "decode_image",
    "message",
    "quat_conjugate",
    "quat_from_rpy",
    "quat_multiply",
    "quat_rotate",
    "rpy_from_quat",
]
