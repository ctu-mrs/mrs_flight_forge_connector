"""What the clients return: plain records and numpy arrays, many of them views into the received frame."""

from dataclasses import dataclass, field
from typing import List, NamedTuple, Optional

import numpy as np

from .geometry import Pose
from .transport import FlightForgeError
from .wire.Compression import Compression
from .wire.PixelFormat import PixelFormat

_CHANNELS = {
    PixelFormat.RGB8: 3,
    PixelFormat.RGBA8: 4,
    PixelFormat.BGR8: 3,
    PixelFormat.BGRA8: 4,
    PixelFormat.MONO8: 1,
    PixelFormat.MONO16: 1,
}


class SensorInfo(NamedTuple):
    id: int
    type: int  # SensorType
    link: str


class RobotInfo(NamedTuple):
    id: int
    name: str
    kind: int  # RobotKind
    label: str


class TimeInfo(NamedTuple):
    wall_time: float  # s
    sim_time: float  # s since the level began
    time_dilation: float


class Collision(NamedTuple):
    collided: bool
    impact_point: np.ndarray  # m


@dataclass
class RobotState:
    """None and empty fields leave that part of the robot as it is."""

    base: Optional[Pose] = None
    joint_positions: List[float] = field(default_factory=list)  # rad or m, description joint order
    joint_velocities: List[float] = field(default_factory=list)  # rad/s or m/s
    link_poses: List[Pose] = field(default_factory=list)  # ground frame, every link in description order
    base_linear_velocity: Optional[np.ndarray] = None  # m/s, ground frame
    base_angular_velocity: Optional[np.ndarray] = None  # rad/s, body frame


@dataclass
class StateResult:
    robot_id: int
    base: Pose  # where the base ended up, short of a hit
    hit: bool
    impact_point: np.ndarray  # m


@dataclass
class Frame:
    """An image as sent: ``data`` holds the codec's output, or packed rows of ``format`` when uncompressed."""

    stamp: float  # s, sim
    width: int
    height: int
    format: int  # PixelFormat
    compression: int  # Compression
    data: np.ndarray  # uint8, a view into the received frame

    def decode(self, keep_alpha=False):
        return decode_image(self, keep_alpha)


@dataclass
class LidarScan:
    stamp: float  # s, sim
    sensor_pose: Pose  # ground frame, at scan time
    rows: int  # 0 when not organised
    cols: int
    points: np.ndarray  # (N, 3) float32 m in the sensor frame, NaN for no return; a view into the received frame
    intensity: np.ndarray  # (N,) int32, empty unless requested
    class_id: np.ndarray  # (N,) int32, empty unless requested
    instance_id: np.ndarray  # (N,) uint32, empty unless requested

    @property
    def hits(self):
        return np.isfinite(self.points).all(axis=1)

    def ranges(self):
        """(N,) m, NaN for no return."""
        return np.linalg.norm(self.points, axis=1)

    def organized(self):
        """(rows, cols, 3), row 0 the top beam and column 0 the leftmost."""
        if not self.rows:
            raise FlightForgeError("this scan is not organised")
        return self.points.reshape(self.rows, self.cols, 3)

    def in_ground(self):
        """(N, 3) float64 in the ground frame."""
        return self.sensor_pose.transform(self.points)


@dataclass
class EventBatch:
    """Events drained from an event camera, time-sorted, as parallel arrays."""

    x: np.ndarray  # uint16 px
    y: np.ndarray  # uint16 px
    polarity: np.ndarray  # int8, +1 brighter, -1 darker
    t: np.ndarray  # float64 s, sim
    stamp: float  # s, sim; the batch is complete up to here
    dropped: int  # lost to the readout cap or a full buffer since the last drain
    width: int
    height: int
    xyp: np.ndarray = None  # uint32 x | y << 16 | brighter << 31, as received
    t_us: np.ndarray = None  # uint32 us after base_stamp, as received
    base_stamp: float = 0.0  # s, sim

    @classmethod
    def unpack(cls, xyp, t_us, base_stamp, stamp, dropped, width, height):
        xyp = np.asarray(xyp, dtype=np.uint32)
        t_us = np.asarray(t_us, dtype=np.uint32)
        return cls(
            x=(xyp & 0xFFFF).astype(np.uint16),
            y=((xyp >> 16) & 0x7FFF).astype(np.uint16),
            polarity=np.where(xyp >> 31, 1, -1).astype(np.int8),
            t=base_stamp + t_us.astype(np.float64) * 1e-6,
            stamp=stamp,
            dropped=dropped,
            width=width,
            height=height,
            xyp=xyp,
            t_us=t_us,
            base_stamp=base_stamp,
        )

    def __len__(self):
        return int(self.x.size)

    def accumulate(self):
        """(H, W) int32 image: +1 per brighter event, -1 per darker."""
        image = np.zeros((self.height, self.width), dtype=np.int32)
        np.add.at(image, (self.y, self.x), self.polarity.astype(np.int32))
        return image


def decode_image(frame, keep_alpha=False):
    """RGB (H, W, 3) or with keep_alpha RGBA (H, W, 4) uint8; mono formats as (H, W). Raw frames come back as views."""
    if frame.compression == Compression.NONE:
        return _raw_pixels(frame, keep_alpha)
    return _decompress(frame, keep_alpha)


def _raw_pixels(frame, keep_alpha):
    channels = _CHANNELS.get(frame.format)
    if channels is None:
        raise FlightForgeError(f"unknown pixel format {frame.format}")
    data = frame.data
    if frame.format == PixelFormat.MONO16:
        data = data.view("<u2")
    if data.size != frame.width * frame.height * channels:
        raise FlightForgeError(f"a {frame.width}x{frame.height} frame of format {frame.format} cannot be {frame.data.size} bytes")
    if channels == 1:
        return data.reshape(frame.height, frame.width)

    pixels = data.reshape(frame.height, frame.width, channels)
    if frame.format in (PixelFormat.BGR8, PixelFormat.BGRA8):
        rgb = pixels[:, :, 2::-1]
    else:
        rgb = pixels[:, :, :3]
    if not keep_alpha:
        return rgb
    if channels == 4:
        return pixels if frame.format == PixelFormat.RGBA8 else np.dstack([rgb, pixels[:, :, 3]])
    return np.dstack([rgb, np.full(rgb.shape[:2], 255, dtype=np.uint8)])


def _decompress(frame, keep_alpha):
    try:
        import cv2
    except ImportError:
        cv2 = None

    if cv2 is not None:
        image = cv2.imdecode(frame.data, cv2.IMREAD_UNCHANGED)
        if image is None:
            raise FlightForgeError("the image does not decode")
        if image.ndim == 2:
            if frame.format in (PixelFormat.MONO8, PixelFormat.MONO16):
                return image
            return cv2.cvtColor(image, cv2.COLOR_GRAY2RGBA if keep_alpha else cv2.COLOR_GRAY2RGB)
        if image.shape[2] == 3:
            return cv2.cvtColor(image, cv2.COLOR_BGR2RGBA if keep_alpha else cv2.COLOR_BGR2RGB)
        return cv2.cvtColor(image, cv2.COLOR_BGRA2RGBA if keep_alpha else cv2.COLOR_BGRA2RGB)

    try:
        import io

        from PIL import Image
    except ImportError:
        raise FlightForgeError(
            "decoding JPEG or PNG needs opencv-python (pip install flightforge[vision]); "
            "or set the camera's encoding to Compression.NONE for raw frames"
        ) from None

    image = Image.open(io.BytesIO(frame.data.tobytes()))
    if frame.format in (PixelFormat.MONO8, PixelFormat.MONO16) and image.mode in ("L", "I;16", "I"):
        return np.asarray(image)
    return np.asarray(image.convert("RGBA" if keep_alpha else "RGB"))
