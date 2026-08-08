"""Pythonic wrapper over the raw bindings.

Raises on failure instead of returning success flags, decodes images to numpy
arrays, and converts point clouds to (N, k) float arrays.
"""

import numpy as np

from ._native import drone as _d
from ._native import game_mode as _g

DEFAULT_GAME_MODE_PORT = 8551


class FlightForgeError(RuntimeError):
    pass


def _check(ok, what):
    if not ok:
        raise FlightForgeError(f"{what} failed (simulator unreachable or request rejected)")


def _decode_image(buffer):
    """PNG bytes -> HxWx3 uint8 RGB array."""
    try:
        import cv2

        image = cv2.imdecode(np.frombuffer(bytes(buffer), dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise FlightForgeError("image decode failed")
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    except ImportError:
        import io

        from PIL import Image

        return np.asarray(Image.open(io.BytesIO(bytes(buffer))).convert("RGB"))


class Drone:
    """One UAV: pose control and every sensor, addressed pythonically."""

    def __init__(self, address, port):
        self._raw = _d.DroneController(address, port)
        _check(self._raw.ConnectSimple(), "drone connect")

    @property
    def raw(self):
        return self._raw

    # --- pose ---

    def get_position(self):
        ok, coords = self._raw.GetLocation()
        _check(ok, "GetLocation")
        return np.array([coords.x, coords.y, coords.z])

    def get_orientation(self):
        ok, rot = self._raw.GetRotation()
        _check(ok, "GetRotation")
        return np.array([rot.pitch, rot.yaw, rot.roll])

    def set_pose(self, position, orientation=(0.0, 0.0, 0.0), check_collisions=False):
        """Teleports; returns (hit, impact_point)."""
        ok, _, _, hit, impact = self._raw.SetLocationAndRotation(
            _d.Coordinates(*[float(v) for v in position]),
            _d.Rotation(*[float(v) for v in orientation]),
            check_collisions,
        )
        _check(ok, "SetLocationAndRotation")
        return hit, np.array([impact.x, impact.y, impact.z])

    def set_pose_async(self, position, orientation=(0.0, 0.0, 0.0), check_collisions=False):
        """Fire-and-forget pose push; the call the MuJoCo bridge uses per step."""
        self._raw.SetLocationAndRotationAsync(
            _d.Coordinates(*[float(v) for v in position]),
            _d.Rotation(*[float(v) for v in orientation]),
            check_collisions,
        )

    def crashed(self):
        ok, crashed = self._raw.GetCrashState()
        _check(ok, "GetCrashState")
        return crashed

    # --- cameras ---

    def rgb(self, sensor_id=-1):
        """(image HxWx3 uint8, stamp)."""
        ok, data, stamp, _ = self._raw.GetRgbCameraData(sensor_id)
        _check(ok, "GetRgbCameraData")
        return _decode_image(data), stamp

    def stereo(self, sensor_id=-1):
        """(left, right, stamp)."""
        ok, left, right, stamp = self._raw.GetStereoCameraData(sensor_id)
        _check(ok, "GetStereoCameraData")
        return _decode_image(left), _decode_image(right), stamp

    def segmentation(self, sensor_id=-1):
        ok, data, stamp, _ = self._raw.GetRgbSegmented(sensor_id)
        _check(ok, "GetRgbSegmented")
        return _decode_image(data), stamp

    def fisheye(self, sensor_id=-1):
        ok, data, stamp, _ = self._raw.GetFisheyeCameraData(sensor_id)
        _check(ok, "GetFisheyeCameraData")
        return _decode_image(data), stamp

    def events(self, sensor_id=-1):
        """((N,4) float array of x, y, polarity, stamp — time-sorted; batch stamp)."""
        ok, events, stamp = self._raw.GetEventCameraData(sensor_id)
        _check(ok, "GetEventCameraData")
        arr = np.array([[e.x, e.y, e.polarity, e.stamp] for e in events], dtype=np.float64)
        return arr.reshape(-1, 4), stamp

    # --- lidar + rangefinder ---

    def lidar(self, sensor_id=-1):
        """((N,4) array of distance, dx, dy, dz; sensor origin offset; stamp)."""
        ok, points, start, stamp = self._raw.GetLidarData(sensor_id)
        _check(ok, "GetLidarData")
        arr = np.array([[p.distance, p.directionX, p.directionY, p.directionZ] for p in points], dtype=np.float64)
        return arr.reshape(-1, 4), np.array([start.x, start.y, start.z]), stamp

    def lidar_segmented(self, sensor_id=-1):
        ok, points, start, stamp = self._raw.GetLidarSegData(sensor_id)
        _check(ok, "GetLidarSegData")
        arr = np.array([[p.distance, p.directionX, p.directionY, p.directionZ, p.segmentation] for p in points], dtype=np.float64)
        return arr.reshape(-1, 5), np.array([start.x, start.y, start.z]), stamp

    def rangefinder(self, sensor_id=-1):
        ok, range_ = self._raw.GetRangefinderData(sensor_id)
        _check(ok, "GetRangefinderData")
        return range_

    # --- sensors + devices ---

    def add_sensor(self, sensor_type):
        ok, sensor_id = self._raw.AddSensor(int(sensor_type))
        _check(ok, "AddSensor")
        return sensor_id

    def remove_sensor(self, sensor_id):
        return self._raw.RemoveSensor(sensor_id)

    def sensors(self):
        ok, infos = self._raw.ListSensors()
        _check(ok, "ListSensors")
        return [(s.id, s.type) for s in infos]

    def add_device(self, name, offset=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0), show_mesh=True):
        """Places a device preset (see devices()); returns [(sensor_id, type), ...]."""
        ok, infos = self._raw.AddDevice(
            name,
            _d.Coordinates(*[float(v) for v in offset]),
            _d.Rotation(*[float(v) for v in rotation]),
            show_mesh,
        )
        _check(ok, f"AddDevice({name})")
        return [(s.id, s.type) for s in infos]

    def devices(self):
        ok, infos = self._raw.ListDevices()
        _check(ok, "ListDevices")
        return {d.name: d.description for d in infos}

    def close(self):
        self._raw.Disconnect()


class Simulator:
    """The game-mode connection: worlds, drones, environment, scene objects."""

    def __init__(self, address="127.0.0.1", port=DEFAULT_GAME_MODE_PORT):
        self._address = address
        self._raw = _g.GameModeController(address, port)
        _check(self._raw.ConnectSimple(), "simulator connect")

        ok, (major, minor) = self._raw.GetApiVersion()
        _check(ok, "GetApiVersion")
        self.api_version = (major, minor)

    @property
    def raw(self):
        return self._raw

    def spawn_drone(self, position=(0.0, 0.0, 100.0), frame="x500"):
        """Spawns a UAV and returns a connected :class:`Drone`."""
        ok, port = self._raw.SpawnDroneAtLocation(_d.Coordinates(*[float(v) for v in position]), frame)
        _check(ok, f"SpawnDroneAtLocation({frame})")
        return Drone(self._address, port)

    def remove_drone(self, drone_or_port):
        port = drone_or_port if isinstance(drone_or_port, int) else drone_or_port.raw.getPort()
        return self._raw.RemoveDrone(port)

    def switch_world(self, name_or_path):
        return self._raw.SwitchWorldLevel(name_or_path)

    def world_origin(self):
        ok, coords = self._raw.GetWorldOrigin()
        _check(ok, "GetWorldOrigin")
        return np.array([coords.x, coords.y, coords.z])

    def set_weather(self, type_id):
        return self._raw.SetWeather(type_id)

    def set_daytime(self, hour, minute):
        return self._raw.SetDatetime(hour, minute)

    def fps(self):
        ok, fps = self._raw.GetFps()
        _check(ok, "GetFps")
        return fps

    # --- scene objects ---

    def spawn_objects(self, yaml_text):
        """Places objects from a spawn-config YAML document; returns their handles."""
        ok, object_ids, error = self._raw.SpawnObjectsFromYaml(yaml_text)
        if not ok:
            raise FlightForgeError(f"SpawnObjectsFromYaml failed: {error}")
        return list(object_ids)

    def remove_object(self, object_id):
        ok, removed = self._raw.RemoveSpawnedObject(object_id)
        return ok and removed > 0

    def clear_objects(self):
        ok, removed = self._raw.RemoveAllSpawnedObjects()
        _check(ok, "RemoveAllSpawnedObjects")
        return removed

    def close(self):
        self._raw.Disconnect()
