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

    def camera_intrinsics(self, sensor_id=-1):
        """The RGB camera's effective pinhole intrinsics as a 3x3 K matrix plus
        (width, height). Valid whether the camera runs on a FOV or on custom
        intrinsics — the server always reports what the rendered image has."""
        ok, config = self._raw.GetRgbCameraConfig(sensor_id)
        _check(ok, "GetRgbCameraConfig")
        i = config.intrinsics
        K = np.array([[i.fx, 0.0, i.cx], [0.0, i.fy, i.cy], [0.0, 0.0, 1.0]])
        return K, (config.width, config.height)

    def set_camera_intrinsics(self, fx, fy, cx, cy, sensor_id=-1):
        """Switches the RGB camera (and its projection) to explicit pinhole
        intrinsics in pixels; resolution stays as configured."""
        ok, config = self._raw.GetRgbCameraConfig(sensor_id)
        _check(ok, "GetRgbCameraConfig")
        config.intrinsics.use_custom = True
        config.intrinsics.fx = float(fx)
        config.intrinsics.fy = float(fy)
        config.intrinsics.cx = float(cx)
        config.intrinsics.cy = float(cy)
        ok = self._raw.SetRgbCameraConfig(config, sensor_id)
        _check(ok, "SetRgbCameraConfig")

    def camera_distortion(self, sensor_id=-1):
        """The RGB camera's Brown-Conrady coefficients as (enabled, [k1 k2 k3 p1 p2])."""
        ok, config = self._raw.GetRgbCameraConfig(sensor_id)
        _check(ok, "GetRgbCameraConfig")
        d = config.distortion
        return d.enable, np.array([d.k1, d.k2, d.k3, d.p1, d.p2])

    def set_camera_distortion(self, k1=0.0, k2=0.0, k3=0.0, p1=0.0, p2=0.0, enable=True, sensor_id=-1):
        """Applies Brown-Conrady distortion (OpenCV convention) to the RGB camera.
        The simulator renders an auto-sized overscan frustum and warps, so the
        image matches a real camera calibrated as (K, [k1 k2 k3 p1 p2])."""
        ok, config = self._raw.GetRgbCameraConfig(sensor_id)
        _check(ok, "GetRgbCameraConfig")
        config.distortion.enable = enable
        config.distortion.k1 = float(k1)
        config.distortion.k2 = float(k2)
        config.distortion.k3 = float(k3)
        config.distortion.p1 = float(p1)
        config.distortion.p2 = float(p2)
        ok = self._raw.SetRgbCameraConfig(config, sensor_id)
        _check(ok, "SetRgbCameraConfig")

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

    def set_mutual_visibility(self, enabled):
        """When False, drones are hidden from each other's cameras and lidars —
        each robot perceives the world as if it were alone, which is what makes
        N robots in one world equivalent to N parallel single-robot worlds."""
        return self._raw.SetMutualDroneVisibility(enabled)

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

    def move_object(self, object_id, position, orientation=(0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0)):
        """Repositions a spawned object. The pose is in the spawn config's frame:
        right-handed (ROS) metres and degrees (roll, pitch, yaw) relative to the
        world origin."""
        ok = self._raw.MoveSpawnedObject(
            object_id,
            [float(v) for v in position],
            [float(v) for v in orientation],
            [float(v) for v in scale],
        )
        _check(ok, f"MoveSpawnedObject({object_id})")

    def objects(self):
        """Current objects as {id: {asset, stencil, position, orientation, scale}},
        with poses reflecting any moves since spawning."""
        ok, infos = self._raw.ListSpawnedObjects()
        _check(ok, "ListSpawnedObjects")
        return {
            o.id: {
                "asset": o.asset,
                "stencil": o.stencil,
                "position": np.array(o.position),
                "orientation": np.array(o.orientation),
                "scale": np.array(o.scale),
            }
            for o in infos
        }

    def export_scene(self):
        """The current spawned objects as a spawn-config YAML document; feeding it
        back to spawn_objects recreates the scene, stencils included."""
        ok, yaml_text = self._raw.ExportScene()
        _check(ok, "ExportScene")
        return yaml_text

    def assets(self, path_prefix="", name_filter=""):
        """Spawnable asset names (cooked object paths + on-disk glTF model names).
        Returns (names, truncated)."""
        ok, names, truncated = self._raw.ListAssets(path_prefix, name_filter)
        _check(ok, "ListAssets")
        return list(names), truncated

    def close(self):
        self._raw.Disconnect()
