"""Pythonic wrapper over the raw bindings.

Raises on failure instead of returning success flags, decodes images to numpy
arrays, and converts point clouds to (N, k) float arrays.
"""

import json
from dataclasses import dataclass

import numpy as np

from ._native import drone as _d
from ._native import game_mode as _g

DEFAULT_GAME_MODE_PORT = 8551


class FlightForgeError(RuntimeError):
    pass


def _check(ok, what):
    if not ok:
        raise FlightForgeError(f"{what} failed (simulator unreachable or request rejected)")


def _decode_image(buffer, keep_alpha=False):
    """PNG bytes -> HxWx3 uint8 RGB array (HxWx4 RGBA with keep_alpha)."""
    try:
        import cv2

        image = cv2.imdecode(np.frombuffer(bytes(buffer), dtype=np.uint8), cv2.IMREAD_UNCHANGED if keep_alpha else cv2.IMREAD_COLOR)
        if image is None:
            raise FlightForgeError("image decode failed")
        if keep_alpha:
            if image.ndim == 2:
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGRA)
            elif image.shape[2] == 3:
                image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
            return cv2.cvtColor(image, cv2.COLOR_BGRA2RGBA)
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    except ImportError:
        import io

        from PIL import Image

        return np.asarray(Image.open(io.BytesIO(bytes(buffer))).convert("RGB"))


@dataclass
class EventBatch:
    """Events drained from an event camera, as parallel arrays."""

    x: np.ndarray
    y: np.ndarray
    polarity: np.ndarray
    t: np.ndarray
    stamp: float
    dropped: int
    width: int
    height: int

    def __len__(self):
        return int(self.x.size)

    def accumulate(self):
        """(H, W) int32 image: +1 per brighter event, -1 per darker."""
        image = np.zeros((self.height, self.width), dtype=np.int32)
        np.add.at(image, (self.y, self.x), self.polarity.astype(np.int32))
        return image


class Drone:
    """One UAV: pose control and every sensor, addressed pythonically."""

    def __init__(self, address, port):
        self._raw = _d.DroneController(address, port)
        self._port = port
        _check(self._raw.ConnectSimple(), "drone connect")

    @property
    def raw(self):
        return self._raw

    @property
    def port(self):
        """The drone's own server port, which is also its handle for removal."""
        return self._port

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

    def set_propeller_speeds(self, speeds):
        """Angular rate per propeller [rad/s]; returns how many the frame has.

        Order is the frame's own: rear-left, rear-right, front-left, front-right, then
        any propeller an external model added. Sign is the direction of turn, so a
        quad's counter-rotating pairs differ in sign. Passing fewer values than the
        frame has leaves the rest untouched, and an empty list hands them back to the
        uniform default spin."""
        ok, count = self._raw.SetPropellerSpeeds([float(v) for v in speeds])
        _check(ok, "SetPropellerSpeeds")
        return count

    def set_pose_trajectory(self, samples):
        """Hands the pawn a pose timeline to follow: an (N, 7) array of rows
        [t, x, y, z, pitch, yaw, roll], t in sim seconds ascending, position in world
        centimetres, angles in degrees. The pawn interpolates along it at every tick, so
        sensors see the motion between your dynamics samples instead of teleport steps.
        Replaces the previous timeline; an empty array detaches the pawn."""
        arr = np.asarray(samples, dtype=np.float64).reshape(-1, 7)
        _check(self._raw.SetPoseTrajectory(arr), "SetPoseTrajectory")

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
        """Class mask as an (H, W, 3) uint8 RGB array plus the stamp.

        Unlike the other cameras this one ships raw BGR rather than an encoded
        image, so it carries no dimensions of its own - they come from the camera
        config, and the bytes are channel-swapped here.
        """
        ok, data, stamp, _ = self._raw.GetRgbSegmented(sensor_id)
        _check(ok, "GetRgbSegmented")

        ok_config, config = self._raw.GetRgbCameraConfig(sensor_id)
        _check(ok_config, "GetRgbCameraConfig (for the segmentation frame size)")

        expected = config.width * config.height * 3
        if len(data) < expected:
            raise FlightForgeError(
                f"segmentation frame is {len(data)} bytes, expected {expected} "
                f"for {config.width}x{config.height}"
            )

        bgr = np.frombuffer(bytes(data), dtype=np.uint8, count=expected).reshape(config.height, config.width, 3)
        return bgr[:, :, ::-1].copy(), stamp

    def fisheye(self, sensor_id=-1):
        ok, data, stamp, _ = self._raw.GetFisheyeCameraData(sensor_id)
        _check(ok, "GetFisheyeCameraData")
        return _decode_image(data), stamp

    def set_fisheye_calibration(self, fx, fy, cx, cy, k=(0.0, 0.0, 0.0, 0.0), fov_deg=None, sensor_id=-1):
        """Renders the fisheye through a Kannala-Brandt calibration (OpenCV
        fisheye / Kalibr ``equidistant``: fx, fy in pixels per radian, cx, cy in
        OpenCV pixel convention, k1..k4 on the theta polynomial), so an OpenVINS
        config for the real camera works unchanged in the simulator. ``fov_deg``
        caps the rendered angle (default: keep the configured one)."""
        ok, config = self._raw.GetFisheyeCameraConfig(sensor_id)
        _check(ok, "GetFisheyeCameraConfig")
        config.lens_model = 3  # FISHEYE_KANNALA_BRANDT
        config.intrinsics.use_custom = True
        config.intrinsics.fx = float(fx)
        config.intrinsics.fy = float(fy)
        config.intrinsics.cx = float(cx)
        config.intrinsics.cy = float(cy)
        config.kb_k1, config.kb_k2, config.kb_k3, config.kb_k4 = (float(v) for v in k)
        if fov_deg is not None:
            config.fov = float(fov_deg)
        ok = self._raw.SetFisheyeCameraConfig(config, sensor_id)
        _check(ok, "SetFisheyeCameraConfig")

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

    def set_camera_noise(self, shot_scale=2.0e-4, read_sigma=3.0e-3, row_sigma=0.0, enable=True, sensor_id=-1):
        """Enables sensor noise, applied server-side in the linearized domain:
        per-channel sigma = sqrt(shot_scale * signal + read_sigma**2) with signal
        in [0,1], plus optional per-row banding. Deterministic per frame stamp."""
        ok, config = self._raw.GetRgbCameraConfig(sensor_id)
        _check(ok, "GetRgbCameraConfig")
        config.noise.enable = enable
        config.noise.shot_scale = float(shot_scale)
        config.noise.read_sigma = float(read_sigma)
        config.noise.row_sigma = float(row_sigma)
        ok = self._raw.SetRgbCameraConfig(config, sensor_id)
        _check(ok, "SetRgbCameraConfig")

    def set_rolling_shutter(self, readout_time=0.03, enable=True, sensor_id=-1):
        """Enables first-order rolling shutter: rows are shifted by the rotational
        image flow over their readout delay (top-to-bottom readout_time seconds),
        driven by the camera's measured angular velocity between captures."""
        ok, config = self._raw.GetRgbCameraConfig(sensor_id)
        _check(ok, "GetRgbCameraConfig")
        config.rolling_shutter.enable = enable
        config.rolling_shutter.readout_time = float(readout_time)
        ok = self._raw.SetRgbCameraConfig(config, sensor_id)
        _check(ok, "SetRgbCameraConfig")

    def events(self, sensor_id=-1):
        """((N,4) float array of x, y, polarity (+1/-1), stamp — time-sorted; batch stamp).

        Drains everything since the previous call. See :meth:`event_batch` for the
        compact form and the drop count."""
        batch = self.event_batch(sensor_id)
        arr = np.empty((batch.x.size, 4), dtype=np.float64)
        arr[:, 0] = batch.x
        arr[:, 1] = batch.y
        arr[:, 2] = batch.polarity
        arr[:, 3] = batch.t
        return arr, batch.stamp

    def event_batch(self, sensor_id=-1):
        """One drained :class:`EventBatch`: x, y as uint16, polarity as int8 (+1/-1),
        t as float64 sim seconds, plus the batch stamp, the sensor size and how many
        events the readout cap or a full buffer dropped since the previous drain."""
        ok, xyp, t_us, base, stamp, dropped, width, height = self._raw.GetEventCameraData(sensor_id)
        _check(ok, "GetEventCameraData")
        return EventBatch(
            x=(xyp & 0xFFFF).astype(np.uint16),
            y=((xyp >> 16) & 0x7FFF).astype(np.uint16),
            polarity=np.where(xyp >> 31, 1, -1).astype(np.int8),
            t=base + t_us.astype(np.float64) * 1e-6,
            stamp=stamp,
            dropped=dropped,
            width=width,
            height=height,
        )

    def event_camera_config(self, sensor_id=-1):
        ok, config = self._raw.GetEventCameraConfig(sensor_id)
        _check(ok, "GetEventCameraConfig")
        return config

    def set_event_camera_config(self, sensor_id=-1, **fields):
        """Updates the named fields of the event camera's config and applies it.

        Optics: width, height, fov, offset, orientation. Pixel model: contrast_threshold_pos
        / _neg (log units), threshold_sigma (per-pixel mismatch), refractory_period (s),
        cutoff_hz (photoreceptor bandwidth at full scale), leak_rate_hz, shot_noise_rate_hz,
        hot_pixel_fraction, max_event_rate_hz (readout ceiling, 0 = unlimited),
        max_events_per_frame, use_gpu, seed, use_sim_time."""
        config = self.event_camera_config(sensor_id)
        for name, value in fields.items():
            if not hasattr(config, name):
                raise AttributeError(f"EventCameraConfig has no field {name!r}")
            setattr(config, name, value)
        ok = self._raw.SetEventCameraConfig(config, sensor_id)
        _check(ok, "SetEventCameraConfig")

    def instance_segmentation(self, sensor_id=-1):
        """Per-actor instance ids as an (H, W) uint32 array (0 = background) plus
        the frame stamp. Rendered by a dedicated mesh pass: pixel-exact,
        occlusion-correct, no 255-instance stencil cap."""
        ids, _, stamp = self.panoptic(sensor_id)
        return ids, stamp

    def panoptic(self, sensor_id=-1):
        """Instance ids (H, W) uint32 and class ids (H, W) uint8 from one frame of
        the instance camera (the class rides in the PNG's alpha channel), plus the
        stamp. Class ids are leaves of sim.class_map(); 0 = unlabelled."""
        ok, data, stamp = self._raw.GetInstanceSegData(sensor_id)
        _check(ok, "GetInstanceSegData")
        rgba = _decode_image(data, keep_alpha=True)
        ids = rgba[:, :, 0].astype(np.uint32) + (rgba[:, :, 1].astype(np.uint32) << 8) + (rgba[:, :, 2].astype(np.uint32) << 16)
        return ids, rgba[:, :, 3].copy(), stamp

    def instance_map(self, sensor_id=-1):
        """{instance_id: actor_path} snapshotted with the last instance frame."""
        ok, entries = self._raw.GetInstanceSegMap(sensor_id)
        _check(ok, "GetInstanceSegMap")
        return {e.id: e.actor for e in entries}

    def instance_classes(self, sensor_id=-1):
        """{instance_id: class_id} snapshotted with the last instance frame."""
        ok, entries = self._raw.GetInstanceSegMap(sensor_id)
        _check(ok, "GetInstanceSegMap")
        return {e.id: e.class_id for e in entries}

    def camera_info(self, sensor_id=-1):
        """Effective calibration of every camera on the drone (or of one sensor
        id), one dict per eye: ``projection`` (pinhole | fisheye_equidistant |
        fisheye_equisolid | fisheye_stereographic | fisheye_kannala_brandt),
        ``distortion_model`` as sensor_msgs/CameraInfo names it (plumb_bob |
        equidistant), ``K`` (3x3, OpenCV pixel convention), ``D`` (plumb_bob
        k1 k2 p1 p2 k3, or Kannala-Brandt k1..k4 - never empty), ``size`` (w, h),
        ``fov_deg``, ``offset``, ``rotation`` (pitch, yaw, roll) in the config
        convention, ``baseline`` for a stereo pair and ``readout_time`` (0 =
        global shutter). Only cameras that exist are
        listed: a fresh drone creates each default sensor on its first use, so
        configure or read a camera before asking for its calibration."""
        ok, infos = self._raw.GetCameraInfo(sensor_id)
        _check(ok, "GetCameraInfo")
        return [
            {
                "sensor_id": c.sensor_id,
                "sensor_type": c.sensor_type,
                "eye": c.eye,
                "projection": c.projection,
                "distortion_model": c.distortion_model,
                "K": np.array([[c.fx, 0.0, c.cx], [0.0, c.fy, c.cy], [0.0, 0.0, 1.0]]),
                "D": np.array(c.d),
                "size": (c.width, c.height),
                "fov_deg": c.fov_deg,
                "offset": (c.offset_x, c.offset_y, c.offset_z),
                "rotation": (c.rotation_pitch, c.rotation_yaw, c.rotation_roll),
                "baseline": c.baseline,
                "readout_time": c.readout_time,
            }
            for c in infos
        ]

    def depth(self, sensor_id=-1):
        """Depth as an (H, W) uint16 array in millimetres (0 = invalid,
        65535 = clamp - the Kinect/RealSense convention) plus the stamp."""
        ok, data, width, height, stamp = self._raw.GetDepthCameraData(sensor_id)
        _check(ok, "GetDepthCameraData")
        depth_mm = np.frombuffer(data, dtype=np.uint16).reshape(height, width)
        return depth_mm, stamp

    # --- lidar + rangefinder ---

    def lidar(self, sensor_id=-1):
        """((N,4) array of distance, dx, dy, dz; sensor origin offset; stamp)."""
        ok, points, start, stamp = self._raw.GetLidarData(sensor_id)
        _check(ok, "GetLidarData")
        arr = np.array([[p.distance, p.directionX, p.directionY, p.directionZ] for p in points], dtype=np.float64)
        return arr.reshape(-1, 4), np.array([start.x, start.y, start.z]), stamp

    def lidar_scan(self, labels=True, intensity=False, sensor_id=-1):
        """One trace of the lidar with the channels asked for, as a dict:
        ``points`` (N,4) distance, dx, dy, dz; ``start``; ``stamp``; and, when
        requested, per-point ``class_id`` (segmentation stencil, -1 without a
        return), ``instance_id`` (the instance camera's id space, 0 background)
        and ``intensity`` (material class). Prefer this over lidar() +
        lidar_segmented(), which trace separately."""
        ok, scan = self._raw.GetLidarScan(labels, intensity, sensor_id)
        _check(ok, "GetLidarScan")
        scan["start"] = np.array(scan["start"])
        return scan

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

    def meta(self):
        """The server's op catalogue as a dict: ``api``, ``hash`` and one entry per
        op with its id, name, per-op hash and the request/response field trees."""
        ok, meta = self._raw.GetMeta()
        _check(ok, "GetMeta")
        return json.loads(meta["catalogue_json"])

    def class_map(self):
        """The segmentation class tree as {id: {"path", "parent_id"}}; sensors
        emit leaf ids, so collapse through parent_id for coarser labels (see
        collapse_classes)."""
        ok, classes, _ = self._raw.GetClassMap()
        _check(ok, "GetClassMap")
        return {c["id"]: {"path": c["path"], "parent_id": c["parent_id"]} for c in classes}

    def collapse_classes(self, class_ids, depth):
        """Maps an array of leaf class ids to their ancestor at `depth` (1 = top
        level, 2 = second, ...); ids shallower than that stay as they are."""
        tree = self.class_map()

        def ancestor(class_id):
            chain = []
            while class_id in tree:
                chain.append(class_id)
                class_id = tree[class_id]["parent_id"]
            return chain[-depth] if len(chain) >= depth else (chain[0] if chain else 0)

        lut = {i: ancestor(i) for i in set(np.unique(class_ids))}
        return np.vectorize(lambda i: lut.get(int(i), 0))(class_ids)

    def check_compatibility(self, op_names=()):
        """Ops this client was built against that the server lacks or encodes
        differently; empty means compatible. Restrict to the names you use to
        ignore the rest of the API."""
        ok, problems = self._raw.CheckCompatibility(list(op_names))
        _check(ok, "GetMeta")
        return problems

    def spawn_drone(self, position=(0.0, 0.0, 100.0), frame="x500"):
        """Spawns a UAV and returns a connected :class:`Drone`."""
        ok, port = self._raw.SpawnDroneAtLocation(_d.Coordinates(*[float(v) for v in position]), frame)
        _check(ok, f"SpawnDroneAtLocation({frame})")
        return Drone(self._address, port)

    def remove_drone(self, drone_or_port):
        port = drone_or_port if isinstance(drone_or_port, int) else drone_or_port.port
        return self._raw.RemoveDrone(port)

    def switch_world(self, name_or_path):
        return self._raw.SwitchWorldLevel(name_or_path)

    def world_origin(self):
        """The scene frame's origin in world centimetres (the "UAV" PlayerStart)."""
        ok, coords = self._raw.GetWorldOrigin()
        _check(ok, "GetWorldOrigin")
        return np.array([coords.x, coords.y, coords.z])

    # --- frames ---
    #
    # The two halves of the API do not share a frame, and mixing them silently puts
    # things in the wrong place:
    #
    #   spawn_objects / objects / move_object   metres, relative to world_origin
    #   spawn_drone / set_pose / get_position   centimetres, absolute world
    #
    # so an object at [0, 0, 0] is NOT where a drone at (0, 0, 0) sits - they differ by
    # world_origin, which is wherever the level's "UAV" PlayerStart happens to be.

    def scene_to_world(self, position_m):
        """Scene position in metres -> world position in centimetres."""
        return self.world_origin() + np.asarray(position_m, dtype=np.float64) * 100.0

    def world_to_scene(self, position_cm):
        """World position in centimetres -> scene position in metres."""
        return (np.asarray(position_cm, dtype=np.float64) - self.world_origin()) / 100.0

    def set_mutual_visibility(self, enabled):
        """When False, drones are hidden from each other's cameras and lidars —
        each robot perceives the world as if it were alone, which is what makes
        N robots in one world equivalent to N parallel single-robot worlds."""
        return self._raw.SetMutualDroneVisibility(enabled)

    def set_weather(self, type_id):
        return self._raw.SetWeather(type_id)

    def set_daytime(self, hour, minute):
        return self._raw.SetDatetime(hour, minute)

    def sim_time(self):
        """(simulated seconds since the level began, time dilation)."""
        ok, seconds, dilation = self._raw.GetSimTime()
        _check(ok, "GetSimTime")
        return seconds, dilation

    def set_time_dilation(self, dilation):
        """Simulated seconds per wall second; returns what the engine actually applied.

        Below 1 the world runs slow, which is how a consumer that cannot keep up in real
        time still sees every frame: at 0.5 the sim advances half as fast, so twice as
        many frames land per simulated second. The engine clamps the value."""
        ok, applied = self._raw.SetTimeDilation(float(dilation))
        _check(ok, "SetTimeDilation")
        return applied

    def set_fixed_timestep(self, dt, adaptive_min_dt=0.0):
        """Makes every engine tick advance sim time by exactly ``dt`` seconds, however
        long the frame takes to render; 0 restores the wall-clock timestep. Returns
        what is in effect.

        With ``adaptive_min_dt`` > 0 the step shrinks towards that floor whenever an
        event camera saw more than one contrast threshold of change in a pixel, which
        keeps fast motion from aliasing (the ESIM rule)."""
        ok, applied = self._raw.SetFixedTimestep(float(dt), float(adaptive_min_dt))
        _check(ok, "SetFixedTimestep")
        return applied

    def set_lockstep(self, enabled=True):
        """Pauses the world; from then on it advances only through :meth:`step`."""
        _check(self._raw.SetLockstep(bool(enabled)), "SetLockstep")

    def step(self, ticks=1):
        """Runs ``ticks`` engine ticks and returns (sim_time, ticks_so_far) once their
        frames are rendered and every camera has its data: a pose or trajectory set
        before the call is what the sensors saw, and the events are ready after it."""
        ok, sim_time, tick_count = self._raw.Step(int(ticks))
        _check(ok, "Step")
        return sim_time, tick_count

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

    def world_mesh(self, center=(0.0, 0.0, 0.0), extent=(0.0, 0.0, 0.0), max_triangles=0):
        """Ground-truth triangle mesh of the world's static geometry.

        Returns (vertices (N,3) float32, faces (M,3) uint32, truncated, sources), in
        the scene frame: right-handed metres relative to the world origin. `extent`
        half-sizes an axis-aligned box filter around `center`; any zero disables it.
        Drones are excluded; static meshes and landscape are exported, BSP is not."""
        ok, vertices, faces, truncated, sources = self._raw.GetWorldMesh(
            [float(v) for v in center], [float(v) for v in extent], int(max_triangles)
        )
        _check(ok, "GetWorldMesh")
        return vertices.reshape(-1, 3), faces.reshape(-1, 3), truncated, sources

    def distance_batch(self, points, max_radius=5.0, tolerance=0.001, terrain_only=False):
        """Exact Euclidean distance [m] from each point to the nearest collision surface.

        Undiscretized ESDF ground truth: pass the voxel centres of an estimated ESDF and
        compare. `points` is (N,3) in the scene frame. Distances equal `max_radius` when
        truncated and 0.0 inside geometry. Blocks the sim's game thread while computing —
        run it between flights, not during them."""
        pts = np.ascontiguousarray(points, dtype=np.float32).reshape(-1)
        ok, distances = self._raw.GetDistanceBatch(pts, float(max_radius), float(tolerance), bool(terrain_only))
        _check(ok, "GetDistanceBatch")
        return distances

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
