"""Simulator and Robot: every 1.0 op, in the ground frame, metres, quaternions (x, y, z, w) and rad/s."""

import importlib

import numpy as np

from .data import Collision, EventBatch, Frame, LidarScan, RobotInfo, RobotState, SensorInfo, StateResult, TimeInfo
from .geometry import Pose, as_pose
from .messages import Message, message, pose_from_wire, pose_to_wire, table_of, vec3_from_wire, vec3_to_wire
from .transport import Connection, FlightForgeError
from .wire import GetDistanceBatchRequest as _distance_request
from .wire import GetSurfaceHeightRequest as _surface_request
from .wire.CaptureMode import CaptureMode
from .wire.GraphicsQuality import GraphicsQuality
from .wire.Pose import PoseT
from .wire.Projection import Projection
from .wire.RequestBody import RequestBody
from .wire.RobotState import RobotStateT
from .wire.RobotStateEntry import RobotStateEntryT
from .wire.TrajectorySample import TrajectorySampleT
from .wire.Weather import Weather

API_MAJOR = 1

# legacy friendly names onto level names; a full package path also works
WORLD_LEVELS = {
    "valley": "Valley",
    "forest": "Forest",
    "infinite_forest": "infinite_forest",
    "warehouse": "Warehouse",
    "cave": "cave",
    "erding_airbase": "erding_airbase",
    "temesvar": "Temesvar_annotated",
    "electric_towers": "electric_towers",
    "eletric_towers": "electric_towers",
    "race_1": "Race_1",
    "race_2": "Race_2",
    "race_3": "Race_3",
    "industrial_warehouse": "/Game/Worlds/IndustrialWarehouse/Maps/industrial_warehouse",
    "service_tunnel": "service_tunnel",
    "dead_spruce_forest": "dead_spruce_forest",
    "mala_skala": "mala_skala",
    "kayenta_mine": "kayenta_mine",
    "sprind_lab": "sprind_lab",
    "spring_lab": "sprind_lab",
}

# any name resolving to a packed mesh or an on-disk .glb also spawns
ROBOT_NAMES = ("x500", "t650", "a300", "robofly", "wing", "wing_2", "gimbal", "empty", "drone10", "drone20")

_PROJECTIONS = {value: name.lower() for name, value in vars(Projection).items() if not name.startswith("_")}


# wire helpers //{


def _table_class(name):
    return getattr(importlib.import_module(f"flightforge.wire.{name}"), name)


def _call(connection, request, fields, robot_id=None, sensor_id=None, timeout=None):
    """Sends the request table named request with fields and returns the Reply."""
    table = message(request, **fields)._table
    return connection.call(getattr(RequestBody, request), table.Pack, robot_id, sensor_id, timeout)


def _body(reply, name):
    """The reply's body as a Message; bulk vectors stay views into the frame."""
    accessor = reply.body(_table_class(name))
    module = importlib.import_module(f"flightforge.wire.{name}")
    return Message(getattr(module, f"{name}T").InitFromObj(accessor))


def _numpy(value, dtype):
    """A scalar vector from the generated accessors, which answer 0 for an absent one."""
    if isinstance(value, np.ndarray):
        return value
    return np.zeros(0, dtype=dtype)


def _struct_vector(table, name, width):
    """A vector of float structs as an (N, width) float32 view into the frame."""
    count = getattr(table, f"{name}Length")()
    if count == 0:
        return np.zeros((0, width), dtype=np.float32)
    start = getattr(table, name)(0)._tab.Pos
    return np.frombuffer(table._tab.Bytes, dtype="<f4", count=count * width, offset=start).reshape(count, width)


def _create_struct_vector(builder, array):
    """array is (N, k) little-endian, one struct per row."""
    raw = array.tobytes()
    builder.StartVector(array.shape[1] * array.itemsize, array.shape[0], array.itemsize)
    builder.head = builder.head - len(raw)
    builder.Bytes[builder.head : builder.head + len(raw)] = raw
    return builder.EndVector()


def _frame(image, stamp):
    if image is None:
        raise FlightForgeError("the reply carries no image")
    data = _numpy(image.DataAsNumpy(), np.uint8)
    return Frame(stamp, image.Width(), image.Height(), image.Format(), image.Compression(), data)


def _enum(enum_class, value):
    if isinstance(value, str):
        try:
            return getattr(enum_class, value.upper())
        except AttributeError:
            raise ValueError(f"{enum_class.__name__} has no member {value!r}") from None
    return int(value)


def _state_to_wire(state):
    if isinstance(state, Pose):
        state = RobotState(base=state)
    out = RobotStateT()
    if state.base is not None:
        out.base = pose_to_wire(as_pose(state.base))
    if len(state.joint_positions):
        out.jointPositions = [float(v) for v in state.joint_positions]
    if len(state.joint_velocities):
        out.jointVelocities = [float(v) for v in state.joint_velocities]
    if len(state.link_poses):
        out.linkPoses = [pose_to_wire(as_pose(p)) for p in state.link_poses]
    if state.base_linear_velocity is not None:
        out.baseLinearVelocity = vec3_to_wire(state.base_linear_velocity)
    if state.base_angular_velocity is not None:
        out.baseAngularVelocity = vec3_to_wire(state.base_angular_velocity)
    return out


def _state_from_wire(state):
    if state is None:
        return RobotState()
    return RobotState(
        base=pose_from_wire(state.base) if state.base is not None else None,
        joint_positions=[float(v) for v in state.jointPositions] if state.jointPositions is not None else [],
        joint_velocities=[float(v) for v in state.jointVelocities] if state.jointVelocities is not None else [],
        link_poses=[pose_from_wire(p) for p in state.linkPoses] if state.linkPoses else [],
        base_linear_velocity=vec3_from_wire(state.baseLinearVelocity) if state.baseLinearVelocity is not None else None,
        base_angular_velocity=vec3_from_wire(state.baseAngularVelocity) if state.baseAngularVelocity is not None else None,
    )


def _point(value):
    return np.zeros(3) if value is None else value


def _state_result(result):
    return StateResult(result.robot_id, result.base, result.hit, _point(result.impact_point))


def _sensors(entries):
    return [SensorInfo(s.id, s.type, s.link or "") for s in entries or []]


def _sensor(sensor_id):
    return None if sensor_id is None or sensor_id < 0 else int(sensor_id)


# //}


class Simulator:
    """The simulator: worlds, time, scene objects and robots, over one connection the robots share by default."""

    def __init__(self, address="127.0.0.1", port=None, *, timeout=10.0, step_timeout=120.0, connect_attempts=1, retry_delay=0.5):
        """port defaults to FLIGHTFORGE_PORT, else 8600; timeouts in s."""
        self._connection = Connection(address, port, timeout)
        self._connection.connect(connect_attempts, retry_delay)
        self.step_timeout = float(step_timeout)

        major, minor = self.info()[:2]
        if major != API_MAJOR:
            self._connection.close()
            raise FlightForgeError(f"the simulator speaks API {major}.{minor}, this client {API_MAJOR}.x")
        self.api_version = (major, minor)

    @property
    def connection(self):
        return self._connection

    def close(self):
        self._connection.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _call(self, request, /, **fields):
        return _call(self._connection, request, fields)

    # connection //{

    def ping(self):
        self._call("PingRequest")

    def info(self):
        """(api_major, api_minor, compiled schema bytes)."""
        body = self._call("GetInfoRequest").body(_table_class("GetInfoResponse"))
        return body.ApiMajor(), body.ApiMinor(), _numpy(body.SchemaAsNumpy(), np.uint8).tobytes()

    def schema(self):
        """The compiled schema (flightforge.bfbs) the simulator speaks."""
        return self.info()[2]

    # //}

    # time //{

    def time(self):
        body = _body(self._call("GetTimeRequest"), "GetTimeResponse")
        return TimeInfo(body.wall_time, body.sim_time, body.time_dilation)

    def sim_time(self):
        """(sim s since the level began, time dilation)."""
        info = self.time()
        return info.sim_time, info.time_dilation

    def fps(self):
        return _body(self._call("GetFpsRequest"), "GetFpsResponse").fps

    def set_time_dilation(self, dilation):
        """Sim s per wall s; returns what the engine applied after its clamp."""
        return _body(self._call("SetTimeDilationRequest", dilation=float(dilation)), "SetTimeDilationResponse").applied

    def set_fixed_timestep(self, dt, adaptive_min_dt=0.0):
        """Every tick advances sim time by dt s, 0 follows the wall clock; adaptive_min_dt > 0 lets an event camera shrink it."""
        reply = self._call("SetFixedTimestepRequest", dt=float(dt), adaptive_min_dt=float(adaptive_min_dt))
        return _body(reply, "SetFixedTimestepResponse").applied

    def set_lockstep(self, enabled=True):
        """Parks the world paused; from then on it advances only through step()."""
        self._call("SetLockstepRequest", enabled=bool(enabled))

    def step(self, ticks=1):
        """Runs ticks and returns (sim_time, ticks since lockstep began) once every camera has its frames."""
        reply = _call(self._connection, "StepRequest", {"ticks": int(ticks)}, timeout=self.step_timeout)
        body = _body(reply, "StepResponse")
        return body.sim_time, body.tick_count

    # //}

    # world //{

    def switch_world(self, level):
        """A level name, a package path or a name from WORLD_LEVELS."""
        self._call("SwitchWorldRequest", level=WORLD_LEVELS.get(level, level))

    def set_weather(self, weather):
        """A Weather value or its name; on a world without a sky or weather system the error says "has no sky system"."""
        self._call("SetWeatherRequest", weather=_enum(Weather, weather))

    def set_daytime(self, hour, minute=0):
        """Through the world's sky system, else its own sun and sky lights; with neither the error says "has no sky system"."""
        if not (0 <= hour <= 23 and 0 <= minute <= 59):
            raise ValueError("not a time of day")
        self._call("SetDaytimeRequest", hour=int(hour), minute=int(minute))

    def set_graphics_settings(self, quality):
        """A GraphicsQuality value or its name."""
        self._call("SetGraphicsSettingsRequest", quality=_enum(GraphicsQuality, quality))

    def set_forest_density(self, level):
        self._call("SetForestDensityRequest", level=int(level))

    def set_forest_hilly_level(self, level):
        self._call("SetForestHillyLevelRequest", level=int(level))

    def set_mutual_visibility(self, enabled):
        """False hides every robot from the others' sensors, so N robots behave like N separate worlds."""
        self._call("SetMutualVisibilityRequest", enabled=bool(enabled))

    def capture_mode(self):
        return _body(self._call("GetCaptureModeRequest"), "GetCaptureModeResponse").mode

    def set_capture_mode(self, mode):
        """A CaptureMode value or its name."""
        self._call("SetCaptureModeRequest", mode=_enum(CaptureMode, mode))

    def world_origin(self):
        """Where the ground frame's origin sits in Unreal cm, for aligning with editor coordinates only."""
        return _body(self._call("GetWorldOriginRequest"), "GetWorldOriginResponse").origin

    def class_map(self):
        """The segmentation class tree as {id: {"path", "parent_id"}}; sensors emit leaf ids."""
        body = _body(self._call("GetClassMapRequest"), "GetClassMapResponse")
        return {c.id: {"path": c.path, "parent_id": c.parent_id} for c in body.classes or []}

    def collapse_classes(self, class_ids, depth):
        """Maps leaf class ids to their ancestor at depth (1 = top level); shallower ids stay."""
        tree = self.class_map()

        def ancestor(class_id):
            chain = []
            while class_id in tree:
                chain.append(class_id)
                class_id = tree[class_id]["parent_id"]
            return chain[-depth] if len(chain) >= depth else (chain[0] if chain else 0)

        lut = {int(i): ancestor(int(i)) for i in np.unique(class_ids)}
        return np.vectorize(lambda i: lut.get(int(i), 0))(class_ids)

    # //}

    # scene //{

    def spawn_objects(self, yaml_text):
        """Places the objects of a spawn-config YAML document (frame: ground by default); returns their ids."""
        ids = _body(self._call("SpawnObjectsFromYamlRequest", yaml=yaml_text), "SpawnObjectsFromYamlResponse").object_ids
        return [int(i) for i in _numpy(ids, np.int32)]

    def remove_object(self, object_id):
        """True when something was removed."""
        return _body(self._call("RemoveSpawnedObjectRequest", object_id=int(object_id)), "RemoveSpawnedObjectResponse").removed_count > 0

    def clear_objects(self):
        """Removes every spawned object; returns how many."""
        return _body(self._call("RemoveSpawnedObjectRequest", object_id=-1), "RemoveSpawnedObjectResponse").removed_count

    def move_object(self, object_id, pose, orientation=None, scale=(1.0, 1.0, 1.0)):
        """pose is a Pose, or a position with an optional quaternion; scale is per axis."""
        self._call(
            "MoveSpawnedObjectRequest",
            object_id=int(object_id),
            pose=as_pose(pose, orientation),
            scale=Message(vec3_to_wire(scale)),
        )

    def objects(self):
        """{id: {asset, stencil, pose, position, orientation, scale}}, poses reflecting any moves."""
        body = _body(self._call("ListSpawnedObjectsRequest"), "ListSpawnedObjectsResponse")
        out = {}
        for o in body.objects or []:
            pose, scale = o.pose, o.scale
            out[o.id] = {
                "asset": o.asset,
                "stencil": o.stencil,
                "pose": pose,
                "position": pose.position,
                "orientation": pose.orientation,
                "scale": np.ones(3) if scale is None else scale,
            }
        return out

    def export_scene(self):
        """The spawned objects as a spawn-config YAML document that recreates them."""
        return _body(self._call("ExportSceneRequest"), "ExportSceneResponse").yaml or ""

    def assets(self, path_prefix="", name_filter=""):
        """(spawnable asset names, truncated)."""
        body = _body(self._call("ListAssetsRequest", path_prefix=path_prefix, name_filter=name_filter), "ListAssetsResponse")
        return list(body.assets or []), body.truncated

    def load_library(self, path):
        """Loads a manifest or a directory of glTF models on the simulator's machine; returns how many objects it holds."""
        return _body(self._call("LoadLibraryRequest", path=path), "LoadLibraryResponse").count

    def library(self, library="", object_class="", filter="", offset=0, limit=200):
        """(one page of LibraryObject messages, total matches); spawn_name goes in an object's asset."""
        reply = self._call("ListLibraryRequest", library=library, object_class=object_class, filter=filter, offset=int(offset), limit=int(limit))
        body = _body(reply, "ListLibraryResponse")
        return list(body.objects or []), body.total

    def world_mesh(self, center=(0.0, 0.0, 0.0), extent=(0.0, 0.0, 0.0), max_triangles=0):
        """(vertices (N, 3) float32 m, faces (M, 3) uint32, truncated, source count); any extent <= 0 takes the whole world."""
        reply = self._call(
            "GetWorldMeshRequest",
            center=Message(vec3_to_wire(center)),
            extent=Message(vec3_to_wire(extent)),
            max_triangles=int(max_triangles),
        )
        body = reply.body(_table_class("GetWorldMeshResponse"))
        vertices = _struct_vector(body, "Vertices", 3)
        faces = _numpy(body.IndicesAsNumpy(), np.uint32).reshape(-1, 3)
        return vertices, faces, body.Truncated(), body.SourceCount()

    def distance_batch(self, points, max_radius=5.0, tolerance=0.001, terrain_only=False):
        """Distance in m from each (N, 3) point to the nearest collision surface: max_radius when nothing is nearer, 0 inside."""
        points = np.ascontiguousarray(points, dtype="<f4").reshape(-1, 3)

        def build(builder):
            vector = _create_struct_vector(builder, points)
            _distance_request.Start(builder)
            _distance_request.AddPoints(builder, vector)
            _distance_request.AddMaxRadius(builder, float(max_radius))
            _distance_request.AddTolerance(builder, float(tolerance))
            _distance_request.AddTerrainOnly(builder, bool(terrain_only))
            return _distance_request.End(builder)

        reply = self._connection.call(RequestBody.GetDistanceBatchRequest, build)
        body = reply.body(_table_class("GetDistanceBatchResponse"))
        return _numpy(body.DistancesAsNumpy(), np.float32)

    def surface_height(self, points, ignore_robots=True):
        """Height in m of the first surface below each (N, 2) or (N, 3) point, NaN where there is none; no z or a NaN z traces from the top."""
        points = np.atleast_2d(np.asarray(points, dtype="<f8"))
        if points.ndim != 2 or points.shape[1] not in (2, 3):
            raise ValueError(f"points must be (N, 2) or (N, 3), got {points.shape}")
        if points.shape[1] == 2:
            points = np.column_stack([points, np.full(len(points), np.nan)])
        points = np.ascontiguousarray(points, dtype="<f8")

        def build(builder):
            vector = _create_struct_vector(builder, points)
            _surface_request.Start(builder)
            _surface_request.AddPoints(builder, vector)
            _surface_request.AddIgnoreRobots(builder, bool(ignore_robots))
            return _surface_request.End(builder)

        reply = self._connection.call(RequestBody.GetSurfaceHeightRequest, build)
        body = reply.body(_table_class("GetSurfaceHeightResponse"))
        heights = _numpy(body.HeightsAsNumpy(), np.float64)
        if len(heights) != len(points):
            raise FlightForgeError(f"get_surface_height answered {len(heights)} heights for {len(points)} points")
        return heights

    # //}

    # robots //{

    def spawn_robot(self, position=(0.0, 0.0, 2.0), name="x500", *, orientation=None, label="", description_yaml="", own_connection=False):
        """Spawns a robot and returns its Robot; position is a Pose or a point with an optional quaternion.

        description_yaml spawns from an inline description instead of name.
        """
        reply = self._call(
            "SpawnRobotRequest",
            name=name,
            description_yaml=description_yaml,
            pose=as_pose(position, orientation),
            label=label,
        )
        robot_id = _body(reply, "SpawnRobotResponse").robot_id
        return Robot(self, robot_id, own_connection=own_connection)

    def robot(self, robot_id, own_connection=False):
        """A Robot for a robot that already exists."""
        return Robot(self, robot_id, own_connection=own_connection)

    def robots(self):
        body = _body(self._call("ListRobotsRequest"), "ListRobotsResponse")
        return [RobotInfo(r.id, r.name or "", r.kind, r.label or "") for r in body.robots or []]

    def remove_robot(self, robot):
        """A Robot or a robot id."""
        _call(self._connection, "RemoveRobotRequest", {}, robot_id=robot.id if isinstance(robot, Robot) else int(robot))

    def set_states(self, states, collide=False, wait=True):
        """Many robots in one message: {Robot or id: RobotState or Pose}; results are empty unless wait."""
        items = states.items() if isinstance(states, dict) else states
        entries = []
        for robot, state in items:
            entry = RobotStateEntryT()
            entry.robotId = robot.id if isinstance(robot, Robot) else int(robot)
            entry.state = _state_to_wire(state)
            entries.append(entry)
        reply = self._call("SetStatesRequest", states=entries, collide=bool(collide), wait_applied=bool(wait))
        if reply.response.BodyType() == 0:
            return []
        return [_state_result(r) for r in _body(reply, "SetStatesResponse").results or []]

    # //}


class Robot:
    """One robot's state and sensors; a sensor_id of -1 addresses the robot's default sensor of the op's type."""

    def __init__(self, connection, robot_id, *, own_connection=False):
        """connection is a Simulator, a Robot or a Connection; own_connection opens a separate one to the same simulator."""
        if isinstance(connection, (Simulator, Robot)):
            connection = connection.connection
        if own_connection:
            connection = Connection(connection.address, connection.port, connection.timeout).connect()
        self._connection = connection
        self._owns_connection = bool(own_connection)
        self._id = int(robot_id)

    @property
    def id(self):
        return self._id

    @property
    def connection(self):
        return self._connection

    def close(self):
        """Closes the connection when this robot owns it."""
        if self._owns_connection:
            self._connection.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def __repr__(self):
        return f"Robot(id={self._id}, {self._connection!r})"

    def _call(self, request, sensor_id=None, /, **fields):
        return _call(self._connection, request, fields, robot_id=self._id, sensor_id=_sensor(sensor_id))

    # state //{

    def get_state(self):
        """(RobotState, stamp in sim s)."""
        body = _body(self._call("GetStateRequest"), "GetStateResponse")
        return _state_from_wire(body._table.state), body.stamp

    def get_pose(self):
        state, _ = self.get_state()
        return state.base if state.base is not None else Pose()

    def get_position(self):
        """(3,) m."""
        return self.get_pose().position

    def get_orientation(self):
        """(x, y, z, w)."""
        return self.get_pose().orientation

    def set_state(self, state, collide=False, wait=True):
        """A RobotState or a Pose; returns the StateResult once applied, or None with wait=False."""
        reply = self._call("SetStateRequest", state=Message(_state_to_wire(state)), collide=bool(collide), wait_applied=bool(wait))
        if not wait:
            return None
        return _state_result(_body(reply, "SetStateResponse"))

    def set_pose(self, pose, orientation=None, collide=False):
        """Teleports the base: a Pose, or a position with an optional quaternion; returns the StateResult."""
        return self.set_state(RobotState(base=as_pose(pose, orientation)), collide)

    def set_pose_async(self, pose, orientation=None, collide=False):
        """Returns before the pose is applied, for pose streams."""
        self.set_state(RobotState(base=as_pose(pose, orientation)), collide, wait=False)

    def set_joint_velocities(self, velocities):
        """rad/s or m/s per joint in description().joint_names order; empty restores the default spin."""
        self.set_state(RobotState(joint_velocities=list(velocities)))

    def set_joint_positions(self, positions):
        """rad or m per joint in description().joint_names order."""
        return self.set_state(RobotState(joint_positions=list(positions)))

    def set_trajectory(self, samples):
        """The robot follows samples, interpolated each tick, and holds the last; empty releases it.

        samples is an (N, 8) array of [t, x, y, z, qx, qy, qz, qw] with t in sim s ascending, or (t, Pose or RobotState) pairs.
        """
        if isinstance(samples, np.ndarray) or (len(samples) and np.ndim(samples[0]) == 1 and not isinstance(samples[0], tuple)):
            rows = np.asarray(samples, dtype=np.float64)
            if rows.size and (rows.ndim != 2 or rows.shape[1] != 8):
                raise ValueError("trajectory rows are [t, x, y, z, qx, qy, qz, qw]")
            samples = [(row[0], Pose(row[1:4], row[4:8])) for row in rows.reshape(-1, 8)]

        wire_samples = []
        for t, state in samples:
            sample = TrajectorySampleT()
            sample.t = float(t)
            sample.state = _state_to_wire(state)
            wire_samples.append(sample)
        self._call("SetTrajectoryRequest", samples=wire_samples)

    def description(self):
        """name, kind, segmentation_class, links, joint_names, mounts and the description yaml."""
        return _body(self._call("GetDescriptionRequest"), "GetDescriptionResponse")

    def collision(self, clear=False):
        """clear makes the next read report only collisions after this one."""
        body = _body(self._call("GetCollisionRequest", clear=bool(clear)), "GetCollisionResponse")
        return Collision(body.collided, _point(body.impact_point))

    def crashed(self):
        return self.collision().collided

    def move_line_visible(self):
        return _body(self._call("GetMoveLineVisibleRequest"), "GetMoveLineVisibleResponse").visible

    def set_move_line_visible(self, visible):
        self._call("SetMoveLineVisibleRequest", visible=bool(visible))

    # //}

    # sensors and devices //{

    def add_sensor(self, sensor_type, pose=None, link="", mount=""):
        """pose is relative to link (empty is the base link) or to the named mount; returns the sensor id."""
        fields = {"type": int(sensor_type), "link": link, "mount": mount}
        if pose is not None:
            fields["pose"] = as_pose(pose)
        return _body(self._call("AddSensorRequest", **fields), "AddSensorResponse").sensor_id

    def remove_sensor(self, sensor_id):
        self._call("RemoveSensorRequest", int(sensor_id))

    def sensors(self):
        return _sensors(_body(self._call("ListSensorsRequest"), "ListSensorsResponse").sensors)

    def add_device(self, name, pose=None, show_mesh=True, link="", mount=""):
        """Places a device preset from devices(); returns its sensors in creation order."""
        fields = {"device_name": name, "link": link, "mount": mount, "show_mesh": bool(show_mesh)}
        if pose is not None:
            fields["pose"] = as_pose(pose)
        return _sensors(_body(self._call("AddDeviceRequest", **fields), "AddDeviceResponse").sensors)

    def devices(self):
        """{name: description}."""
        body = _body(self._call("ListDevicesRequest"), "ListDevicesResponse")
        return {d.name: d.description for d in body.devices or []}

    def camera_info(self, sensor_id=-1):
        """Calibration per eye of every camera the robot has, or of one sensor.

        Each dict has sensor_id, sensor_type, eye, projection (pinhole, fisheye_*), distortion_model (plumb_bob or
        equidistant), K (3x3, OpenCV pixel convention), D, size (w, h), fov_deg, link, mount (Pose relative to the
        link; optical frame z = mount x, x = -mount y, y = -mount z), baseline_m and readout_time (0 global shutter).
        """
        body = _body(self._call("GetCameraInfoRequest", sensor_id), "GetCameraInfoResponse")
        return [
            {
                "sensor_id": c.sensor_id,
                "sensor_type": c.sensor_type,
                "eye": c.eye,
                "projection": _PROJECTIONS.get(c.projection, str(c.projection)),
                "distortion_model": c.distortion_model,
                "K": np.array([[c.fx, 0.0, c.cx], [0.0, c.fy, c.cy], [0.0, 0.0, 1.0]]),
                "D": np.array(c.d if c.d is not None else [], dtype=np.float64),
                "size": (c.width, c.height),
                "fov_deg": c.fov,
                "link": c.link or "",
                "mount": c.mount,
                "baseline_m": c.baseline_m,
                "readout_time": c.readout_time,
            }
            for c in body.cameras or []
        ]

    # //}

    # configs //{

    def _config(self, kind, sensor_id):
        body = _body(self._call(f"Get{kind}ConfigRequest", sensor_id), f"Get{kind}ConfigResponse")
        if body._table.config is None:
            raise FlightForgeError("the reply carries no config")
        return body.config

    def _set_config(self, kind, config, sensor_id, fields):
        if config is None:
            config = self._config(kind, sensor_id)
        table_of(config, f"{kind}Config")
        config.update(**fields)
        self._call(f"Set{kind}ConfigRequest", sensor_id, config=config)
        return config

    def rgb_camera_config(self, sensor_id=-1):
        """Also the depth, instance and legacy segmentation cameras' config."""
        return self._config("RgbCamera", sensor_id)

    def set_rgb_camera_config(self, config=None, sensor_id=-1, **fields):
        """Sends config (default: the current one) with fields assigned; returns what was sent."""
        return self._set_config("RgbCamera", config, sensor_id, fields)

    def stereo_camera_config(self, sensor_id=-1):
        return self._config("StereoCamera", sensor_id)

    def set_stereo_camera_config(self, config=None, sensor_id=-1, **fields):
        return self._set_config("StereoCamera", config, sensor_id, fields)

    def fisheye_camera_config(self, sensor_id=-1):
        return self._config("FisheyeCamera", sensor_id)

    def set_fisheye_camera_config(self, config=None, sensor_id=-1, **fields):
        return self._set_config("FisheyeCamera", config, sensor_id, fields)

    def event_camera_config(self, sensor_id=-1):
        return self._config("EventCamera", sensor_id)

    def set_event_camera_config(self, config=None, sensor_id=-1, **fields):
        return self._set_config("EventCamera", config, sensor_id, fields)

    def lidar_config(self, sensor_id=-1):
        return self._config("Lidar", sensor_id)

    def set_lidar_config(self, config=None, sensor_id=-1, **fields):
        return self._set_config("Lidar", config, sensor_id, fields)

    # //}

    # calibration //{

    def camera_intrinsics(self, sensor_id=-1):
        """(K 3x3, (width, height)) of the RGB-type camera, as the rendered image has them."""
        config = self.rgb_camera_config(sensor_id)
        i = config.intrinsics
        return np.array([[i.fx, 0.0, i.cx], [0.0, i.fy, i.cy], [0.0, 0.0, 1.0]]), (config.width, config.height)

    def set_camera_intrinsics(self, fx, fy, cx, cy, sensor_id=-1):
        """Explicit pinhole intrinsics in px, OpenCV pixel convention; the resolution stays."""
        self.set_rgb_camera_config(sensor_id=sensor_id, intrinsics=dict(use_custom=True, fx=float(fx), fy=float(fy), cx=float(cx), cy=float(cy)))

    def camera_distortion(self, sensor_id=-1):
        """(enabled, [k1 k2 k3 p1 p2])."""
        d = self.rgb_camera_config(sensor_id).distortion
        return d.enable, np.array([d.k1, d.k2, d.k3, d.p1, d.p2])

    def set_camera_distortion(self, k1=0.0, k2=0.0, k3=0.0, p1=0.0, p2=0.0, enable=True, sensor_id=-1):
        """Brown-Conrady, OpenCV convention."""
        coefficients = dict(k1=k1, k2=k2, k3=k3, p1=p1, p2=p2)
        self.set_rgb_camera_config(sensor_id=sensor_id, distortion=dict(enable=bool(enable), **{k: float(v) for k, v in coefficients.items()}))

    def set_camera_noise(self, shot_scale=2.0e-4, read_sigma=3.0e-3, row_sigma=0.0, enable=True, sensor_id=-1):
        """sigma = sqrt(shot_scale * signal + read_sigma^2), signal in [0, 1], plus per-row banding."""
        noise = dict(enable=bool(enable), shot_scale=float(shot_scale), read_sigma=float(read_sigma), row_sigma=float(row_sigma))
        self.set_rgb_camera_config(sensor_id=sensor_id, noise=noise)

    def set_rolling_shutter(self, readout_time=0.03, enable=True, sensor_id=-1):
        """readout_time in s, top row to bottom."""
        self.set_rgb_camera_config(sensor_id=sensor_id, rolling_shutter=dict(enable=bool(enable), readout_time=float(readout_time)))

    def set_fisheye_calibration(self, fx, fy, cx, cy, k=(0.0, 0.0, 0.0, 0.0), fov_deg=None, sensor_id=-1):
        """A Kannala-Brandt calibration (OpenCV fisheye, Kalibr equidistant): f in px per rad, k1..k4; fov_deg caps the render."""
        from .wire.FisheyeLens import FisheyeLens

        fields = dict(
            lens=FisheyeLens.KANNALA_BRANDT,
            intrinsics=dict(use_custom=True, fx=float(fx), fy=float(fy), cx=float(cx), cy=float(cy)),
            kb_k1=float(k[0]),
            kb_k2=float(k[1]),
            kb_k3=float(k[2]),
            kb_k4=float(k[3]),
        )
        if fov_deg is not None:
            fields["fov"] = float(fov_deg)
        self.set_fisheye_camera_config(sensor_id=sensor_id, **fields)

    # //}

    # cameras //{

    def _image(self, request, response, sensor_id):
        body = self._call(request, sensor_id).body(_table_class(response))
        return _frame(body.Image(), body.Stamp())

    def rgb(self, sensor_id=-1, decode=True):
        """(RGB (H, W, 3) uint8, stamp); raw frames come back as views, decode=False returns the Frame as sent."""
        frame = self._image("GetRgbCameraDataRequest", "RgbCameraDataResponse", sensor_id)
        return (frame.decode(), frame.stamp) if decode else frame

    def stereo(self, sensor_id=-1, decode=True):
        """(left, right, stamp), or with decode=False (left Frame, right Frame)."""
        body = self._call("GetStereoCameraDataRequest", sensor_id).body(_table_class("StereoCameraDataResponse"))
        left, right = _frame(body.Left(), body.Stamp()), _frame(body.Right(), body.Stamp())
        return (left.decode(), right.decode(), body.Stamp()) if decode else (left, right)

    def fisheye(self, sensor_id=-1, decode=True):
        frame = self._image("GetFisheyeCameraDataRequest", "FisheyeCameraDataResponse", sensor_id)
        return (frame.decode(), frame.stamp) if decode else frame

    def depth(self, sensor_id=-1):
        """(H, W) uint16 mm, 0 invalid or too close, 65535 clamped, and the stamp."""
        frame = self._image("GetDepthCameraDataRequest", "DepthCameraDataResponse", sensor_id)
        return frame.decode(), frame.stamp

    def segmentation(self, sensor_id=-1):
        """(class mask as RGB (H, W, 3) uint8, stamp) from the legacy stencil segmentation camera."""
        frame = self._image("GetRgbSegCameraDataRequest", "RgbSegCameraDataResponse", sensor_id)
        return frame.decode(), frame.stamp

    def panoptic(self, sensor_id=-1):
        """(instance ids (H, W) uint32 with 0 background, class ids (H, W) uint8, stamp) from one instance frame."""
        frame = self._image("GetInstanceSegDataRequest", "InstanceSegDataResponse", sensor_id)
        rgba = frame.decode(keep_alpha=True)
        ids = rgba[:, :, 0].astype(np.uint32) | (rgba[:, :, 1].astype(np.uint32) << 8) | (rgba[:, :, 2].astype(np.uint32) << 16)
        return ids, np.ascontiguousarray(rgba[:, :, 3]), frame.stamp

    def instance_segmentation(self, sensor_id=-1):
        """(instance ids (H, W) uint32, stamp)."""
        ids, _, stamp = self.panoptic(sensor_id)
        return ids, stamp

    def _instance_entries(self, sensor_id):
        return _body(self._call("GetInstanceSegMapRequest", sensor_id), "InstanceSegMapResponse").entries or []

    def instance_map(self, sensor_id=-1):
        """{instance id: actor} snapshotted with the last instance frame."""
        return {e.id: e.actor for e in self._instance_entries(sensor_id)}

    def instance_classes(self, sensor_id=-1):
        """{instance id: class id} snapshotted with the last instance frame."""
        return {e.id: e.class_id for e in self._instance_entries(sensor_id)}

    def event_batch(self, sensor_id=-1):
        """Drains the events since the previous call."""
        body = self._call("GetEventCameraDataRequest", sensor_id).body(_table_class("EventCameraDataResponse"))
        return EventBatch.unpack(
            _numpy(body.XypAsNumpy(), np.uint32),
            _numpy(body.TUsAsNumpy(), np.uint32),
            body.BaseStamp(),
            body.Stamp(),
            body.Dropped(),
            body.Width(),
            body.Height(),
        )

    def events(self, sensor_id=-1):
        """((N, 4) float64 of x, y, polarity, t in sim s, batch stamp)."""
        batch = self.event_batch(sensor_id)
        return np.column_stack([batch.x, batch.y, batch.polarity, batch.t]).astype(np.float64).reshape(-1, 4), batch.stamp

    # //}

    # lidar and rangefinder //{

    def lidar_scan(self, intensity=False, labels=False, sensor_id=-1):
        """One scan; points stay a view into the received frame. labels adds class_id and instance_id."""
        reply = self._call("GetLidarScanRequest", sensor_id, with_intensity=bool(intensity), with_labels=bool(labels))
        body = reply.body(_table_class("LidarScanResponse"))
        sensor_pose = body.SensorPose()
        return LidarScan(
            stamp=body.Stamp(),
            sensor_pose=pose_from_wire(None if sensor_pose is None else PoseT.InitFromObj(sensor_pose)),
            rows=body.Rows(),
            cols=body.Cols(),
            points=_struct_vector(body, "Points", 3),
            intensity=_numpy(body.IntensityAsNumpy(), np.int32),
            class_id=_numpy(body.ClassIdAsNumpy(), np.int32),
            instance_id=_numpy(body.InstanceIdAsNumpy(), np.uint32),
        )

    def rangefinder(self, sensor_id=-1):
        """(range in m, NaN for no return; stamp)."""
        body = _body(self._call("GetRangefinderDataRequest", sensor_id), "RangefinderDataResponse")
        return body.range, body.stamp

    # //}
