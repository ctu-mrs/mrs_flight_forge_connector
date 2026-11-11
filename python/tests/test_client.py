"""The pure-Python client against a local fake simulator: transport, addressing, decoding, no copies."""

import math
import threading
import time

import numpy as np
import pytest

from fake_simulator import response_frame

import flightforge
from flightforge import Compression, FlightForgeConnectionError, FlightForgeError, PixelFormat, Pose, RobotState, Simulator
from flightforge.wire.DepthCameraDataResponse import DepthCameraDataResponseT
from flightforge.wire.EventCameraDataResponse import EventCameraDataResponseT
from flightforge.wire.GetDistanceBatchResponse import GetDistanceBatchResponseT
from flightforge.wire.Image import ImageT
from flightforge.wire.LidarScanResponse import LidarScanResponseT
from flightforge.wire.RgbCameraDataResponse import RgbCameraDataResponseT
from flightforge.wire.SetStateResponse import SetStateResponseT
from flightforge.wire.SetTimeDilationResponse import SetTimeDilationResponseT
from flightforge.wire.SpawnRobotResponse import SpawnRobotResponseT
from flightforge.wire.Vec3f import Vec3fT


def connect(server, **kwargs):
    return Simulator(port=server.port, **kwargs)


def root_buffer(array):
    while isinstance(array, (np.ndarray, memoryview)):
        array = array.base if isinstance(array, np.ndarray) else array.obj
    return array


# transport //{


def test_connects_and_reads_the_api_version(fake):
    server = fake()
    sim = connect(server)
    assert sim.api_version == (1, 0)
    assert sim.schema() == bytes([1, 2, 3])
    assert server.calls[0].robot_id is None and server.calls[0].sensor_id is None


def test_refuses_another_major_version(fake):
    from flightforge.wire.GetInfoResponse import GetInfoResponseT

    def handler(call):
        info = GetInfoResponseT()
        info.apiMajor, info.apiMinor = 2, 0
        return info

    with pytest.raises(FlightForgeError, match="API 2.0"):
        connect(fake(handler))


def test_port_comes_from_the_environment(monkeypatch):
    monkeypatch.setenv("FLIGHTFORGE_PORT", "18999")
    assert flightforge.Connection().port == 18999
    assert flightforge.Connection(port=1234).port == 1234
    monkeypatch.delenv("FLIGHTFORGE_PORT")
    assert flightforge.Connection().port == 8600


def test_unreachable_simulator_fails_after_the_retries():
    start = time.monotonic()
    with pytest.raises(FlightForgeConnectionError):
        Simulator(port=1, connect_attempts=3, retry_delay=0.05)
    assert time.monotonic() - start >= 0.1


def test_stale_replies_are_skipped(fake):
    def handler(call):
        if call.name == "PingRequest":
            return [response_frame(call.seq - 1), response_frame(call.seq + 7), response_frame(call.seq)]

    sim = connect(fake(handler))
    sim.ping()
    sim.ping()


def test_a_reply_split_into_pieces_is_reassembled(fake):

    def handler(call):
        if call.name == "SetTimeDilationRequest":
            body = SetTimeDilationResponseT()
            body.applied = call.body.dilation
            frame = response_frame(call.seq, body)
            return [frame[i : i + 3] for i in range(0, len(frame), 3)]

    sim = connect(fake(handler))
    assert sim.set_time_dilation(0.25) == 0.25


def test_a_late_reply_after_a_timeout_does_not_desync_the_stream(fake):
    late = []

    def handler(call):
        if call.name == "SetTimeDilationRequest":
            body = SetTimeDilationResponseT()
            body.applied = call.body.dilation
            frame = response_frame(call.seq, body)
            if not late:
                late.append(frame)
                return [frame[:5]]
            return [late[0][5:], frame]

    sim = connect(fake(handler), timeout=0.2)
    with pytest.raises(FlightForgeError, match="timed out"):
        sim.set_time_dilation(0.5)
    assert sim.set_time_dilation(2.0) == 2.0


def test_the_server_error_surfaces_with_its_message(fake):
    def handler(call):
        if call.name == "SetStateRequest":
            return [response_frame(call.seq, ok=False, error="joint_positions are not supported yet")]

    robot = connect(fake(handler)).robot(4)
    with pytest.raises(FlightForgeError, match="joint_positions are not supported yet"):
        robot.set_joint_positions([0.1])


def test_one_connection_is_safe_from_many_threads(fake):
    def handler(call):
        if call.name == "SetTimeDilationRequest":
            body = SetTimeDilationResponseT()
            body.applied = call.body.dilation
            return body

    sim = connect(fake(handler))
    errors = []

    def worker(index):
        for i in range(50):
            value = index + i / 100.0
            if sim.set_time_dilation(value) != value:
                errors.append((index, i))

    threads = [threading.Thread(target=worker, args=(t,)) for t in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert not errors


def test_reconnects_after_the_simulator_hangs_up(fake):
    server = fake()
    sim = connect(server)
    server.drop_clients()
    with pytest.raises(FlightForgeConnectionError):
        sim.ping()
    sim.ping()


def test_step_uses_its_own_timeout(fake):
    from flightforge.wire.StepResponse import StepResponseT

    def handler(call):
        if call.name == "StepRequest":
            time.sleep(0.3)
            body = StepResponseT()
            body.simTime, body.tickCount = 1.5, call.body.ticks
            return body

    sim = connect(fake(handler), timeout=0.1, step_timeout=2.0)
    assert sim.step(7) == (1.5, 7)


def test_a_reply_that_is_not_a_response_is_refused(fake):
    def handler(call):
        if call.name == "PingRequest":
            return [b"\x04\x00\x00\x00\x00\x00\x00\x00\xff\xff\xff\x7f"]

    sim = connect(fake(handler))
    with pytest.raises(FlightForgeError):
        sim.ping()


# //}

# addressing and encoding //{


def test_ops_address_the_simulator_the_robot_and_the_sensor(fake):
    def handler(call):
        if call.name == "SpawnRobotRequest":
            body = SpawnRobotResponseT()
            body.robotId = 3
            return body
        if call.name == "GetRangefinderDataRequest":
            from flightforge.wire.RangefinderDataResponse import RangefinderDataResponseT

            body = RangefinderDataResponseT()
            body.range, body.stamp = 2.5, 1.0
            return body

    server = fake(handler)
    sim = connect(server)
    robot = sim.spawn_robot(Pose.at(1.0, 2.0, 3.0, math.pi / 2), "t650", label="uav1")
    assert robot.id == 3

    spawn = server.calls[-1]
    assert spawn.robot_id is None and spawn.body.name == b"t650" and spawn.body.label == b"uav1"
    assert spawn.body.pose.position.y == 2.0 and spawn.body.pose.orientation.z == pytest.approx(math.sqrt(0.5))

    assert robot.rangefinder() == (2.5, 1.0)
    assert (server.calls[-1].robot_id, server.calls[-1].sensor_id) == (3, None)
    robot.rangefinder(sensor_id=5)
    assert (server.calls[-1].robot_id, server.calls[-1].sensor_id) == (3, 5)

    sim.remove_robot(robot)
    assert server.calls[-1].name == "RemoveRobotRequest" and server.calls[-1].robot_id == 3


def test_set_state_carries_every_field(fake):
    def handler(call):
        if call.name == "SetStateRequest" and call.body.waitApplied:
            body = SetStateResponseT()
            body.robotId = 3
            body.base = call.body.state.base
            body.hit = True
            return body

    server = fake(handler)
    robot = connect(server).robot(3)
    state = RobotState(
        base=Pose.at(1.0, 2.0, 3.0),
        joint_positions=[0.5],
        joint_velocities=[10.0, -10.0],
        link_poses=[Pose.at(1.0, 2.0, 3.0), Pose.at(1.1, 2.0, 3.0)],
        base_linear_velocity=(1.0, 0.0, 0.0),
        base_angular_velocity=(0.0, 0.0, 0.5),
    )
    result = robot.set_state(state, collide=True)
    assert result.robot_id == 3 and result.hit and np.allclose(result.base.position, [1.0, 2.0, 3.0])

    sent = server.calls[-1].body
    assert sent.collide and sent.waitApplied
    assert list(sent.state.jointPositions) == [0.5] and list(sent.state.jointVelocities) == [10.0, -10.0]
    assert len(sent.state.linkPoses) == 2 and sent.state.linkPoses[1].position.x == pytest.approx(1.1)
    assert sent.state.baseAngularVelocity.z == 0.5

    assert robot.set_state(Pose.at(0.0, 0.0, 1.0), wait=False) is None
    assert not server.calls[-1].body.waitApplied and server.calls[-1].body.state.jointVelocities is None


def test_euler_angles_are_refused_where_a_quaternion_belongs(fake):
    robot = connect(fake()).robot(0)
    with pytest.raises(ValueError, match="quaternion"):
        robot.set_pose((0.0, 0.0, 1.0), (0.0, 90.0, 0.0))


def test_trajectory_rows_are_time_and_pose(fake):
    server = fake()
    robot = connect(server).robot(1)
    q = flightforge.quat_from_rpy(yaw=0.3)
    robot.set_trajectory(np.array([[0.0, 1.0, 2.0, 3.0, *q], [0.5, 2.0, 2.0, 3.0, *q]]))
    samples = server.calls[-1].body.samples
    assert [s.t for s in samples] == [0.0, 0.5]
    assert samples[1].state.base.position.x == 2.0 and samples[1].state.base.orientation.z == pytest.approx(q[2])

    robot.set_trajectory([(1.0, RobotState(joint_velocities=[5.0]))])
    assert list(server.calls[-1].body.samples[0].state.jointVelocities) == [5.0]

    robot.set_trajectory([])
    assert not server.calls[-1].body.samples

    with pytest.raises(ValueError, match="qw"):
        robot.set_trajectory(np.zeros((2, 7)))


def test_distance_batch_sends_every_point(fake):
    def handler(call):
        if call.name == "GetDistanceBatchRequest":
            body = GetDistanceBatchResponseT()
            body.distances = np.array([math.dist((0, 0, 0), (p.x, p.y, p.z)) for p in call.body.points], dtype=np.float32)
            return body

    sim = connect(fake(handler))
    points = np.array([[3.0, 4.0, 0.0], [0.0, 0.0, 2.0], [1.0, 2.0, 2.0]])
    assert np.allclose(sim.distance_batch(points), [5.0, 2.0, 3.0])
    assert sim.distance_batch(np.zeros((0, 3))).size == 0


# //}

# decoding without copies //{


def test_lidar_scan_reaches_numpy_without_a_copy(fake):
    points = np.array([[1.0, 0.0, 0.0], [np.nan, np.nan, np.nan], [0.0, 2.5, -0.5]], dtype=np.float32)

    def handler(call):
        if call.name == "GetLidarScanRequest":
            body = LidarScanResponseT()
            body.stamp = 4.0
            body.sensorPose = flightforge.messages.pose_to_wire(Pose.at(1.0, 0.0, 2.0, math.pi / 2))
            body.rows, body.cols = 1, 3
            body.points = []
            for p in points:
                v = Vec3fT()
                v.x, v.y, v.z = (float(c) for c in p)
                body.points.append(v)
            if call.body.withLabels:
                body.classId = np.array([5, -1, 6], dtype=np.int32)
                body.instanceId = np.array([9, 0, 10], dtype=np.uint32)
            return body

    robot = connect(fake(handler)).robot(0)
    scan = robot.lidar_scan(labels=True)

    assert scan.points.shape == (3, 3) and scan.points.dtype == np.float32
    assert np.array_equal(scan.points, points, equal_nan=True)
    frame = root_buffer(scan.points)
    assert isinstance(frame, bytearray) and len(frame) > scan.points.nbytes
    assert root_buffer(scan.class_id) is frame

    assert list(scan.hits) == [True, False, True]
    assert list(scan.class_id) == [5, -1, 6] and list(scan.instance_id) == [9, 0, 10]
    assert scan.intensity.size == 0
    assert scan.organized().shape == (1, 3, 3)
    assert np.allclose(scan.in_ground()[0], [1.0, 1.0, 2.0])


def test_raw_camera_frame_reaches_numpy_without_a_copy(fake):
    width, height = 4, 3
    bgra = np.zeros((height, width, 4), dtype=np.uint8)
    bgra[..., 0], bgra[..., 1], bgra[..., 2], bgra[..., 3] = 10, 20, 30, 255
    bgra[1, 2] = (1, 2, 3, 255)

    def handler(call):
        if call.name == "GetRgbCameraDataRequest":
            body = RgbCameraDataResponseT()
            body.stamp = 2.0
            body.image = ImageT()
            body.image.width, body.image.height = width, height
            body.image.format, body.image.compression = PixelFormat.BGRA8, Compression.NONE
            body.image.data = bgra.ravel()
            return body

    robot = connect(fake(handler)).robot(0)
    image, stamp = robot.rgb()
    assert stamp == 2.0 and image.shape == (height, width, 3)
    assert tuple(image[0, 0]) == (30, 20, 10) and tuple(image[1, 2]) == (3, 2, 1)
    frame = root_buffer(image)
    assert isinstance(frame, bytearray) and len(frame) > bgra.nbytes

    raw = robot.rgb(decode=False)
    assert raw.format == PixelFormat.BGRA8 and raw.data.size == bgra.nbytes


def test_depth_is_uint16_millimetres(fake):
    depth = np.array([[0, 1500], [65535, 42]], dtype="<u2")

    def handler(call):
        if call.name == "GetDepthCameraDataRequest":
            body = DepthCameraDataResponseT()
            body.stamp = 1.0
            body.image = ImageT()
            body.image.width, body.image.height = 2, 2
            body.image.format, body.image.compression = PixelFormat.MONO16, Compression.NONE
            body.image.data = np.frombuffer(depth.tobytes(), dtype=np.uint8)
            return body

    image, _ = connect(fake(handler)).robot(0).depth()
    assert image.dtype == np.uint16 and np.array_equal(image, depth)


def test_compressed_frames_decode_to_rgb(fake):
    cv2 = pytest.importorskip("cv2")
    rgb = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    ok, png = cv2.imencode(".png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    assert ok

    def handler(call):
        if call.name == "GetRgbCameraDataRequest":
            body = RgbCameraDataResponseT()
            body.image = ImageT()
            body.image.width, body.image.height = 3, 2
            body.image.format, body.image.compression = PixelFormat.RGB8, Compression.PNG
            body.image.data = png.ravel()
            return body

    image, _ = connect(fake(handler)).robot(0).rgb()
    assert np.array_equal(image, rgb)


def test_events_come_back_as_arrays(fake):
    def handler(call):
        if call.name == "GetEventCameraDataRequest":
            body = EventCameraDataResponseT()
            body.baseStamp, body.stamp, body.width, body.height = 3.0, 3.01, 64, 48
            body.xyp = np.array([5 | (7 << 16) | (1 << 31), 63 | (47 << 16)], dtype=np.uint32)
            body.tUs = np.array([0, 2000], dtype=np.uint32)
            return body

    robot = connect(fake(handler)).robot(0)
    batch = robot.event_batch()
    assert list(batch.x) == [5, 63] and list(batch.y) == [7, 47] and list(batch.polarity) == [1, -1]
    assert np.allclose(batch.t, [3.0, 3.002]) and batch.stamp == 3.01
    assert isinstance(root_buffer(batch.xyp), bytearray)

    events, stamp = robot.events()
    assert events.shape == (2, 4) and stamp == 3.01


# //}


# coverage //{


def _default_reply(call):
    """An empty reply of the type the request expects, enough for the client to read it."""
    import importlib
    import re

    name = call.name
    candidates = [name.replace("Request", "Response"), re.sub(r"^Get(\w+)Request$", r"\1Response", name)]
    for candidate in candidates:
        if not candidate.endswith("Response"):
            continue
        try:
            module = importlib.import_module(f"flightforge.wire.{candidate}")
        except ImportError:
            continue
        return getattr(module, f"{candidate}T")()
    return None


def test_every_request_in_the_schema_has_a_method(fake):
    from flightforge.wire.RequestBody import RequestBody

    server = fake(lambda call: None if call.name == "GetInfoRequest" else _default_reply(call))
    sim = connect(server, timeout=2.0)
    robot = sim.robot(1)
    ops = [
        sim.ping, sim.time, sim.fps, lambda: sim.set_time_dilation(1.0), lambda: sim.set_fixed_timestep(0.0),
        sim.set_lockstep, sim.step, lambda: sim.switch_world("warehouse"), lambda: sim.set_weather("RAIN"),
        lambda: sim.set_daytime(12, 30), lambda: sim.set_graphics_settings("EPIC"), lambda: sim.set_forest_density(2),
        lambda: sim.set_forest_hilly_level(1), lambda: sim.set_mutual_visibility(False), sim.capture_mode,
        lambda: sim.set_capture_mode("ON_DEMAND"), sim.world_origin, sim.class_map, lambda: sim.spawn_objects("objects: []"),
        lambda: sim.remove_object(3), sim.clear_objects, lambda: sim.move_object(3, Pose()), sim.objects, sim.export_scene,
        sim.assets, lambda: sim.load_library("/tmp"), sim.library, sim.world_mesh, lambda: sim.distance_batch([[0, 0, 0]]),
        sim.spawn_robot, sim.robots, lambda: sim.remove_robot(1), lambda: sim.set_states({1: Pose()}),
        robot.description, robot.get_state, lambda: robot.set_pose(Pose()), lambda: robot.set_trajectory([]),
        robot.collision, robot.move_line_visible, lambda: robot.set_move_line_visible(True),
        lambda: robot.add_sensor(flightforge.SensorType.LIDAR), lambda: robot.remove_sensor(2), robot.sensors,
        lambda: robot.add_device("realsense_d435i"), robot.devices, robot.camera_info,
        robot.rgb_camera_config, lambda: robot.set_rgb_camera_config(flightforge.message("RgbCameraConfig")),
        robot.stereo_camera_config, lambda: robot.set_stereo_camera_config(flightforge.message("StereoCameraConfig")),
        robot.fisheye_camera_config, lambda: robot.set_fisheye_camera_config(flightforge.message("FisheyeCameraConfig")),
        robot.event_camera_config, lambda: robot.set_event_camera_config(flightforge.message("EventCameraConfig")),
        robot.lidar_config, lambda: robot.set_lidar_config(flightforge.message("LidarConfig")),
        robot.rgb, robot.stereo, robot.fisheye, robot.depth, robot.segmentation, robot.panoptic, robot.instance_map,
        robot.event_batch, robot.lidar_scan, robot.rangefinder,
    ]
    for op in ops:
        try:
            op()
        except FlightForgeError:
            pass  # empty replies carry no image or config; the request was still sent

    sent = {call.name for call in server.calls}
    expected = {name for name, value in vars(RequestBody).items() if not name.startswith("_") and name != "NONE"}
    assert not expected - sent, f"no method sends {sorted(expected - sent)}"


# //}
