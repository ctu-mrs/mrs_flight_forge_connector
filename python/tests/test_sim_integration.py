"""Integration tests against a live simulator on FLIGHTFORGE_PORT (default 8600); they skip cleanly without one.

Run with:  pytest -m sim          (only these)
           pytest                 (everything; these skip if no sim)
"""

import math
import os
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from flightforge import (
    Compression,
    FlightForgeError,
    PixelFormat,
    Pose,
    Robot,
    RobotKind,
    RobotState,
    SensorType,
    Simulator,
    message,
)

pytestmark = pytest.mark.sim

# an /Engine path exists in every install, so nothing depends on the host project's content
RING_YAML = """
objects:
  - asset: /Engine/BasicShapes/Cube
    position: [5.0, 0.0, 1.0]
  - asset: /Engine/BasicShapes/Cube
    position: [5.0, 2.0, 1.0]
    scale: 0.5
"""

LOCAL_SCHEMA = Path(__file__).resolve().parents[2] / "include" / "flight_forge_connector" / "wire" / "flightforge.bfbs"


def root_buffer(array):
    while isinstance(array, (np.ndarray, memoryview)):
        array = array.base if isinstance(array, np.ndarray) else array.obj
    return array


def livox(robot, sensor_id, **fields):
    robot.set_lidar_config(sensor_id=sensor_id, enable=True, livox=True, livox_pattern="mid360", range_m=70.0, **fields)


# basics //{


def test_api_version(sim):
    assert sim.api_version == (1, 0)


def test_schema_matches_this_build(sim):
    if not LOCAL_SCHEMA.exists():
        pytest.skip("the connector's compiled schema is not next to this package")
    assert sim.schema() == LOCAL_SCHEMA.read_bytes()


def test_fps_reports(sim):
    assert sim.fps() > 0.0


def test_rgb_capture(robot):
    image, stamp = robot.rgb()
    assert image.ndim == 3 and image.shape[2] == 3 and image.dtype == np.uint8
    assert stamp > 0.0

    jpeg = robot.rgb(decode=False)
    assert jpeg.compression == Compression.JPEG and bytes(jpeg.data[:2]) == b"\xff\xd8"


def test_raw_frames_arrive_without_a_copy(robot):
    config = robot.rgb_camera_config()
    robot.set_rgb_camera_config(encoding=message("ImageEncoding", compression=Compression.NONE))

    lit = False
    for _ in range(40):
        frame = robot.rgb(decode=False)
        assert frame.compression == Compression.NONE and frame.format == PixelFormat.BGRA8
        assert frame.data.size == frame.width * frame.height * 4
        image = frame.decode()
        assert image.shape == (config.height, config.width, 3)
        assert isinstance(root_buffer(image), bytearray)
        lit = bool(image.any())
        if lit:
            break
        time.sleep(0.1)
    assert lit

    scan = robot.lidar_scan()
    assert scan.points.dtype == np.float32 and scan.points.shape[1] == 3
    assert isinstance(root_buffer(scan.points), bytearray)


# //}

# camera calibration //{


def test_camera_info(robot):
    assert robot.camera_info() == []  # nothing exists until a camera is touched
    K, size = robot.camera_intrinsics()
    infos = robot.camera_info()
    rgb = min((c for c in infos if c["sensor_type"] == SensorType.RGB_CAMERA), key=lambda c: c["sensor_id"])
    assert rgb["projection"] == "pinhole" and rgb["distortion_model"] == "plumb_bob"
    assert rgb["size"] == size
    assert np.allclose(rgb["K"], K)
    assert rgb["K"][0, 2] == pytest.approx(size[0] / 2.0 - 0.5)  # OpenCV pixel convention
    assert rgb["D"].shape == (5,) and not rgb["D"].any()
    assert rgb["readout_time"] == 0.0
    assert rgb["link"] == "base_link" and isinstance(rgb["mount"], Pose)
    assert robot.camera_info(rgb["sensor_id"])[0]["sensor_id"] == rgb["sensor_id"]
    with pytest.raises(FlightForgeError):
        robot.camera_info(9999)


def test_stereo_pair_reports_its_baseline(robot):
    sensor_id = robot.add_sensor(SensorType.STEREO_CAMERA)
    left = robot.stereo_camera_config(sensor_id).mount_left
    robot.set_stereo_camera_config(sensor_id=sensor_id, mount_right=Pose(left.position - [0.0, 0.12, 0.0], left.orientation))
    eyes = robot.camera_info(sensor_id)
    assert len(eyes) == 2 and eyes[0]["eye"] == 0 and eyes[1]["eye"] == 1
    assert eyes[0]["baseline_m"] == pytest.approx(0.12)
    left_image, right_image, stamp = robot.stereo(sensor_id)
    assert left_image.shape == right_image.shape and stamp > 0.0
    robot.remove_sensor(sensor_id)


def test_fisheye_calibration_round_trip(robot):
    sensor_id = robot.add_sensor(SensorType.FISHEYE_CAMERA)
    k = (-0.012, 0.021, -0.017, 0.004)
    robot.set_fisheye_calibration(254.3, 252.9, 405.7, 291.2, k=k, fov_deg=190.0, sensor_id=sensor_id)
    (info,) = robot.camera_info(sensor_id)
    assert info["projection"] == "fisheye_kannala_brandt" and info["distortion_model"] == "equidistant"
    assert np.allclose(info["K"], [[254.3, 0.0, 405.7], [0.0, 252.9, 291.2], [0.0, 0.0, 1.0]])
    assert np.allclose(info["D"], k)
    assert info["fov_deg"] == 190.0
    image, _ = robot.fisheye(sensor_id)
    assert image.ndim == 3
    robot.remove_sensor(sensor_id)


def test_intrinsics_roundtrip(robot):
    K0, (width, height) = robot.camera_intrinsics()
    assert K0[0, 0] > 0.0 and K0[0, 2] == pytest.approx(width / 2.0, abs=1.0)

    robot.set_camera_intrinsics(fx=500.0, fy=505.0, cx=300.0, cy=250.0)
    K1, _ = robot.camera_intrinsics()
    assert np.allclose([K1[0, 0], K1[1, 1], K1[0, 2], K1[1, 2]], [500.0, 505.0, 300.0, 250.0])

    image, _ = robot.rgb()
    assert image.shape[:2] == (height, width)


def test_distortion_roundtrip(robot):
    robot.set_camera_distortion(k1=-0.2, k2=0.05, p1=0.001)
    enabled, coefficients = robot.camera_distortion()
    assert enabled
    assert np.allclose(coefficients[:2], [-0.2, 0.05]) and coefficients[3] == pytest.approx(0.001)

    # the served image stays at the nominal resolution despite the overscan render
    _, (width, height) = robot.camera_intrinsics()
    image, _ = robot.rgb()
    assert image.shape[:2] == (height, width)

    robot.set_camera_distortion(enable=False)


def test_noise_and_rolling_shutter_config(robot):
    robot.set_camera_noise(shot_scale=3.0e-4, read_sigma=4.0e-3)
    robot.set_rolling_shutter(0.025)

    config = robot.rgb_camera_config()
    assert config.noise.enable and config.noise.shot_scale == pytest.approx(3.0e-4)
    assert config.rolling_shutter.enable and config.rolling_shutter.readout_time == pytest.approx(0.025)

    robot.set_camera_noise(enable=False)
    robot.set_rolling_shutter(0.0, enable=False)


# //}

# scene //{


def test_scene_editor_roundtrip(sim):
    sim.clear_objects()
    ids = sim.spawn_objects(RING_YAML)
    assert len(ids) == 2

    try:
        objects = sim.objects()
        assert set(ids) <= set(objects)
        assert np.allclose(objects[ids[0]]["position"], [5.0, 0.0, 1.0], atol=0.05)

        sim.move_object(ids[0], Pose.at(6.0, -1.0, 2.0, math.radians(45.0)))
        moved = sim.objects()[ids[0]]
        assert np.allclose(moved["position"], [6.0, -1.0, 2.0], atol=0.05)
        assert moved["pose"].yaw == pytest.approx(math.radians(45.0), abs=0.01)

        exported = sim.export_scene()
        assert "Cube" in exported and "frame: ground" in exported
        assert "class: prop/box" in exported  # the host project's Cube rule

        assert sim.clear_objects() == 2
        assert sim.objects() == {}

        restored = sim.spawn_objects(exported)
        assert len(restored) == 2
        assert np.allclose(sim.objects()[restored[0]]["position"], [6.0, -1.0, 2.0], atol=0.05)
    finally:
        sim.clear_objects()


def test_old_scene_frames_are_refused_by_name(sim):
    with pytest.raises(FlightForgeError, match="ground"):
        sim.spawn_objects("frame: origin\nobjects:\n  - asset: /Engine/BasicShapes/Cube\n")
    with pytest.raises(FlightForgeError, match="unreal"):
        sim.spawn_objects("frame: world\nobjects:\n  - asset: /Engine/BasicShapes/Cube\n")


def test_asset_catalog(sim):
    names, truncated = sim.assets(name_filter="cube")
    assert isinstance(names, list) and all(isinstance(n, str) for n in names)
    assert isinstance(truncated, bool)


def test_library_lists_what_it_loaded(sim, tmp_path):
    manifest = tmp_path / "library.yaml"
    manifest.write_text(
        "name: pytest\nobjects:\n  - id: crate\n    asset: /Engine/BasicShapes/Cube\n    class: box\n"
        "    description: a wooden crate\n    scale: 2.0\n    height: 2.0\n    base_z: 0.5\n"
    )
    assert sim.load_library(str(manifest)) == 1
    objects, total = sim.library("pytest", filter="WOODEN", limit=10)
    assert total == 1 and objects[0].spawn_name == "lib:pytest/crate" and objects[0]["object_class"] == "box"


def test_mutual_visibility_toggles(sim):
    sim.set_mutual_visibility(False)
    sim.set_mutual_visibility(True)


def test_world_mesh_is_ground_truth(sim):
    """A cube of known scale: if its vertices span anything but scale^3 metres at the requested position, the
    export's transforms or frames are wrong."""
    sim.clear_objects()
    sim.spawn_objects(
        "objects:\n"
        "  - asset: /Game/LevelPrototyping/Meshes/SM_Cube.SM_Cube\n"
        "    position: [15.0, 7.0, 4.0]\n"
        "    scale: [2.0, 2.0, 2.0]\n"
    )
    time.sleep(0.3)

    try:
        vertices, faces, truncated, sources = sim.world_mesh(center=(15.0, 7.0, 4.0), extent=(6.0, 6.0, 6.0))
        assert sources >= 1
        assert len(faces) > 0 and faces.shape[1] == 3

        near = np.all(np.abs(vertices - np.array([15.0, 7.0, 4.0])) < 3.2, axis=1)
        assert near.sum() >= 8, "the spawned cube is missing from the mesh"

        span = vertices[near].max(axis=0) - vertices[near].min(axis=0)
        assert np.allclose(span, 2.0, atol=0.05), f"cube span {span} is not 2 m"
    finally:
        sim.clear_objects()


# //}

# sensors and devices //{


def test_sensor_lifecycle(robot):
    sensor_id = robot.add_sensor(SensorType.RGB_CAMERA)
    assert sensor_id >= 0
    assert any(s.id == sensor_id and s.type == SensorType.RGB_CAMERA and s.link == "base_link" for s in robot.sensors())
    robot.remove_sensor(sensor_id)
    with pytest.raises(FlightForgeError):
        robot.remove_sensor(sensor_id)


def test_devices_listed(robot):
    devices = robot.devices()
    assert "realsense_d435i" in devices and "azure_kinect_dk" in devices
    sensors = robot.add_device("realsense_d435i", Pose.at(0.12, 0.0, -0.02))
    assert len(sensors) > 1 and all(s.link == "base_link" for s in sensors)


def test_depth_capture(robot):
    depth, stamp = robot.depth()
    assert depth.dtype == np.uint16 and depth.ndim == 2 and stamp > 0.0
    # over open ground something must be in range: not all zeros, not all clamp
    assert (depth > 0).any() and (depth < 65535).any()


def test_instance_segmentation_smoke(sim, robot):
    sim.spawn_objects(RING_YAML)
    try:
        instance_ids, stamp = robot.instance_segmentation()
        assert instance_ids.dtype == np.uint32 and instance_ids.ndim == 2
        assert stamp > 0.0

        mapping = robot.instance_map()
        visible = set(np.unique(instance_ids).tolist()) - {0}
        # every visible id must be explainable
        assert visible <= set(mapping)
    finally:
        sim.clear_objects()


def test_segmentation_returns_a_mask_not_a_photograph(robot):
    sensor_id = robot.add_sensor(SensorType.RGB_SEG_CAMERA)
    colours = 0
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        mask, _ = robot.segmentation(sensor_id)
        colours = len(np.unique(mask.reshape(-1, 3), axis=0))
        if 1 < colours < 512:
            break
        time.sleep(0.2)
    assert 1 < colours < 512
    robot.remove_sensor(sensor_id)


def test_lidar_scan_labels_agree_with_the_instance_camera(sim):
    """A point and a pixel on the same object carry the same instance id, and the point's class id is the
    object's class."""
    sim.clear_objects()
    robot = sim.spawn_robot((0.0, 0.0, 1.5))
    try:
        camera = robot.add_sensor(SensorType.INSTANCE_SEG_CAMERA)
        (cube_id,) = sim.spawn_objects("objects:\n  - asset: /Engine/BasicShapes/Cube\n    position: [4.0, 0.0, 1.5]\n    scale: 2.0\n")
        stencil = sim.objects()[cube_id]["stencil"]
        time.sleep(1.5)
        for _ in range(3):
            robot.instance_segmentation(camera)

        image, classes, _ = robot.panoptic(camera)
        h, w = image.shape
        pixel_id = int(image[h // 2, w // 2])
        pixel_class = int(classes[h // 2, w // 2])
        assert pixel_id != 0

        # the class comes from the tagger: a rule when the project has one for the cube, else the stencil
        tree = sim.class_map()
        by_path = {v["path"]: k for k, v in tree.items()}
        expected_class = by_path.get("prop/box", stencil)
        assert pixel_class == expected_class
        assert robot.instance_classes(camera)[pixel_id] == expected_class

        scan = robot.lidar_scan(labels=True)
        points = scan.points
        assert scan.class_id.shape == (len(points),) and scan.instance_id.shape == (len(points),)
        assert scan.intensity.size == 0  # not requested
        hit = scan.hits
        direction = np.where(hit[:, None], points / np.linalg.norm(np.where(hit[:, None], points, 1.0), axis=1, keepdims=True), 0.0)
        forward = hit & (direction[:, 0] > 0.995)  # returns within ~6 deg of the optical axis
        assert forward.sum() > 0
        assert set(scan.instance_id[forward].tolist()) == {pixel_id}
        assert set(scan.class_id[forward].tolist()) == {expected_class}
        assert np.all(scan.class_id[~hit] == -1) and np.all(scan.instance_id[~hit] == 0)

        if "prop/box" in by_path:
            # leaf ids collapse through the tree
            assert tree[expected_class]["parent_id"] == by_path["prop"]
            assert sim.collapse_classes(np.array([expected_class]), 1)[0] == by_path["prop"]
    finally:
        sim.clear_objects()
        sim.remove_robot(robot)


def test_segmentation_covers_rendered_geometry(sim):
    """Every pixel the renderer put geometry into carries an instance id: the depth buffer is the oracle, which
    unlike a physics trace sees masked leaves, wind and foliage without collision the same way the id pass must."""
    robot = sim.spawn_robot(Pose.from_rpy((0.0, 0.0, 3.0), pitch=math.radians(20.0)))
    try:
        camera = robot.add_sensor(SensorType.INSTANCE_SEG_CAMERA)
        time.sleep(2.0)
        for _ in range(4):
            robot.panoptic(camera)
            robot.depth()
        ids, _, _ = robot.panoptic(camera)
        depth, _ = robot.depth()
        geometry = (depth > 0) & (depth < 65535)
        assert geometry.sum() > 10000
        unlabelled = geometry & (ids == 0)
        assert unlabelled.sum() / geometry.sum() < 0.01
    finally:
        sim.remove_robot(robot)


def test_instance_segmentation_survives_scene_edits(sim):
    """Destroying actors while an instance camera runs must not kill the simulator."""
    robot = sim.spawn_robot((0.0, 0.0, 3.0))
    try:
        sensor_id = robot.add_sensor(SensorType.INSTANCE_SEG_CAMERA)
        time.sleep(0.5)

        for i in range(8):
            sim.spawn_objects(
                "objects:\n"
                "  - asset: /Game/LevelPrototyping/Meshes/SM_Cube.SM_Cube\n"
                f"    position: [{6 + i}, 0.0, 1.0]\n"
                "    scale: [1.5, 1.5, 1.5]\n"
            )
            robot.instance_segmentation(sensor_id)
            sim.clear_objects()
            robot.instance_segmentation(sensor_id)

        # the simulator answering at all is the assertion; a dangling proxy took it down
        assert sim.fps() > 0
        ids, _ = robot.instance_segmentation(sensor_id)
        assert ids.shape[0] > 0 and ids.shape[1] > 0
    finally:
        sim.remove_robot(robot)
        sim.clear_objects()


def test_sensor_body_occludes_the_airframe_but_not_itself(sim):
    """With a body, a lidar sees the robot it is bolted to and never its own housing."""
    robot = sim.spawn_robot((0.0, 0.0, 4.0))
    try:
        sensor_id = robot.add_sensor(SensorType.LIDAR)

        def sweep(show_body):
            # the stock profile stops 7 degrees below horizontal and never looks down
            robot.set_lidar_config(
                sensor_id=sensor_id,
                enable=True,
                livox=False,
                range_m=50.0,
                mount=Pose.at(0.0, 0.0, 0.5),
                fov_down=85.0,
                fov_up=15.0,
                show_sensor_body=show_body,
            )
            time.sleep(0.9)
            robot.lidar_scan(sensor_id=sensor_id)
            time.sleep(0.3)

            scan = robot.lidar_scan(sensor_id=sensor_id)
            distance = scan.ranges()
            hit = scan.hits
            elevation = np.degrees(np.arcsin(np.clip(np.where(hit, scan.points[:, 2] / np.where(hit, distance, 1.0), 0.0), -1, 1)))

            below = hit & (elevation < -55) & (distance > 0.2) & (distance < 0.8)
            housing = hit & (distance < 0.15)
            return int(below.sum()), int(housing.sum())

        off_frame, off_housing = sweep(False)
        on_frame, on_housing = sweep(True)

        assert off_frame == 0, "the airframe must stay invisible without a sensor body"
        assert on_frame > 0, "with a body the lidar must see the airframe it is mounted on"
        assert off_housing == 0 and on_housing == 0, "a lidar must never return its own housing"
    finally:
        sim.remove_robot(robot)


def test_gltf_model_is_visible_to_the_lidar(sim):
    """A runtime-loaded glTF model must have collision, not just a mesh."""
    models = os.environ.get(
        "FF_MODEL_DIR",
        "/home/david/Documents/Unreal Projects/TestPlugin/Plugins/FlightForgePlugin/Content/ExternalModels",
    )
    if not os.path.isdir(models):
        pytest.skip(f"no glTF models to spawn (set FF_MODEL_DIR; tried {models})")

    sim.clear_objects()
    try:
        ids = sim.spawn_objects(f"model_paths:\n  - {models}\nobjects:\n  - asset: drone20\n    position: [6.0, 0.0, 2.0]\n    scale: 20.0\n")
    except FlightForgeError as error:
        pytest.skip(f"could not spawn a glTF model: {error}")
    if not ids:
        pytest.skip("could not spawn a glTF model")

    robot = sim.spawn_robot((0.0, 0.0, 2.0))
    try:
        sensor_id = robot.add_sensor(SensorType.LIDAR)
        livox(robot, sensor_id, mount=Pose.at(0.0, 0.0, 0.6))

        time.sleep(1.2)
        robot.lidar_scan(sensor_id=sensor_id)
        time.sleep(0.3)  # a Livox scan holds what accumulated since the previous one
        xyz = robot.lidar_scan(sensor_id=sensor_id).points
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        assert len(xyz) > 0

        # the model spans roughly 3-9 m ahead at this scale; anything landing there is it
        forward = xyz[
            (np.abs(np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))) < 20.0)
            & (np.linalg.norm(xyz[:, :2], axis=1) < 10.0)
            & (np.abs(xyz[:, 2]) < 4.0)
        ]
        assert len(forward) > 20, "the glTF model returned no lidar hits - collision is missing"
    finally:
        sim.clear_objects()
        sim.remove_robot(robot)


# //}

# robots //{


def test_robot_state_round_trips(sim, robot):
    state = RobotState(
        base=Pose.at(1.0, -2.0, 3.0, math.radians(30.0)),
        base_linear_velocity=(1.0, 2.0, 3.0),
        base_angular_velocity=(0.1, 0.2, 0.3),
    )
    result = robot.set_state(state)
    assert result.robot_id == robot.id and np.allclose(result.base.position, [1.0, -2.0, 3.0], atol=0.01)

    read, stamp = robot.get_state()
    assert np.allclose(read.base.position, [1.0, -2.0, 3.0], atol=0.01)
    assert read.base.yaw == pytest.approx(math.radians(30.0), abs=1e-4)
    assert read.base_linear_velocity[1] == pytest.approx(2.0) and read.base_angular_velocity[2] == pytest.approx(0.3)
    assert stamp > 0.0
    assert np.allclose(robot.get_position(), [1.0, -2.0, 3.0], atol=0.01)

    assert any(r.id == robot.id and r.label == "test" and r.name == "x500" and r.kind == RobotKind.MULTIROTOR for r in sim.robots())

    (moved,) = sim.set_states({robot: Pose.at(0.0, 0.0, 2.0)})
    assert moved.robot_id == robot.id and np.allclose(moved.base.position, [0.0, 0.0, 2.0], atol=0.01)
    assert sim.set_states({robot.id: Pose.at(0.0, 0.0, 2.5)}, wait=False) == []
    with pytest.raises(FlightForgeError):
        sim.set_states({99999: Pose.at(0.0, 0.0, 2.0)})
    assert robot.collision().collided in (True, False)


def test_robot_describes_its_propellers(robot):
    description = robot.description()
    assert description.links[0].name == "base_link" and description.links[0].parent == ""
    assert description.kind == RobotKind.MULTIROTOR and len(description.joint_names) == 4
    assert "base_link" in description.yaml

    robot.set_joint_velocities([100.0] * 4)
    state, _ = robot.get_state()
    assert state.joint_velocities == [100.0] * 4 and len(state.joint_positions) == 4
    with pytest.raises(FlightForgeError):
        robot.set_joint_velocities([1.0] * 5)
    robot.set_joint_velocities([])


def test_what_the_server_refuses_surfaces_its_message(sim, robot):
    with pytest.raises(FlightForgeError, match="joint_positions"):
        robot.set_joint_positions([0.1])

    links = len(robot.description().links)
    with pytest.raises(FlightForgeError) as refused:
        robot.set_state(RobotState(link_poses=[Pose.at(0.0, 0.0, 2.0)] * links))
    assert str(refused.value)

    with pytest.raises(FlightForgeError) as refused:
        sim.spawn_robot((0.0, 0.0, 2.0), description_yaml="name: inline\nlinks:\n  - name: base_link\n")
    assert str(refused.value)

    with pytest.raises(FlightForgeError) as refused:
        robot.add_sensor(SensorType.RGB_CAMERA, link=robot.description().links[1].name)
    assert str(refused.value)


def test_scene_and_robot_share_the_ground_frame(sim):
    """An object and a robot placed with the same coordinates are in the same place: park the robot a known
    distance short of a post and let the lidar say how far away it really is."""
    sim.clear_objects()
    sim.spawn_objects("objects:\n  - asset: /Engine/BasicShapes/Cylinder\n    position: [10.0, 0.0, 2.0]\n    scale: [0.5, 0.5, 4.0]\n")

    stand_off = 6.0
    robot = sim.spawn_robot((10.0 - stand_off, 0.0, 2.0))
    try:
        assert np.allclose(robot.get_position(), [4.0, 0.0, 2.0], atol=0.01)

        sensor_id = robot.add_sensor(SensorType.LIDAR)
        livox(robot, sensor_id, mount=Pose.at(0.0, 0.0, 0.6))

        time.sleep(1.0)
        robot.lidar_scan(sensor_id=sensor_id)  # drain the slice spanning the reconfigure
        time.sleep(0.3)  # a Livox scan holds what accumulated since the previous one
        xyz = robot.lidar_scan(sensor_id=sensor_id).points
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        forward = xyz[(np.abs(np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))) < 4.0) & (np.abs(xyz[:, 2]) < 1.0)]
        assert len(forward) > 0, "no returns towards where the post should be"

        # the post's near face sits one radius closer than its centre
        assert np.linalg.norm(forward[:, :2], axis=1).min() == pytest.approx(stand_off - 0.25, abs=0.3)
    finally:
        sim.clear_objects()
        sim.remove_robot(robot)


def test_conventions(sim, robot):
    """A cube at a known ground-frame pose, seen through lidar, rangefinder and camera reprojection; a mirrored
    axis or a flipped yaw lands it somewhere else. The robot faces +y (+90 deg yaw) with C 4 m ahead, D 3 m left."""
    sim.clear_objects()
    sim.spawn_objects(
        "objects:\n"
        "  - asset: /Engine/BasicShapes/Cube\n    position: [0.0, 2.0, 3.0]\n"
        "  - asset: /Engine/BasicShapes/Cube\n    position: [-3.0, -2.0, 3.0]\n"
    )
    pose = Pose.at(0.0, -2.0, 3.0, math.pi / 2)
    try:
        robot.set_pose(pose)
        assert np.allclose(robot.get_position(), pose.position, atol=0.01)
        assert np.allclose(robot.get_pose().rotate([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-6)

        lidar = robot.add_sensor(SensorType.LIDAR)
        scan = robot.lidar_scan(sensor_id=lidar)
        assert scan.rows > 1 and scan.cols > 1 and len(scan.points) == scan.rows * scan.cols
        assert np.allclose(scan.sensor_pose.position, pose.position, atol=0.01)
        assert scan.sensor_pose.yaw == pytest.approx(math.pi / 2, abs=1e-3)

        p = scan.points[scan.hits]
        ahead = p[(np.abs(p[:, 1]) < 0.25) & (np.abs(p[:, 2]) < 0.25) & (p[:, 0] > 0)][:, 0].min()
        left = p[(np.abs(p[:, 0]) < 0.25) & (np.abs(p[:, 2]) < 0.25) & (p[:, 1] > 0)][:, 1].min()
        assert ahead == pytest.approx(3.5, abs=0.05)
        assert left == pytest.approx(2.5, abs=0.05)

        # back in the ground frame C's near face is at y = 1.5
        ground = scan.in_ground()[scan.hits]
        assert ground[(np.abs(ground[:, 0]) < 0.25) & (np.abs(ground[:, 2] - 3.0) < 0.25) & (ground[:, 1] > -2.0)][:, 1].min() == pytest.approx(1.5, abs=0.05)

        # organised: row 0 is the top beam
        rows = scan.organized()
        seen = np.isfinite(rows).all(axis=2).any(axis=1)
        with np.errstate(invalid="ignore"):
            elevation = np.nanmean(rows[seen, :, 2] / np.linalg.norm(rows[seen], axis=2), axis=1)
        assert elevation[0] > elevation[-1]
        robot.remove_sensor(lidar)

        # the rangefinder hangs 10 cm under the base and looks down onto C's top 3.5 m below the base
        robot.set_pose(Pose.at(0.0, 2.0, 6.0))
        distance, _ = robot.rangefinder()
        assert distance == pytest.approx(2.4, abs=0.05)

        # E at body (5, 1.5, 0.5): left of and above the image centre, its near face 4.5 m ahead of the base
        sim.clear_objects()
        robot.set_pose(pose)
        sim.spawn_objects("objects:\n  - asset: /Engine/BasicShapes/Cube\n    position: [-1.5, 3.0, 3.5]\n")
        camera = robot.add_sensor(SensorType.DEPTH_CAMERA)
        robot.depth(camera)
        depth, _ = robot.depth(camera)
        (info,) = robot.camera_info(camera)
        mount = info["mount"]
        assert abs(mount.orientation[3]) == pytest.approx(1.0)

        # optical frame: z = mount x, x = -mount y, y = -mount z
        target = np.array([5.0, 1.5, 0.5]) - mount.position
        K = info["K"]
        u = K[0, 0] * (-target[1] / target[0]) + K[0, 2]
        v = K[1, 1] * (-target[2] / target[0]) + K[1, 2]
        assert 0 <= u < K[0, 2] and 0 <= v < K[1, 2]
        seen = depth[int(round(v)), int(round(u))] / 1000.0
        assert seen == pytest.approx(4.5 - mount.position[0], abs=0.1)
        robot.remove_sensor(camera)

        sim.clear_objects()
        sim.spawn_objects("objects:\n  - asset: /Engine/BasicShapes/Cube\n    position: [0.0, 2.0, 3.0]\n")
        vertices, _, _, _ = sim.world_mesh(center=(0.0, 2.0, 3.0), extent=(0.6, 0.6, 0.6))
        corners = np.array([[dx, 2.0 + dy, 3.0 + dz] for dx in (-0.5, 0.5) for dy in (-0.5, 0.5) for dz in (-0.5, 0.5)])
        assert all(np.linalg.norm(vertices - c, axis=1).min() < 0.01 for c in corners)
        distances = sim.distance_batch([[0.0, 2.0, 4.5], [1.5, 2.0, 3.0], [0.0, 3.5, 3.0]])
        assert np.allclose(distances, 1.0, atol=0.02)
    finally:
        sim.clear_objects()


# //}

# time //{


def test_time_dilation_changes_how_fast_the_world_runs(sim):
    """Simulated time must advance at the requested fraction of wall time."""
    try:
        for factor in (0.5, 2.0, 1.0):
            applied = sim.set_time_dilation(factor)
            assert abs(applied - factor) < 1e-3, f"asked {factor}, engine applied {applied}"

            time.sleep(0.3)
            t0_sim, dilation = sim.sim_time()
            assert abs(dilation - factor) < 1e-3

            t0_wall = time.time()
            time.sleep(1.0)
            t1_sim, _ = sim.sim_time()

            rate = (t1_sim - t0_sim) / (time.time() - t0_wall)
            assert abs(rate - factor) < 0.2 * factor, f"at dilation {factor} the world ran at {rate:.3f}x"
    finally:
        sim.set_time_dilation(1.0)


def test_event_camera_in_lockstep(sim, robot):
    """A yaw sweep handed over as a trajectory and stepped 200 ms yields a sorted stream inside the stepped window."""
    sensor_id = robot.add_sensor(SensorType.EVENT_CAMERA)
    robot.set_event_camera_config(sensor_id=sensor_id, width=320, height=240)
    try:
        assert sim.set_fixed_timestep(0.001) == pytest.approx(0.001)
        sim.set_lockstep(True)
        start, _ = sim.step(20)
        robot.event_batch(sensor_id)

        base = robot.get_pose()
        ts = start + np.linspace(0.0, 0.2, 21)
        rows = [[t, *base.position, *Pose.at(0, 0, 0, math.radians(100.0) * (t - start)).orientation] for t in ts]
        robot.set_trajectory(np.array(rows))

        end, _ = sim.step(200)
        assert end - start == pytest.approx(0.2, abs=2e-3)

        batch = robot.event_batch(sensor_id)
        assert batch.width == 320 and batch.height == 240
        assert len(batch) > 1000 and batch.dropped == 0
        assert batch.base_stamp >= start - 1e-9 and batch.stamp <= end + 1e-9
        assert np.all(np.diff(batch.t_us.astype(np.int64)) >= 0)
        assert batch.x.max() < 320 and batch.y.max() < 240
    finally:
        sim.set_lockstep(False)
        sim.set_fixed_timestep(0.0)
        robot.set_trajectory([])
        robot.remove_sensor(sensor_id)


# //}

# connections //{


def test_robots_on_their_own_connections_overlap(sim, robot):
    failures = []

    def reader(index):
        with Robot(sim, robot.id, own_connection=True) as own:
            for _ in range(10):
                try:
                    own.get_state()
                    own.lidar_scan() if index % 2 else own.rgb()
                    own.rangefinder()
                except FlightForgeError as error:
                    failures.append(error)

    threads = [threading.Thread(target=reader, args=(i,)) for i in range(6)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert not failures


def test_shared_connection_from_many_threads(sim, robot):
    failures = []

    def reader():
        for _ in range(20):
            try:
                robot.get_state()
                sim.sim_time()
            except FlightForgeError as error:
                failures.append(error)

    threads = [threading.Thread(target=reader) for _ in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert not failures


def test_gym_envs_build_through_make(sim):
    """Both environments must survive gymnasium.make(), whose PassiveEnvChecker validates observation_space at
    construction, not merely be registered."""
    gymnasium = pytest.importorskip("gymnasium")
    pytest.importorskip("flightforge.gym_envs")

    for env_id in ("FlightForge/CameraNav-v0", "FlightForge/LidarNav-v0"):
        env = gymnasium.make(env_id, simulator=sim, max_steps=4)
        try:
            assert isinstance(env.observation_space, gymnasium.spaces.Space), env_id

            observation, _ = env.reset()
            assert observation.shape == env.observation_space.shape, env_id

            observation, reward, terminated, truncated, info = env.step(env.action_space.sample())
            assert np.isfinite(observation).all(), env_id
            assert np.isfinite(reward), env_id
        finally:
            env.close()


@pytest.mark.skipif(os.environ.get("FF_ALLOW_WORLD_SWITCH") is None, reason="switches the loaded map; set FF_ALLOW_WORLD_SWITCH=1 to run")
def test_world_switch_keeps_the_server_answering(sim):
    """A map carrying its own GameModeOverride used to leave the server unbound after the switch."""
    target = "/Game/FirstPerson/Lvl_FirstPerson"
    sim.switch_world(target)

    deadline = time.time() + 120.0
    while time.time() < deadline:
        try:
            with Simulator(port=sim.connection.port) as probe:
                if probe.fps() > 0:
                    break
        except FlightForgeError:
            pass
        time.sleep(2.0)
    else:
        pytest.fail(f"the server never answered again after switching to {target}")

    with Simulator(port=sim.connection.port) as after:
        robot = after.spawn_robot((0.0, 0.0, 3.0))
        after.remove_robot(robot)


# //}
