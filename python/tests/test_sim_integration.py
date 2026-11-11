"""Integration tests against a live simulator (skip cleanly without one).

Run with:  pytest -m sim          (only these)
           pytest                 (everything; these skip if no sim)

This suite doubles as the runtime validation harness for the API 0.14 work:
cameras and calibration, the scene editor round-trip, mutual visibility and
instance segmentation.
"""

import os
import time

import numpy as np
import pytest

from flightforge.client import Simulator

pytestmark = pytest.mark.sim

# An /Engine path resolves without search_paths and ships with every install, so the
# test does not depend on what content the host project happens to have.
RING_YAML = """
objects:
  - asset: /Engine/BasicShapes/Cube
    position: [5.0, 0.0, 1.0]
  - asset: /Engine/BasicShapes/Cube
    position: [5.0, 2.0, 1.0]
    scale: 0.5
"""


def test_api_version(sim):
    assert sim.api_version >= (0, 14)


def test_camera_info(drone):
    assert drone.camera_info() == []  # nothing exists until a camera is touched
    K, size = drone.camera_intrinsics()
    infos = drone.camera_info()
    rgb = [c for c in infos if c["sensor_id"] == 0][0]
    assert rgb["projection"] == "pinhole" and rgb["distortion_model"] == "plumb_bob"
    assert rgb["size"] == size
    assert np.allclose(rgb["K"], K)
    assert rgb["K"][0, 2] == pytest.approx(size[0] / 2.0 - 0.5)  # OpenCV pixel convention
    assert rgb["D"].shape == (5,) and not rgb["D"].any()
    assert rgb["readout_time"] == 0.0
    assert drone.camera_info(0)[0]["sensor_id"] == 0


def test_fisheye_calibration_round_trip(drone):
    sensor_id = drone.add_sensor(7)  # fisheye
    k = (-0.012, 0.021, -0.017, 0.004)
    drone.set_fisheye_calibration(254.3, 252.9, 405.7, 291.2, k=k, fov_deg=190.0, sensor_id=sensor_id)
    (info,) = drone.camera_info(sensor_id)
    assert info["projection"] == "fisheye_kannala_brandt" and info["distortion_model"] == "equidistant"
    assert np.allclose(info["K"], [[254.3, 0.0, 405.7], [0.0, 252.9, 291.2], [0.0, 0.0, 1.0]])
    assert np.allclose(info["D"], k)
    assert info["fov_deg"] == 190.0
    drone.remove_sensor(sensor_id)


def test_meta_describes_the_wire(sim):
    meta = sim.meta()
    assert tuple(meta["api"]) == sim.api_version
    ops = {op["name"]: op for op in meta["ops"]}
    assert "get_rgb_camera_data" in ops and "step" in ops
    fields = [f["name"] for f in ops["get_rgb_camera_data"]["response"]["fields"]]
    assert fields == ["type", "status", "seq", "image_", "stamp_"]
    assert sim.check_compatibility() == []
    assert sim.check_compatibility(["get_fps", "spawn_drone_at_location"]) == []


def test_fps_reports(sim):
    assert sim.fps() > 0.0


def test_rgb_capture(drone):
    image, stamp = drone.rgb()
    assert image.ndim == 3 and image.shape[2] == 3
    assert stamp > 0.0


def test_intrinsics_roundtrip(drone):
    K0, (width, height) = drone.camera_intrinsics()
    assert K0[0, 0] > 0.0 and K0[0, 2] == pytest.approx(width / 2.0, abs=1.0)

    drone.set_camera_intrinsics(fx=500.0, fy=505.0, cx=300.0, cy=250.0)
    K1, _ = drone.camera_intrinsics()
    assert np.allclose([K1[0, 0], K1[1, 1], K1[0, 2], K1[1, 2]], [500.0, 505.0, 300.0, 250.0])

    image, _ = drone.rgb()
    assert image.shape[:2] == (height, width)


def test_distortion_roundtrip(drone):
    drone.set_camera_distortion(k1=-0.2, k2=0.05, p1=0.001)
    enabled, coefficients = drone.camera_distortion()
    assert enabled
    assert np.allclose(coefficients[:2], [-0.2, 0.05]) and coefficients[3] == pytest.approx(0.001)

    # the served image stays at the nominal resolution despite the overscan render
    _, (width, height) = drone.camera_intrinsics()
    image, _ = drone.rgb()
    assert image.shape[:2] == (height, width)

    drone.set_camera_distortion(enable=False)


def test_noise_and_rolling_shutter_config(drone):
    drone.set_camera_noise(shot_scale=3.0e-4, read_sigma=4.0e-3)
    drone.set_rolling_shutter(0.025)

    ok, config = drone.raw.GetRgbCameraConfig()
    assert ok
    assert config.noise.enable and config.noise.shot_scale == pytest.approx(3.0e-4)
    assert config.rolling_shutter.enable and config.rolling_shutter.readout_time == pytest.approx(0.025)

    drone.set_camera_noise(enable=False)
    drone.set_rolling_shutter(0.0, enable=False)


def test_scene_editor_roundtrip(sim):
    ids = sim.spawn_objects(RING_YAML)
    assert len(ids) == 2

    try:
        objects = sim.objects()
        assert set(ids) <= set(objects)
        assert np.allclose(objects[ids[0]]["position"], [5.0, 0.0, 1.0], atol=0.05)

        sim.move_object(ids[0], (6.0, -1.0, 2.0), (0.0, 0.0, 45.0))
        moved = sim.objects()[ids[0]]
        assert np.allclose(moved["position"], [6.0, -1.0, 2.0], atol=0.05)
        assert moved["orientation"][2] == pytest.approx(45.0, abs=0.5)

        exported = sim.export_scene()
        assert "Cube" in exported
        assert "class: prop/box" in exported  # the host project's Cube rule

        sim.clear_objects()
        assert sim.objects() == {}

        restored = sim.spawn_objects(exported)
        assert len(restored) == 2
        assert np.allclose(sim.objects()[restored[0]]["position"], [6.0, -1.0, 2.0], atol=0.05)
    finally:
        sim.clear_objects()


def test_asset_catalog(sim):
    names, truncated = sim.assets(name_filter="cube")
    assert isinstance(names, list)
    assert isinstance(truncated, bool)


def test_mutual_visibility_toggles(sim):
    assert sim.set_mutual_visibility(False)
    assert sim.set_mutual_visibility(True)


def test_sensor_lifecycle(drone):
    from flightforge import SensorType

    sensor_id = drone.add_sensor(SensorType.RGB_CAMERA)
    assert sensor_id >= 0
    assert any(s_id == sensor_id for s_id, _ in drone.sensors())
    assert drone.remove_sensor(sensor_id)


def test_instance_segmentation_smoke(sim, drone):
    ids = sim.spawn_objects(RING_YAML)
    try:
        instance_ids, stamp = drone.instance_segmentation()
        assert instance_ids.dtype == np.uint32 and instance_ids.ndim == 2
        assert stamp > 0.0

        mapping = drone.instance_map()
        visible = set(np.unique(instance_ids)) - {0}
        # every visible id must be explainable
        assert visible <= set(mapping)
    finally:
        sim.clear_objects()


def test_devices_listed(drone):
    devices = drone.devices()
    assert "realsense_d435i" in devices


def test_gym_envs_build_through_make(sim):
    """Both environments must survive gymnasium.make(), not merely be registered.

    make() wraps the env in PassiveEnvChecker, which validates observation_space at
    construction. LidarNav left that space as None until the first reset, so it was
    registered, importable and completely unusable through the standard entry point -
    which is what checking gymnasium.spec() alone failed to notice.
    """
    gymnasium = pytest.importorskip("gymnasium")
    pytest.importorskip("flightforge.gym_envs")

    import flightforge.gym_envs  # noqa: F401 - registers the ids

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


def test_scene_and_drone_frames_are_related_by_world_origin(sim):
    """The two halves of the API do not share a frame, so pin the conversion down.

    Objects are placed in metres relative to the "UAV" PlayerStart; drones are placed in
    absolute world centimetres. An object at [0,0,0] is therefore not where a drone at
    (0,0,0) sits, and code that mixes the two silently puts things in the wrong place.
    This measures the relationship rather than trusting it: park the drone a known
    distance short of a post and let the lidar say how far away it really is.
    """
    import flight_forge_drone as ffd

    sim.clear_objects()
    sim.spawn_objects(
        "objects:\n"
        "  - asset: /Engine/BasicShapes/Cylinder\n"
        "    position: [10.0, 0.0, 2.0]\n"
        "    scale: [0.5, 0.5, 4.0]\n"
    )

    stand_off_m = 6.0
    target = sim.scene_to_world([10.0 - stand_off_m, 0.0, 2.0])

    drone = sim.spawn_robot(tuple(target))
    try:
        drone.set_pose(tuple(target), (0.0, 0.0, 0.0))
        assert np.allclose(drone.get_position(), target, atol=1.0)
        assert np.allclose(sim.world_to_scene(target), [4.0, 0.0, 2.0], atol=1e-6)

        sensor_id = drone.add_sensor(ffd.LIDAR_LIVOX)
        ok, config = drone.raw.GetLidarConfig(sensor_id)
        assert ok
        config.Enable = True
        config.Livox = True
        config.LivoxSensor = "mid360"
        config.beamLength = 7000.0
        config.offset = ffd.Coordinates(0.0, 0.0, 60.0)  # clear of the airframe
        assert drone.raw.SetLidarConfig(config, sensor_id)

        time.sleep(1.0)
        drone.lidar(sensor_id)  # drain the slice spanning the reconfigure
        points, _, _ = drone.lidar(sensor_id)

        hits = points[points[:, 0] > 0]
        assert len(hits) > 0

        unit = hits[:, 1:4] / np.linalg.norm(hits[:, 1:4], axis=1, keepdims=True)
        xyz = hits[:, 0:1] * unit / 100.0

        forward = xyz[
            (np.abs(np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))) < 4.0)
            & (np.abs(xyz[:, 2]) < 1.0)
        ]
        assert len(forward) > 0, "no returns towards where the post should be"

        # the post's near face sits one radius closer than its centre
        nearest = np.linalg.norm(forward[:, :2], axis=1).min()
        assert nearest == pytest.approx(stand_off_m - 0.25, abs=0.3)
    finally:
        sim.clear_objects()
        sim.remove_robot(drone)
        drone.close()


@pytest.mark.skipif(os.environ.get("FF_ALLOW_WORLD_SWITCH") is None,
                    reason="switches the loaded map; set FF_ALLOW_WORLD_SWITCH=1 to run")
def test_world_switch_keeps_the_server_answering(sim):
    """A map carrying its own GameModeOverride used to leave the server unbound.

    The override was applied when travelling to the Entry map and then dropped when
    the destination was opened, so the simulator came up on the new map and answered
    nothing: every map-changing workflow died there.
    """
    target = "/Game/FirstPerson/Lvl_FirstPerson"
    sim.switch_world(target)

    deadline = time.time() + 120.0
    while time.time() < deadline:
        try:
            if Simulator().fps() > 0:
                break
        except Exception:
            pass
        time.sleep(2.0)
    else:
        pytest.fail(f"the server never answered again after switching to {target}")

    after = Simulator()
    assert after.fps() > 0
    drone = after.spawn_robot(tuple(after.world_origin() + np.array([0.0, 0.0, 300.0])))
    after.remove_robot(drone)


def test_lidar_scan_labels_agree_with_the_instance_camera(sim):
    """A point and a pixel on the same object carry the same instance id, and the
    point's class id is the object's segmentation stencil."""
    import flight_forge_drone as ffd

    sim.clear_objects()
    drone = sim.spawn_robot(tuple(sim.scene_to_world([0.0, 0.0, 1.5])))
    try:
        drone.set_pose(sim.scene_to_world([0.0, 0.0, 1.5]), (0.0, 0.0, 0.0))
        camera = drone.add_sensor(ffd.INSTANCE_SEG_CAMERA)
        (cube_id,) = sim.spawn_objects("objects:\n  - asset: /Engine/BasicShapes/Cube\n    position: [4.0, 0.0, 1.5]\n    scale: 2.0\n")
        stencil = sim.objects()[cube_id]["stencil"]
        time.sleep(1.5)
        for _ in range(3):
            drone.instance_segmentation(camera)

        image, classes, _ = drone.panoptic(camera)
        h, w = image.shape
        pixel_id = int(image[h // 2, w // 2])
        pixel_class = int(classes[h // 2, w // 2])
        assert pixel_id != 0

        # the class comes from the tagger: a rule when the project has one for the cube, else the stencil
        tree = sim.class_map()
        by_path = {v["path"]: k for k, v in tree.items()}
        expected_class = by_path.get("prop/box", stencil)
        assert pixel_class == expected_class
        assert drone.instance_classes(camera)[pixel_id] == expected_class

        scan = drone.lidar_scan(labels=True)
        pts = scan["points"]
        assert scan["class_id"].shape == (len(pts),) and scan["instance_id"].shape == (len(pts),)
        assert scan["intensity"].size == 0  # not requested
        d = pts[:, 1:4] / np.linalg.norm(pts[:, 1:4], axis=1, keepdims=True)
        forward = (pts[:, 0] > 0) & (d[:, 0] > 0.995)  # returns within ~6 deg of the optical axis
        assert forward.sum() > 0
        assert set(scan["instance_id"][forward]) == {pixel_id}
        assert set(scan["class_id"][forward]) == {expected_class}
        assert np.all(scan["class_id"][pts[:, 0] < 0] == -1) and np.all(scan["instance_id"][pts[:, 0] < 0] == 0)

        if "prop/box" in by_path:
            # leaf ids collapse through the tree
            assert tree[expected_class]["parent_id"] == by_path["prop"]
            assert sim.collapse_classes(np.array([expected_class]), 1)[0] == by_path["prop"]
    finally:
        sim.clear_objects()
        sim.remove_robot(drone)
        drone.close()


def test_segmentation_covers_rendered_geometry(sim):
    """Every pixel the renderer put geometry into carries an instance id: the depth
    buffer is the oracle, which unlike a physics trace sees masked leaves, wind and
    foliage without collision the same way the id pass must."""
    import flight_forge_drone as ffd

    drone = sim.spawn_robot(tuple(sim.scene_to_world([0.0, 0.0, 3.0])))
    try:
        drone.set_pose(sim.scene_to_world([0.0, 0.0, 3.0]), (-20.0, 0.0, 0.0))
        camera = drone.add_sensor(ffd.INSTANCE_SEG_CAMERA)
        time.sleep(2.0)
        for _ in range(4):
            drone.panoptic(camera)
            drone.depth()
        ids, _, _ = drone.panoptic(camera)
        depth, _ = drone.depth()
        geometry = (depth > 0) & (depth < 65535)
        assert geometry.sum() > 10000
        unlabelled = geometry & (ids == 0)
        assert unlabelled.sum() / geometry.sum() < 0.01
    finally:
        sim.remove_robot(drone)
        drone.close()


def test_instance_segmentation_survives_scene_edits(sim):
    """Destroying actors while an instance-seg camera runs must not kill the simulator.

    The pass used to collect raw scene proxies on the game thread and dereference them
    later on the render thread; anything destroyed in between left it walking freed
    memory. That is exactly what runtime scene editing does, and it segfaulted the sim.
    """
    import flight_forge_drone as ffd

    drone = sim.spawn_robot(tuple(sim.scene_to_world([0.0, 0.0, 3.0])))
    try:
        sensor_id = drone.add_sensor(ffd.INSTANCE_SEG_CAMERA)
        time.sleep(0.5)

        for i in range(8):
            sim.spawn_objects(
                "objects:\n"
                "  - asset: /Game/LevelPrototyping/Meshes/SM_Cube.SM_Cube\n"
                f"    position: [{6 + i}, 0.0, 1.0]\n"
                "    scale: [1.5, 1.5, 1.5]\n"
            )
            drone.instance_segmentation(sensor_id)
            sim.clear_objects()
            drone.instance_segmentation(sensor_id)

        # the simulator answering at all is the assertion; a dangling proxy took it down
        assert sim.fps() > 0
        ids, stamp = drone.instance_segmentation(sensor_id)
        assert ids.shape[0] > 0 and ids.shape[1] > 0
    finally:
        sim.remove_robot(drone)
        sim.clear_objects()


def test_time_dilation_changes_how_fast_the_world_runs(sim):
    """Simulated time must advance at the requested fraction of wall time.

    A consumer that cannot keep up in real time runs the world slow instead of dropping
    data; below 1 the world advances slower than the wall clock, which also buys more
    frames per simulated second for a sensor to be sampled at.
    """
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


def test_sensor_body_occludes_the_airframe_but_not_itself(sim):
    """With a body, a lidar sees the drone it is bolted to and never its own housing.

    Self-occlusion is real and a rig has to be designed around it, but a beam cannot be
    blocked by the aperture it leaves from. Off by default, because a sensor left at the
    frame's origin sits inside the body and would find nothing else.
    """
    import flight_forge_drone as ffd

    drone = sim.spawn_robot(tuple(sim.scene_to_world([0.0, 0.0, 4.0])))
    try:
        sensor_id = drone.add_sensor(ffd.LIDAR)

        def sweep(show_body):
            ok, config = drone.raw.GetLidarConfig(sensor_id)
            config.Enable = True
            config.Livox = False
            config.beamLength = 5000.0
            config.offset = ffd.Coordinates(0.0, 0.0, 50.0)
            # the stock profile stops 7 degrees below horizontal and never looks down
            config.FOVVertDown = 85.0
            config.FOVVertUp = 15.0
            config.showSensorBody = show_body
            assert drone.raw.SetLidarConfig(config, sensor_id)
            time.sleep(0.9)
            drone.lidar(sensor_id)
            time.sleep(0.3)

            points, _, _ = drone.lidar(sensor_id)
            p = np.asarray(points)
            dist = p[:, 0] / 100.0
            hit = dist > 0
            unit = p[:, 1:4] / np.maximum(np.linalg.norm(p[:, 1:4], axis=1, keepdims=True), 1e-9)
            el = np.degrees(np.arcsin(np.clip(unit[:, 2], -1, 1)))

            below = hit & (el < -55) & (dist > 0.2) & (dist < 0.8)
            housing = hit & (dist < 0.15)
            return int(below.sum()), int(housing.sum())

        off_frame, off_housing = sweep(False)
        on_frame, on_housing = sweep(True)

        assert off_frame == 0, "the airframe must stay invisible without a sensor body"
        assert on_frame > 0, "with a body the lidar must see the airframe it is mounted on"
        assert off_housing == 0 and on_housing == 0, "a lidar must never return its own housing"
    finally:
        sim.remove_robot(drone)


def test_world_mesh_is_ground_truth(sim):
    """The mesh export must contain a spawned object at its exact size and place.

    A cube of known scale is the cheapest possible ground truth: if its 24 vertices
    span anything but scale x scale x scale metres at the requested position, the
    export's transforms or frames are wrong.
    """
    import numpy as np

    sim.clear_objects()
    sim.spawn_objects(
        "objects:\n"
        "  - asset: /Game/LevelPrototyping/Meshes/SM_Cube.SM_Cube\n"
        "    position: [15.0, 7.0, 4.0]\n"
        "    scale: [2.0, 2.0, 2.0]\n"
    )
    time.sleep(0.3)

    vertices, faces, truncated, sources = sim.world_mesh(center=(15.0, 7.0, 4.0), extent=(6.0, 6.0, 6.0))
    assert sources >= 1
    assert len(faces) > 0

    near = np.all(np.abs(vertices - np.array([15.0, 7.0, 4.0])) < 3.2, axis=1)
    assert near.sum() >= 8, "the spawned cube is missing from the mesh"

    span = vertices[near].max(axis=0) - vertices[near].min(axis=0)
    assert np.allclose(span, 2.0, atol=0.05), f"cube span {span} is not 2 m"

    sim.clear_objects()


def test_gltf_model_is_visible_to_the_lidar(sim):
    """A runtime-loaded glTF model must have collision, not just a mesh.

    glTFRuntime builds a static mesh with no collision unless asked, so a model spawned
    at runtime rendered perfectly while every ray-traced sensor passed straight through
    it - an obstacle a camera can see and a lidar cannot.
    """
    import os

    import flight_forge_drone as ffd

    models = os.environ.get(
        "FF_MODEL_DIR",
        "/home/david/Documents/Unreal Projects/TestPlugin/Plugins/FlightForgePlugin/Content/ExternalModels",
    )
    if not os.path.isdir(models):
        pytest.skip(f"no glTF models to spawn (set FF_MODEL_DIR; tried {models})")

    sim.clear_objects()
    ok, ids, error = sim.raw.SpawnObjectsFromYaml(
        f"model_paths:\n  - {models}\nobjects:\n"
        "  - asset: drone20\n"
        "    position: [6.0, 0.0, 2.0]\n"
        "    scale: 20.0\n"
    )
    if not ok or not len(ids):
        pytest.skip(f"could not spawn a glTF model: {error}")

    drone = sim.spawn_robot(tuple(sim.scene_to_world([0.0, 0.0, 2.0])))
    try:
        drone.set_pose(tuple(sim.scene_to_world([0.0, 0.0, 2.0])), (0.0, 0.0, 0.0))

        sensor_id = drone.add_sensor(ffd.LIDAR_LIVOX)
        ok, config = drone.raw.GetLidarConfig(sensor_id)
        assert ok
        config.Enable = True
        config.Livox = True
        config.LivoxSensor = "mid360"
        config.beamLength = 7000.0
        config.offset = ffd.Coordinates(0.0, 0.0, 60.0)
        assert drone.raw.SetLidarConfig(config, sensor_id)

        time.sleep(1.2)
        drone.lidar(sensor_id)
        points, _, _ = drone.lidar(sensor_id)

        hits = points[points[:, 0] > 0]
        assert len(hits) > 0

        unit = hits[:, 1:4] / np.linalg.norm(hits[:, 1:4], axis=1, keepdims=True)
        xyz = hits[:, 0:1] * unit / 100.0

        # the model spans roughly 3-9 m ahead at this scale; anything landing there is it
        forward = xyz[
            (np.abs(np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))) < 20.0)
            & (np.linalg.norm(xyz[:, :2], axis=1) < 10.0)
            & (np.abs(xyz[:, 2]) < 4.0)
        ]
        assert len(forward) > 20, "the glTF model returned no lidar hits - collision is missing"
    finally:
        sim.clear_objects()
        sim.remove_robot(drone)
        drone.close()
