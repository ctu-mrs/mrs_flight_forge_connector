"""Integration tests against a live simulator (skip cleanly without one).

Run with:  pytest -m sim          (only these)
           pytest                 (everything; these skip if no sim)

This suite doubles as the runtime validation harness for the API 0.14 work:
cameras and calibration, the scene editor round-trip, mutual visibility and
instance segmentation.
"""

import numpy as np
import pytest

pytestmark = pytest.mark.sim

RING_YAML = """
objects:
  - asset: Cube
    position: [5.0, 0.0, 1.0]
  - asset: Cube
    position: [5.0, 2.0, 1.0]
    scale: 0.5
"""


def test_api_version(sim):
    assert sim.api_version >= (0, 14)


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
