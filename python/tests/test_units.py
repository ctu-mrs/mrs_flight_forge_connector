"""Unit tests that run anywhere - no simulator, no editor."""

import math

import numpy as np
import pytest


def test_mujoco_frame_conversion():
    from flightforge.mujoco_bridge import mujoco_to_unreal_position, quat_wxyz_to_unreal_euler

    p = mujoco_to_unreal_position([1.0, 2.0, 3.0], (100.0, 200.0, 300.0))
    assert np.allclose(p, [200.0, 0.0, 600.0])

    assert np.allclose(quat_wxyz_to_unreal_euler([1, 0, 0, 0]), [0.0, 0.0, 0.0])

    yaw90 = [math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)]
    assert np.allclose(quat_wxyz_to_unreal_euler(yaw90), [0.0, -90.0, 0.0], atol=1e-6)


def test_instance_id_decode():
    # the client decodes 24-bit ids little-endian from RGB
    rgb = np.zeros((2, 2, 3), dtype=np.uint32)
    rgb[0, 0] = (0x2A, 0x00, 0x00)  # 42
    rgb[0, 1] = (0x00, 0x01, 0x00)  # 256
    rgb[1, 0] = (0x01, 0x01, 0x01)  # 65793
    ids = rgb[:, :, 0] + (rgb[:, :, 1] << 8) + (rgb[:, :, 2] << 16)
    assert ids[0, 0] == 42 and ids[0, 1] == 256 and ids[1, 0] == 65793 and ids[1, 1] == 0


def test_decode_image_roundtrip():
    cv2 = pytest.importorskip("cv2")
    from flightforge.client import _decode_image

    original = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    ok, png = cv2.imencode(".png", cv2.cvtColor(original, cv2.COLOR_RGB2BGR))
    assert ok

    decoded = _decode_image(png.tobytes())
    assert np.array_equal(decoded, original)


def test_gym_registration():
    gymnasium = pytest.importorskip("gymnasium")
    pytest.importorskip("flightforge.gym_envs")

    assert gymnasium.spec("FlightForge/CameraNav-v0") is not None
    assert gymnasium.spec("FlightForge/LidarNav-v0") is not None


def test_binding_defaults():
    d = pytest.importorskip("flight_forge_drone")

    intrinsics = d.CameraIntrinsics()
    assert not intrinsics.use_custom and intrinsics.fx == 0.0

    distortion = d.CameraDistortion()
    assert not distortion.enable and distortion.k1 == 0.0

    noise = d.CameraNoise()
    assert not noise.enable and noise.shot_scale == pytest.approx(2.0e-4)

    rolling_shutter = d.CameraRollingShutter()
    assert not rolling_shutter.enable and rolling_shutter.readout_time == pytest.approx(0.03)

    assert int(d.SensorType.INSTANCE_SEG_CAMERA) == 8

    config = d.RgbCameraConfig()
    config.intrinsics.fx = 500.0
    config.distortion.k1 = -0.2
    config.noise.enable = True
    config.rolling_shutter.readout_time = 0.02


def test_stereo_config_property_roundtrip():
    d = pytest.importorskip("flight_forge_drone")

    config = d.StereoCameraConfig()

    intrinsics = d.CameraIntrinsics()
    intrinsics.use_custom = True
    intrinsics.fx = 420.0
    config.intrinsics = intrinsics
    assert config.intrinsics.use_custom and config.intrinsics.fx == 420.0

    distortion = d.CameraDistortion()
    distortion.enable = True
    distortion.p2 = 0.001
    config.distortion = distortion
    assert config.distortion.enable and config.distortion.p2 == pytest.approx(0.001)
