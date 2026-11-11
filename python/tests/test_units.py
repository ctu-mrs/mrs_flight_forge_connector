"""Unit tests that run anywhere: no simulator, no editor."""

import importlib
import math
import pkgutil

import numpy as np
import pytest

import flightforge
from flightforge import Pose, quat_from_rpy, quat_multiply, rpy_from_quat


def test_mujoco_pose_is_the_ground_frame():
    from flightforge.mujoco_bridge import mujoco_pose

    pose = mujoco_pose([1.0, 2.0, 3.0], [1.0, 0.0, 0.0, 0.0], origin=(10.0, 0.0, 0.5))
    assert np.allclose(pose.position, [11.0, 2.0, 3.5])
    assert np.allclose(pose.orientation, [0.0, 0.0, 0.0, 1.0])

    yaw90 = [math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)]
    assert mujoco_pose([0, 0, 0], yaw90).yaw == pytest.approx(math.pi / 2)


def test_plus_ninety_degrees_of_yaw_faces_plus_y():
    pose = Pose.at(0.0, 0.0, 0.0, math.pi / 2)
    assert np.allclose(pose.rotate([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0])
    assert np.allclose(pose.rotate([0.0, 1.0, 0.0]), [-1.0, 0.0, 0.0])


def test_positive_pitch_puts_the_nose_down():
    assert np.allclose(Pose.from_rpy((0, 0, 0), pitch=math.pi / 2).rotate([1.0, 0.0, 0.0]), [0.0, 0.0, -1.0])


def test_rpy_round_trip():
    for rpy in [(0.1, -0.2, 0.3), (-1.0, 0.5, 3.0), (0.0, 0.0, -math.pi / 2)]:
        q = quat_from_rpy(*rpy)
        assert np.linalg.norm(q) == pytest.approx(1.0)
        assert np.allclose(rpy_from_quat(q), rpy)


def test_pose_composition_and_inverse():
    a = Pose.at(1.0, 2.0, 0.5, math.pi / 2)
    b = Pose.from_rpy((3.0, 0.0, 0.0), roll=0.2)
    composed = a * b
    point = np.array([0.3, -0.4, 1.2])
    assert np.allclose(composed.transform(point), a.transform(b.transform(point)))
    assert np.allclose((a.inverse() * a).position, 0.0)
    assert np.allclose(quat_multiply(a.inverse().orientation, a.orientation), [0, 0, 0, 1])
    assert np.allclose(a.transform(np.array([[4.0, 0.0, 0.0], [0.0, 0.0, 1.0]])), [[1.0, 6.0, 0.5], [1.0, 2.0, 1.5]])


def test_instance_id_decode():
    # the client decodes 24-bit ids little-endian from RGB
    rgb = np.zeros((2, 2, 3), dtype=np.uint32)
    rgb[0, 0] = (0x2A, 0x00, 0x00)
    rgb[0, 1] = (0x00, 0x01, 0x00)
    rgb[1, 0] = (0x01, 0x01, 0x01)
    ids = rgb[:, :, 0] + (rgb[:, :, 1] << 8) + (rgb[:, :, 2] << 16)
    assert ids[0, 0] == 42 and ids[0, 1] == 256 and ids[1, 0] == 65793 and ids[1, 1] == 0


def test_decode_image_roundtrip():
    cv2 = pytest.importorskip("cv2")

    original = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    ok, png = cv2.imencode(".png", cv2.cvtColor(original, cv2.COLOR_RGB2BGR))
    assert ok

    frame = flightforge.Frame(0.0, 3, 2, flightforge.PixelFormat.RGB8, flightforge.Compression.PNG, png.ravel())
    assert np.array_equal(frame.decode(), original)
    assert frame.decode(keep_alpha=True).shape == (2, 3, 4)


def test_raw_formats_decode_to_rgb():
    bgr = np.array([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8)
    frame = flightforge.Frame(0.0, 2, 1, flightforge.PixelFormat.BGR8, flightforge.Compression.NONE, bgr.ravel())
    assert np.array_equal(frame.decode(), bgr[:, :, ::-1])
    assert np.array_equal(frame.decode(keep_alpha=True)[..., 3], [[255, 255]])

    wrong = flightforge.Frame(0.0, 3, 1, flightforge.PixelFormat.BGR8, flightforge.Compression.NONE, bgr.ravel())
    with pytest.raises(flightforge.FlightForgeError):
        wrong.decode()


def test_gym_registration():
    gymnasium = pytest.importorskip("gymnasium")
    pytest.importorskip("flightforge.gym_envs")

    assert gymnasium.spec("FlightForge/CameraNav-v0") is not None
    assert gymnasium.spec("FlightForge/LidarNav-v0") is not None


def test_config_defaults():
    from flightforge import message

    intrinsics = message("CameraIntrinsics")
    assert not intrinsics.use_custom and intrinsics.fx == 0.0

    distortion = message("CameraDistortion")
    assert not distortion.enable and distortion.k1 == 0.0

    noise = message("CameraNoise")
    assert not noise.enable and noise.shot_scale == pytest.approx(2.0e-4)

    rolling_shutter = message("CameraRollingShutter")
    assert not rolling_shutter.enable and rolling_shutter.readout_time == pytest.approx(0.03)

    assert flightforge.SensorType.INSTANCE_SEG_CAMERA == 8

    encoding = message("ImageEncoding")
    assert encoding.compression == flightforge.Compression.JPEG and encoding.quality == 90

    config = message("RgbCameraConfig", intrinsics=intrinsics, distortion=distortion, noise=noise, rolling_shutter=rolling_shutter)
    config.intrinsics.fx = 500.0
    config.distortion.k1 = -0.2
    config.noise.enable = True
    config.rolling_shutter.readout_time = 0.02
    config.update(intrinsics={"cy": 240.5})
    assert config.intrinsics.fx == 500.0 and config.intrinsics.cy == 240.5 and config["noise"].enable

    with pytest.raises(AttributeError):
        config.beam_length = 1.0


def test_stereo_config_property_roundtrip():
    from flightforge import message

    config = message("StereoCameraConfig")
    config.intrinsics = message("CameraIntrinsics", use_custom=True, fx=420.0)
    assert config.intrinsics.use_custom and config.intrinsics.fx == 420.0

    config.distortion = message("CameraDistortion", enable=True, p2=0.001)
    assert config.distortion.enable and config.distortion.p2 == pytest.approx(0.001)

    config.mount_right = Pose.at(0.0, -0.06, 0.0)
    assert isinstance(config.mount_right, Pose) and config.mount_right.position[1] == pytest.approx(-0.06)
    assert config.mount_left is None


def test_every_schema_field_has_its_snake_case_name():
    import flightforge.wire as wire
    from flightforge.messages import Message, camel, snake

    checked = 0
    for info in pkgutil.iter_modules(wire.__path__):
        module = importlib.import_module(f"flightforge.wire.{info.name}")
        object_class = getattr(module, f"{info.name}T", None)
        if object_class is None:
            continue
        message = Message(object_class())
        for name in message.fields():
            assert camel(name) in vars(message._table) and snake(camel(name)) == name
            getattr(message, name)
            checked += 1
    assert checked > 300


def test_event_batch_decode():
    # the wire packs x | y << 16 | polarity << 31 and microseconds after the base stamp
    from flightforge import EventBatch

    xyp = np.array([5 | (7 << 16) | (1 << 31), 1279 | (719 << 16)], dtype=np.uint32)
    t_us = np.array([0, 1500], dtype=np.uint32)

    batch = EventBatch.unpack(xyp, t_us, base_stamp=12.25, stamp=12.3, dropped=0, width=1280, height=720)

    assert list(batch.x) == [5, 1279] and list(batch.y) == [7, 719]
    assert list(batch.polarity) == [1, -1]
    assert np.allclose(batch.t, [12.25, 12.2515])
    assert len(batch) == 2

    image = batch.accumulate()
    assert image.shape == (720, 1280) and image[7, 5] == 1 and image[719, 1279] == -1


def test_no_native_module_is_needed():
    import sys

    assert not any(name.startswith(("flight_forge_drone", "flight_forge_game_mode")) for name in sys.modules)
    assert not hasattr(flightforge, "_native")
