"""The generated 1.0 wire bindings: messages built and read back in pure Python, no simulator."""

import math

import numpy as np
import pytest

flatbuffers = pytest.importorskip("flatbuffers")

from flightforge.wire.Compression import Compression
from flightforge.wire.Image import ImageT
from flightforge.wire.PixelFormat import PixelFormat
from flightforge.wire.Pose import PoseT
from flightforge.wire.Quat import QuatT
from flightforge.wire.Request import Request, RequestT
from flightforge.wire.RequestBody import RequestBody
from flightforge.wire.Response import Response, ResponseT
from flightforge.wire.ResponseBody import ResponseBody
from flightforge.wire.RgbCameraDataResponse import RgbCameraDataResponse, RgbCameraDataResponseT
from flightforge.wire.RobotState import RobotStateT
from flightforge.wire.RobotStateEntry import RobotStateEntryT
from flightforge.wire.SetStatesRequest import SetStatesRequest, SetStatesRequestT
from flightforge.wire.Vec3 import Vec3T


def pose(x, y, z, yaw):
    p = PoseT()
    p.position = Vec3T()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation = QuatT()
    p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def finish(table):
    builder = flatbuffers.Builder(1024)
    builder.Finish(table.Pack(builder))
    return builder.Output()


def test_set_states_round_trip():
    body = SetStatesRequestT()
    body.collide = True
    body.states = []
    for robot_id in range(3):
        entry = RobotStateEntryT()
        entry.robotId = robot_id
        entry.state = RobotStateT()
        entry.state.base = pose(robot_id, -robot_id, 1.5, 0.5 * robot_id)
        entry.state.jointVelocities = [900.0, -900.0]
        body.states.append(entry)

    envelope = RequestT()
    envelope.seq = 42
    envelope.robotId = None
    envelope.bodyType = RequestBody.SetStatesRequest
    envelope.body = body

    request = Request.GetRootAs(finish(envelope))
    assert request.Seq() == 42
    assert request.RobotId() is None and request.SensorId() is None
    assert request.BodyType() == RequestBody.SetStatesRequest

    states = SetStatesRequest()
    states.Init(request.Body().Bytes, request.Body().Pos)
    assert states.Collide() and states.WaitApplied()
    assert states.StatesLength() == 3
    second = RobotStateT.InitFromObj(states.States(2).State())
    assert (second.base.position.x, second.base.position.y, second.base.position.z) == (2.0, -2.0, 1.5)
    assert second.base.orientation.z == pytest.approx(math.sin(0.5))
    assert list(second.jointVelocities) == [900.0, -900.0]


def test_camera_payload_reads_into_numpy_without_copy():
    width, height = 320, 240
    pixels = (np.arange(width * height * 3) % 251).astype(np.uint8)

    frame = RgbCameraDataResponseT()
    frame.stamp = 7.25
    frame.image = ImageT()
    frame.image.width, frame.image.height = width, height
    frame.image.format = PixelFormat.RGB8
    frame.image.compression = Compression.NONE
    frame.image.data = pixels

    envelope = ResponseT()
    envelope.seq = 5
    envelope.ok = True
    envelope.bodyType = ResponseBody.RgbCameraDataResponse
    envelope.body = frame

    buffer = bytearray(finish(envelope))
    response = Response.GetRootAs(buffer)
    assert response.Ok() and response.Seq() == 5 and response.BodyType() == ResponseBody.RgbCameraDataResponse

    data = RgbCameraDataResponse()
    data.Init(response.Body().Bytes, response.Body().Pos)
    image = data.Image()
    assert data.Stamp() == 7.25 and image.Width() == width and image.Format() == PixelFormat.RGB8

    view = image.DataAsNumpy().reshape(height, width, 3)
    assert np.array_equal(view.ravel(), pixels)
    assert np.shares_memory(view, np.frombuffer(buffer, dtype=np.uint8))

    offset = image.DataAsNumpy().ctypes.data - np.frombuffer(buffer, dtype=np.uint8).ctypes.data
    buffer[offset] ^= 0xFF
    assert view[0, 0, 0] == pixels[0] ^ 0xFF
