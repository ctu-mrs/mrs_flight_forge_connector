"""A local stand-in for the simulator's port: framed FlatBuffers in, scripted replies out."""

import socket
import struct
import threading

import flatbuffers

from flightforge.wire.GetInfoResponse import GetInfoResponseT
from flightforge.wire.Request import Request, RequestT
from flightforge.wire.RequestBody import RequestBody
from flightforge.wire.ResponseBody import ResponseBody
from flightforge.wire.Response import ResponseT

HEADER = struct.Struct("<IB3x")
REQUEST_NAMES = {value: name for name, value in vars(RequestBody).items() if not name.startswith("_")}


def response_frame(seq, body=None, ok=True, error=""):
    """A whole frame, header included, carrying a Response with body (a generated ...T object)."""
    envelope = ResponseT()
    envelope.seq = seq
    envelope.ok = ok
    envelope.error = error
    if body is not None:
        envelope.bodyType = getattr(ResponseBody, type(body).__name__[:-1])
        envelope.body = body
    builder = flatbuffers.Builder(1024)
    builder.Finish(envelope.Pack(builder))
    payload = bytes(builder.Output())
    return HEADER.pack(len(payload), 0) + payload


class Call:
    def __init__(self, request):
        self.seq = request.Seq()
        self.robot_id = request.RobotId()
        self.sensor_id = request.SensorId()
        self.name = REQUEST_NAMES[request.BodyType()]
        self.body = RequestT.InitFromObj(request).body


class FakeSimulator:
    """handler(call) returns a list of frames to send, a ...T body, or None for an empty ok reply."""

    def __init__(self, handler=None):
        self.handler = handler or (lambda call: None)
        self.calls = []
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(("127.0.0.1", 0))
        self._server.listen()
        self.port = self._server.getsockname()[1]
        self._connections = []
        self._stop = False
        threading.Thread(target=self._accept, daemon=True).start()

    def close(self):
        self._stop = True
        self._server.close()
        for connection in self._connections:
            try:
                connection.shutdown(socket.SHUT_RDWR)
                connection.close()
            except OSError:
                pass

    def drop_clients(self):
        for connection in self._connections:
            try:
                connection.shutdown(socket.SHUT_RDWR)
                connection.close()
            except OSError:
                pass
        self._connections.clear()

    def _accept(self):
        while not self._stop:
            try:
                connection, _ = self._server.accept()
            except OSError:
                return
            self._connections.append(connection)
            threading.Thread(target=self._serve, args=(connection,), daemon=True).start()

    @staticmethod
    def _read(connection, size):
        data = bytearray()
        while len(data) < size:
            chunk = connection.recv(size - len(data))
            if not chunk:
                raise ConnectionError
            data += chunk
        return bytes(data)

    def _serve(self, connection):
        try:
            while True:
                length, _ = HEADER.unpack(self._read(connection, HEADER.size))
                call = Call(Request.GetRootAs(self._read(connection, length), 0))
                self.calls.append(call)
                for frame in self._reply(call):
                    connection.sendall(frame)
        except (ConnectionError, OSError):
            pass

    def _reply(self, call):
        result = self.handler(call)
        if result is None and call.name == "GetInfoRequest":
            info = GetInfoResponseT()
            info.apiMajor, info.apiMinor, info.schema = 1, 0, [1, 2, 3]
            result = info
        if isinstance(result, list):
            return result
        return [response_frame(call.seq, result)]
