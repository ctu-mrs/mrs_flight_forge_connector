"""One framed TCP connection to the simulator: a FlatBuffers Request out, its Response back."""

import os
import socket
import struct
import threading
import time

import flatbuffers

from .wire import Request as _request
from .wire.Response import Response
from .wire.ResponseBody import ResponseBody

DEFAULT_PORT = 8600
FRAME_HEADER = struct.Struct("<IB3x")
MAX_FRAME_BYTES = 1 << 30


class FlightForgeError(RuntimeError):
    pass


class FlightForgeConnectionError(FlightForgeError, ConnectionError):
    pass


def default_port():
    """FLIGHTFORGE_PORT when set, else 8600."""
    return int(os.environ.get("FLIGHTFORGE_PORT", DEFAULT_PORT))


def encode_frame_header(length, flags=0):
    return FRAME_HEADER.pack(length, flags)


def finish_request(builder, seq, body_type, body, robot_id=None, sensor_id=None):
    """Finishes a Request around an already built body and returns its bytes."""
    _request.RequestStart(builder)
    _request.RequestAddSeq(builder, seq)
    if robot_id is not None:
        _request.RequestAddRobotId(builder, int(robot_id))
    if sensor_id is not None:
        _request.RequestAddSensorId(builder, int(sensor_id))
    _request.RequestAddBodyType(builder, body_type)
    if body is not None:
        _request.RequestAddBody(builder, body)
    builder.Finish(_request.RequestEnd(builder))
    return builder.Output()


class Reply:
    """A received Response; ``frame`` owns the bytes every view into the reply points at."""

    __slots__ = ("frame", "response")

    def __init__(self, frame, response):
        self.frame = frame
        self.response = response

    def body(self, table_class):
        if self.response.BodyType() != getattr(ResponseBody, table_class.__name__):
            raise FlightForgeError(f"the reply carries no {table_class.__name__}")
        union = self.response.Body()
        table = table_class()
        table.Init(union.Bytes, union.Pos)
        return table


class Connection:
    """Calls take turns under a lock; replies match requests by seq, so one arriving after its caller gave up is skipped."""

    def __init__(self, address="127.0.0.1", port=None, timeout=10.0):
        self.address = address
        self.port = default_port() if port is None else int(port)
        self.timeout = float(timeout)  # s

        self._lock = threading.Lock()
        self._socket = None
        self._next_seq = 1
        self._header = bytearray(FRAME_HEADER.size)
        self._header_got = 0
        self._body = None
        self._body_got = 0

    def __repr__(self):
        return f"Connection({self.address}:{self.port})"

    # lifetime //{

    def connect(self, attempts=1, retry_delay=0.5):
        """Tries ``attempts`` times, ``retry_delay`` seconds apart; raises FlightForgeConnectionError."""
        error = None
        for attempt in range(max(int(attempts), 1)):
            with self._lock:
                if self._socket is not None:
                    return self
                try:
                    self._open()
                    return self
                except OSError as e:
                    error = e
            if attempt + 1 < attempts:
                time.sleep(retry_delay)
        raise FlightForgeConnectionError(f"cannot connect to {self.address}:{self.port}: {error}")

    def close(self):
        with self._lock:
            self._close()

    @property
    def connected(self):
        return self._socket is not None

    def _open(self):
        sock = socket.create_connection((self.address, self.port), timeout=self.timeout)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        self._socket = sock
        self._reset_reader()

    def _close(self):
        if self._socket is not None:
            try:
                self._socket.close()
            finally:
                self._socket = None
        self._reset_reader()

    def _reset_reader(self):
        self._header_got = 0
        self._body = None
        self._body_got = 0

    # //}

    # calls //{

    def call(self, body_type, build_body=None, robot_id=None, sensor_id=None, timeout=None):
        """Sends one request and returns its Reply; raises FlightForgeError with the server's message on failure.

        ``build_body(builder)`` returns the body's offset, or None for an empty body.
        """
        builder = flatbuffers.Builder(256)
        body = build_body(builder) if build_body is not None else None
        with self._lock:
            seq = self._allocate_seq()
            frame = finish_request(builder, seq, body_type, body, robot_id, sensor_id)
            reply = self._exchange(frame, seq, self.timeout if timeout is None else float(timeout))
        if not reply.response.Ok():
            error = reply.response.Error()
            raise FlightForgeError(error.decode("utf-8", "replace") if error else "the request failed")
        return reply

    def call_raw(self, frame, expect_seq, timeout=None):
        """Sends an already finished frame body, for tests that send what a client never would; no ok check."""
        with self._lock:
            return self._exchange(bytes(frame), expect_seq, self.timeout if timeout is None else float(timeout))

    def _allocate_seq(self):
        # 0 is what the simulator answers when it could not read the seq
        if self._next_seq == 0:
            self._next_seq = 1
        seq = self._next_seq
        self._next_seq = (self._next_seq + 1) & 0xFFFFFFFF
        return seq

    def _exchange(self, frame, seq, timeout):
        if self._socket is None:
            try:
                self._open()
            except OSError as error:
                raise FlightForgeConnectionError(f"not connected to {self.address}:{self.port}: {error}") from None

        try:
            self._socket.sendall(encode_frame_header(len(frame)) + frame)
        except OSError as error:
            self._close()
            raise FlightForgeConnectionError(f"the request could not be sent: {error}") from None

        deadline = time.monotonic() + timeout
        while True:
            buffer = self._read_frame(deadline)
            try:
                response = Response.GetRootAs(buffer, 0)
                reply_seq = response.Seq()
            except Exception:
                raise FlightForgeError("the simulator sent a reply that is not a flightforge.wire.Response") from None
            # a reply to a request that already timed out
            if reply_seq != seq:
                continue
            return Reply(buffer, response)

    def _read_frame(self, deadline):
        """One whole frame into a fresh buffer; a timeout keeps what arrived so the stream stays in step."""
        while self._header_got < FRAME_HEADER.size:
            self._header_got += self._receive(memoryview(self._header)[self._header_got :], deadline)

        if self._body is None:
            length, _ = FRAME_HEADER.unpack(self._header)
            if length > MAX_FRAME_BYTES:
                self._close()
                raise FlightForgeError(f"the simulator sent a {length} byte frame")
            self._body = bytearray(length)
            self._body_got = 0

        view = memoryview(self._body)
        while self._body_got < len(self._body):
            self._body_got += self._receive(view[self._body_got :], deadline)

        body = self._body
        self._reset_reader()
        return body

    def _receive(self, view, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise FlightForgeError("timed out waiting for the reply")
        self._socket.settimeout(remaining)
        try:
            got = self._socket.recv_into(view)
        except socket.timeout:
            raise FlightForgeError("timed out waiting for the reply") from None
        except OSError as error:
            self._close()
            raise FlightForgeConnectionError(f"the connection failed: {error}") from None
        if got == 0:
            self._close()
            raise FlightForgeConnectionError("the simulator closed the connection")
        return got

    # //}
