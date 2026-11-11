"""Wire tables as Python objects with the schema's snake_case field names."""

import importlib
import re

import numpy as np

from .geometry import Pose
from .wire.Pose import PoseT
from .wire.Quat import QuatT
from .wire.Vec3 import Vec3T
from .wire.Vec3f import Vec3fT


def camel(name):
    head, *rest = name.split("_")
    return head + "".join(word[:1].upper() + word[1:] for word in rest)


def snake(name):
    return re.sub(r"([A-Z])", r"_\1", name).lower()


def vec3_to_wire(v):
    out = Vec3T()
    out.x, out.y, out.z = (float(c) for c in np.asarray(v, dtype=np.float64).reshape(3))
    return out


def pose_to_wire(pose):
    out = PoseT()
    out.position = vec3_to_wire(pose.position)
    out.orientation = QuatT()
    out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = (float(c) for c in pose.orientation)
    return out


def pose_from_wire(pose):
    """An absent Pose is the identity."""
    if pose is None:
        return Pose()
    p, q = pose.position, pose.orientation
    return Pose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def vec3_from_wire(v):
    return np.zeros(3) if v is None else np.array([v.x, v.y, v.z], dtype=np.float64)


def from_wire(value):
    if isinstance(value, bytes):
        return value.decode("utf-8", "replace")
    if isinstance(value, PoseT):
        return pose_from_wire(value)
    if isinstance(value, (Vec3T, Vec3fT)):
        return vec3_from_wire(value)
    if isinstance(value, QuatT):
        return np.array([value.x, value.y, value.z, value.w], dtype=np.float64)
    if isinstance(value, list):
        return [from_wire(v) for v in value]
    if hasattr(value, "Pack") and hasattr(value, "_UnPack"):
        return Message(value)
    return value


def to_wire(value, current):
    if isinstance(value, Message):
        return value._table
    if isinstance(value, Pose):
        return pose_to_wire(value)
    if isinstance(current, (Vec3T, Vec3fT)):
        return vec3_to_wire(value)
    if isinstance(value, np.generic):
        return value.item()
    return value


class Message:
    """A wire table: read and assign fields by their schema names, e.g. ``config.intrinsics.use_custom = True``.

    Poses read as :class:`Pose` and vectors as numpy arrays; assign a new Pose rather than editing one in place.
    """

    __slots__ = ("_table",)

    def __init__(self, table):
        object.__setattr__(self, "_table", table)

    @property
    def type_name(self):
        return type(self._table).__name__[:-1]

    def fields(self):
        return [snake(name) for name in vars(self._table)]

    def _key(self, name):
        key = camel(name)
        if name.startswith("_") or key not in vars(self._table):
            raise AttributeError(f"{self.type_name} has no field {name!r}")
        return key

    def __getattr__(self, name):
        return from_wire(getattr(self._table, self._key(name)))

    def __setattr__(self, name, value):
        key = self._key(name)
        current = getattr(self._table, key)
        if isinstance(value, dict):
            if not hasattr(current, "_UnPack"):
                raise AttributeError(f"{self.type_name}.{name} is absent; assign message({name!r}...) instead of a dict")
            Message(current).update(**value)
            return
        setattr(self._table, key, to_wire(value, current))

    __getitem__ = __getattr__
    __setitem__ = __setattr__

    def __contains__(self, name):
        return camel(name) in vars(self._table)

    def __dir__(self):
        return self.fields() + ["fields", "to_dict", "type_name", "update"]

    def update(self, **fields):
        """Assigns each field; a dict value updates a nested table field by field."""
        for name, value in fields.items():
            setattr(self, name, value)
        return self

    def to_dict(self):
        def plain(value):
            if isinstance(value, Message):
                return value.to_dict()
            if isinstance(value, list):
                return [plain(v) for v in value]
            return value

        return {name: plain(getattr(self, name)) for name in self.fields()}

    def __repr__(self):
        inner = ", ".join(f"{name}={getattr(self, name)!r}" for name in self.fields())
        return f"{self.type_name}({inner})"


def message(type_name, **fields):
    """A new wire table by schema name, e.g. ``message("ImageEncoding", compression=Compression.NONE)``."""
    module = importlib.import_module(f"flightforge.wire.{type_name}")
    return Message(getattr(module, f"{type_name}T")()).update(**fields)


def table_of(value, type_name):
    """The generated object behind a Message, checked against the expected table."""
    if not isinstance(value, Message) or value.type_name != type_name:
        raise TypeError(f"expected a {type_name}, got {type(value).__name__}")
    return value._table
