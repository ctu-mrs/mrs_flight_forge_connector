"""Poses in the ground frame: right-handed, x forward, y left, z up, metres, unit quaternions (x, y, z, w)."""

import math

import numpy as np


def quat_from_rpy(roll=0.0, pitch=0.0, yaw=0.0):
    """rad, applied yaw * pitch * roll about the parent's z, y, x; returns (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]
    )


def rpy_from_quat(q):
    """(roll, pitch, yaw) in rad from (x, y, z, w), the inverse of quat_from_rpy."""
    x, y, z, w = (float(v) for v in q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def quat_multiply(a, b):
    """a * b: rotate by b, then by a."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]
    )


def quat_conjugate(q):
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)


def quat_rotate(q, vectors):
    """Rotates a (3,) or (N, 3) array by q."""
    v = np.asarray(vectors, dtype=np.float64)
    axis = np.asarray(q[:3], dtype=np.float64)
    t = 2.0 * np.cross(axis, v)
    return v + q[3] * t + np.cross(axis, t)


class Pose:
    """A frame's position and orientation in its parent; the ground frame unless a field says otherwise."""

    __slots__ = ("position", "orientation")

    def __init__(self, position=(0.0, 0.0, 0.0), orientation=(0.0, 0.0, 0.0, 1.0)):
        self.position = np.array(position, dtype=np.float64).reshape(3)
        orientation = np.array(orientation, dtype=np.float64).reshape(-1)
        if orientation.size != 4:
            raise ValueError("orientation is a quaternion (x, y, z, w); build one from roll, pitch, yaw with quat_from_rpy")
        self.orientation = orientation

    @classmethod
    def at(cls, x, y, z, yaw=0.0):
        """yaw in rad about +z, 0 facing +x."""
        return cls((x, y, z), quat_from_rpy(0.0, 0.0, yaw))

    @classmethod
    def from_rpy(cls, position, roll=0.0, pitch=0.0, yaw=0.0):
        return cls(position, quat_from_rpy(roll, pitch, yaw))

    @property
    def rpy(self):
        return rpy_from_quat(self.orientation)

    @property
    def yaw(self):
        return float(self.rpy[2])

    def rotate(self, vectors):
        return quat_rotate(self.orientation, vectors)

    def transform(self, points):
        """Points in this frame into the parent frame, (3,) or (N, 3)."""
        return self.rotate(points) + self.position

    def inverse(self):
        conjugate = quat_conjugate(self.orientation)
        return Pose(-quat_rotate(conjugate, self.position), conjugate)

    def __mul__(self, other):
        return Pose(self.transform(other.position), quat_multiply(self.orientation, other.orientation))

    def __repr__(self):
        p = ", ".join(f"{v:.4g}" for v in self.position)
        q = ", ".join(f"{v:.4g}" for v in self.orientation)
        return f"Pose(position=({p}), orientation=({q}))"


def as_pose(position_or_pose, orientation=None):
    """A Pose from a Pose, or from a position and an optional quaternion."""
    if isinstance(position_or_pose, Pose):
        if orientation is not None:
            raise ValueError("pass either a Pose or a position with an orientation, not both")
        return position_or_pose
    return Pose(position_or_pose, (0.0, 0.0, 0.0, 1.0) if orientation is None else orientation)
