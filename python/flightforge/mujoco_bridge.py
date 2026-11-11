"""MuJoCo owns the dynamics, FlightForge renders.

The bridge steps a MuJoCo model in-process and pushes the tracked body's pose to the simulator with the
fire-and-forget pose call each step. MuJoCo's world frame is right-handed, z up and in metres like the ground
frame, so the only conversion is the quaternion order and where MuJoCo's origin sits in the ground frame.

    from flightforge import MujocoBridge, Simulator

    sim = Simulator()
    robot = sim.spawn_robot((0.0, 0.0, 2.0), "x500")

    bridge = MujocoBridge(robot, "quadrotor.xml", body="drone")
    for _ in range(10_000):
        bridge.data.ctrl[:] = controller(bridge.data)
        bridge.step()
        image, stamp = robot.rgb()
"""

import numpy as np

from .geometry import Pose


def quat_wxyz_to_xyzw(quat):
    w, x, y, z = quat
    return np.array([x, y, z, w], dtype=np.float64)


def mujoco_pose(xpos, xquat, origin=(0.0, 0.0, 0.0)):
    """A MuJoCo body's world position and (w, x, y, z) quaternion as a ground-frame Pose; origin is MuJoCo's origin in m."""
    return Pose(np.asarray(xpos, dtype=np.float64) + np.asarray(origin, dtype=np.float64), quat_wxyz_to_xyzw(xquat))


class MujocoBridge:
    def __init__(self, robot, model, body=None, origin=(0.0, 0.0, 0.0), substeps=1):
        """robot: a flightforge.Robot; model: an MJCF path or a mujoco.MjModel; body: the body to mirror, default the
        first free-joint body; origin: where MuJoCo's origin sits in the ground frame, m; substeps: physics steps per step().
        """
        import mujoco

        self._mujoco = mujoco

        self.model = model if isinstance(model, mujoco.MjModel) else mujoco.MjModel.from_xml_path(model)
        self.data = mujoco.MjData(self.model)

        if body is None:
            free_joints = [j for j in range(self.model.njnt) if self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
            if not free_joints:
                raise ValueError("the model has no free joint; pass body= explicitly")
            body_id = self.model.jnt_bodyid[free_joints[0]]
        else:
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
            if body_id < 0:
                raise ValueError(f"body {body!r} not found in the model")

        self._body_id = body_id
        self._robot = robot
        self._origin = np.asarray(origin, dtype=np.float64)
        self._substeps = int(substeps)

    @property
    def sim_time(self):
        return self.data.time

    def pose(self):
        return mujoco_pose(self.data.xpos[self._body_id], self.data.xquat[self._body_id], self._origin)

    def step(self, push_pose=True):
        """Advances physics by substeps and mirrors the pose to FlightForge."""
        for _ in range(self._substeps):
            self._mujoco.mj_step(self.model, self.data)

        if push_pose:
            self.push_pose()

    def push_pose(self):
        self._robot.set_pose_async(self.pose())

    def reset(self, keyframe=None):
        if keyframe is None:
            self._mujoco.mj_resetData(self.model, self.data)
        else:
            key_id = self._mujoco.mj_name2id(self.model, self._mujoco.mjtObj.mjOBJ_KEY, keyframe)
            if key_id < 0:
                raise ValueError(f"keyframe {keyframe!r} not found")
            self._mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)

        self._mujoco.mj_forward(self.model, self.data)
        self.push_pose()
