"""MuJoCo owns the dynamics, FlightForge renders.

The bridge steps a MuJoCo model in-process and pushes the tracked body's pose
into the simulator with the fire-and-forget pose call each step, so FlightForge
acts as a photorealistic renderer + sensor suite over MuJoCo physics — no
dynamics code in the simulator, any MJCF quadrotor (or JAX/MJX rollout that
fills qpos) works.

Frames: MuJoCo is right-handed, Z-up, meters; Unreal is left-handed, Z-up,
centimeters. The conversion flips Y and the yaw sense and scales to cm.

    import mujoco
    from flightforge import Simulator, MujocoBridge

    sim = Simulator()
    drone = sim.spawn_drone((0, 0, 200), "x500")

    bridge = MujocoBridge(drone, "quadrotor.xml", body="drone",
                          world_origin_uu=sim.world_origin())
    for _ in range(10_000):
        bridge.data.ctrl[:] = controller(bridge.data)
        bridge.step()                      # mj_step + pose push
        image, stamp = drone.rgb()         # rendered by FlightForge
"""

import math

import numpy as np

M_TO_UU = 100.0  # meters -> Unreal units (cm)


def mujoco_to_unreal_position(pos_m, world_origin_uu=(0.0, 0.0, 0.0)):
    """Right-handed meters -> left-handed Unreal centimeters (Y flipped)."""
    return np.array(
        [
            world_origin_uu[0] + pos_m[0] * M_TO_UU,
            world_origin_uu[1] - pos_m[1] * M_TO_UU,
            world_origin_uu[2] + pos_m[2] * M_TO_UU,
        ]
    )


def quat_wxyz_to_unreal_euler(quat):
    """MuJoCo body quaternion (w, x, y, z) -> Unreal (pitch, yaw, roll) degrees.

    Extrinsic ZYX decomposition with the handedness flip folded into the yaw
    and roll signs.
    """
    w, x, y, z = quat

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    deg = 180.0 / math.pi
    return np.array([pitch * deg, -yaw * deg, roll * deg])


class MujocoBridge:

    def __init__(self, drone, model, body=None, world_origin_uu=(0.0, 0.0, 0.0), substeps=1):
        """
        drone: a connected :class:`flightforge.Drone`
        model: MJCF path or an existing ``mujoco.MjModel``
        body:  name of the body to mirror (default: the model's first free-joint body)
        world_origin_uu: Unreal-units offset added to every pushed position
                         (e.g. ``Simulator.world_origin()``)
        substeps: physics steps per :meth:`step` call
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
        self._drone = drone
        self._world_origin_uu = np.asarray(world_origin_uu, dtype=np.float64)
        self._substeps = int(substeps)

    @property
    def sim_time(self):
        return self.data.time

    def step(self, push_pose=True):
        """Advances physics by ``substeps`` and mirrors the pose to FlightForge."""
        for _ in range(self._substeps):
            self._mujoco.mj_step(self.model, self.data)

        if push_pose:
            self.push_pose()

    def push_pose(self):
        position = mujoco_to_unreal_position(self.data.xpos[self._body_id], self._world_origin_uu)
        orientation = quat_wxyz_to_unreal_euler(self.data.xquat[self._body_id])
        self._drone.set_pose_async(position, orientation)

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
