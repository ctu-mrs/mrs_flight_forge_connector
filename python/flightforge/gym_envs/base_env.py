"""Shared machinery for the FlightForge navigation environments.

The task: fly from a start position to a goal in the running simulator. Actions are normalised position and yaw
steps applied by teleporting with collision checking, so the environments need no dynamics model; for dynamics in
the loop combine flightforge.MujocoBridge with a custom environment. Positions are ground-frame metres, yaw in rad.
"""

import gymnasium
import numpy as np
from gymnasium import spaces

from ..client import Simulator
from ..geometry import Pose


class FlightForgeNavEnvBase(gymnasium.Env):

    metadata = {"render_modes": ["rgb_array"], "render_fps": 30}

    def __init__(
        self,
        address="127.0.0.1",
        port=None,
        simulator=None,
        robot_name="x500",
        start_position=(0.0, 0.0, 2.0),
        goal_position=(20.0, 0.0, 2.0),
        goal_threshold=1.0,
        action_step=0.5,
        yaw_step=0.175,
        max_steps=500,
        collision_penalty=100.0,
        goal_reward=100.0,
        render_mode=None,
    ):
        if render_mode is not None and render_mode not in self.metadata["render_modes"]:
            raise ValueError(f"unsupported render_mode {render_mode!r}")

        self.render_mode = render_mode

        self._sim = simulator if simulator is not None else Simulator(address, port)
        self._owns_sim = simulator is None

        self._robot_name = robot_name
        self._start = np.asarray(start_position, dtype=np.float64)
        self._goal = np.asarray(goal_position, dtype=np.float64)
        self._goal_threshold = float(goal_threshold)
        self._action_step = float(action_step)
        self._yaw_step = float(yaw_step)
        self._max_steps = int(max_steps)
        self._collision_penalty = float(collision_penalty)
        self._goal_reward = float(goal_reward)

        self._robot = None
        self._steps = 0
        self._yaw = 0.0
        self._last_distance = None

        # dx, dy, dz, dyaw in [-1, 1], scaled by action_step (m) and yaw_step (rad)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)

    def _observe(self):
        raise NotImplementedError

    def _ensure_robot(self):
        if self._robot is None:
            self._robot = self._sim.spawn_robot(self._start, self._robot_name)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        self._ensure_robot()
        self._yaw = 0.0
        self._steps = 0
        self._robot.set_pose(Pose.at(*self._start, yaw=self._yaw))
        self._last_distance = float(np.linalg.norm(self._goal - self._start))

        return self._observe(), {"distance": self._last_distance}

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)

        position = self._robot.get_position()
        target = position + action[:3] * self._action_step
        self._yaw = float(np.remainder(self._yaw + action[3] * self._yaw_step + np.pi, 2.0 * np.pi) - np.pi)

        hit = self._robot.set_pose(Pose.at(*target, yaw=self._yaw), collide=True).hit

        position = self._robot.get_position()
        distance = float(np.linalg.norm(self._goal - position))

        reward = self._last_distance - distance
        self._last_distance = distance

        crashed = bool(hit or self._robot.crashed())
        reached = distance < self._goal_threshold

        if crashed:
            reward -= self._collision_penalty
        if reached:
            reward += self._goal_reward

        self._steps += 1
        terminated = bool(crashed or reached)
        truncated = bool(self._steps >= self._max_steps)

        info = {"distance": distance, "crashed": crashed, "reached": reached}
        return self._observe(), float(reward), terminated, truncated, info

    def render(self):
        if self.render_mode == "rgb_array":
            image, _ = self._robot.rgb()
            return np.ascontiguousarray(image)
        return None

    def close(self):
        if self._robot is not None:
            self._sim.remove_robot(self._robot)
            self._robot.close()
            self._robot = None
        if self._owns_sim:
            self._sim.close()
