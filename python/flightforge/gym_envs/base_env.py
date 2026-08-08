"""Shared machinery for the FlightForge navigation environments.

The task: fly from a start pose to a goal position in the running simulator.
Actions are normalized position/yaw steps applied by teleporting with
collision checking, so the environments work without any dynamics model; for
dynamics-in-the-loop training combine :class:`flightforge.MujocoBridge` with a
custom environment instead.

Coordinates are Unreal units (centimeters, left-handed) as used by the rest of
the simulator API.
"""

import gymnasium
import numpy as np
from gymnasium import spaces

from ..client import Simulator


class FlightForgeNavEnvBase(gymnasium.Env):

    metadata = {"render_modes": ["rgb_array"], "render_fps": 30}

    def __init__(
        self,
        address="127.0.0.1",
        port=8551,
        simulator=None,
        drone_frame="x500",
        start_position=(0.0, 0.0, 200.0),
        goal_position=(2000.0, 0.0, 200.0),
        goal_threshold=100.0,
        action_step=50.0,
        yaw_step=10.0,
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

        self._drone_frame = drone_frame
        self._start = np.asarray(start_position, dtype=np.float64)
        self._goal = np.asarray(goal_position, dtype=np.float64)
        self._goal_threshold = float(goal_threshold)
        self._action_step = float(action_step)
        self._yaw_step = float(yaw_step)
        self._max_steps = int(max_steps)
        self._collision_penalty = float(collision_penalty)
        self._goal_reward = float(goal_reward)

        self._drone = None
        self._steps = 0
        self._yaw = 0.0
        self._last_distance = None

        # dx, dy, dz, dyaw in [-1, 1], scaled by action_step / yaw_step
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)

    # subclasses provide observation_space and _observe()

    def _observe(self):
        raise NotImplementedError

    def _ensure_drone(self):
        if self._drone is None:
            self._drone = self._sim.spawn_drone(self._start, self._drone_frame)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        self._ensure_drone()
        self._yaw = 0.0
        self._steps = 0
        self._drone.set_pose(self._start, (0.0, self._yaw, 0.0), check_collisions=False)
        self._last_distance = float(np.linalg.norm(self._goal - self._start))

        return self._observe(), {"distance": self._last_distance}

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)

        position = self._drone.get_position()
        target = position + action[:3] * self._action_step
        self._yaw = (self._yaw + action[3] * self._yaw_step) % 360.0

        hit, _ = self._drone.set_pose(target, (0.0, self._yaw, 0.0), check_collisions=True)

        position = self._drone.get_position()
        distance = float(np.linalg.norm(self._goal - position))

        # progress toward the goal, plus terminal bonuses/penalties
        reward = self._last_distance - distance
        self._last_distance = distance

        crashed = hit or self._drone.crashed()
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
            image, _ = self._drone.rgb()
            return image
        return None

    def close(self):
        if self._drone is not None:
            self._sim.remove_drone(self._drone)
            self._drone.close()
            self._drone = None
        if self._owns_sim:
            self._sim.close()
