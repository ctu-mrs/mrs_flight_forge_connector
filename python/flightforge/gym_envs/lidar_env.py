import numpy as np
from gymnasium import spaces

from .base_env import FlightForgeNavEnvBase


class FlightForgeLidarEnv(FlightForgeNavEnvBase):
    """Goal navigation from lidar ranges.

    The observation is the fixed-size vector of beam ranges normalised to [0, 1] by the lidar's range (no return
    reads as 1), followed by the unit direction to the goal in the ground frame.
    """

    def __init__(self, beam_count=None, beam_range=None, **kwargs):
        super().__init__(**kwargs)

        self._beam_count = beam_count
        self._beam_range = beam_range

        # gymnasium.make() validates observation_space at construction, so it cannot wait for the first reset
        if self._beam_count is None or self._beam_range is None:
            self._ensure_robot()
            self._read_lidar_geometry()

        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(self._beam_count + 3,), dtype=np.float32)

    def _read_lidar_geometry(self):
        config = self._robot.lidar_config()
        if self._beam_count is None:
            self._beam_count = int(config.horizontal_rays * config.vertical_rays)
        if self._beam_range is None:
            self._beam_range = float(config.range_m)

    def _observe(self):
        ranges = self._robot.lidar_scan().ranges()

        observation = np.ones(self._beam_count, dtype=np.float32)
        n = min(ranges.size, self._beam_count)
        if n > 0:
            seen = ranges[:n]
            observation[:n] = np.where(np.isfinite(seen), np.clip(seen / self._beam_range, 0.0, 1.0), 1.0)

        to_goal = self._goal - self._robot.get_position()
        norm = np.linalg.norm(to_goal)
        direction = (to_goal / norm if norm > 0 else to_goal).astype(np.float32)

        return np.concatenate([observation, direction])
