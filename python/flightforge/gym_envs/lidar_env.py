import numpy as np
from gymnasium import spaces

from ..client import FlightForgeError
from .base_env import FlightForgeNavEnvBase


class FlightForgeLidarEnv(FlightForgeNavEnvBase):
    """Goal navigation from lidar distances.

    The observation is the fixed-size vector of beam distances normalized to
    [0, 1] by the beam length (no-return beams read as 1), concatenated with
    the normalized goal direction.
    """

    def __init__(self, beam_count=None, beam_length=None, **kwargs):
        super().__init__(**kwargs)

        self._beam_count = beam_count
        self._beam_length = beam_length

        # The space has to exist before the first reset: gymnasium.make() wraps the env in
        # PassiveEnvChecker, which validates observation_space at construction. Deferring it
        # to reset() makes the env unusable through make() - the only path most callers use.
        # Pass beam_count and beam_length to skip talking to the simulator here.
        if self._beam_count is None or self._beam_length is None:
            self._ensure_drone()
            self._read_lidar_geometry()

        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(self._beam_count + 3,), dtype=np.float32)

    def _read_lidar_geometry(self):
        ok_config = self._drone.raw.GetLidarConfig()
        if not ok_config[0]:
            raise FlightForgeError("GetLidarConfig failed")
        config = ok_config[1]

        if self._beam_count is None:
            self._beam_count = int(config.BeamHorRays * config.BeamVertRays)
        if self._beam_length is None:
            self._beam_length = float(config.beamLength)

    def _observe(self):
        points, _, _ = self._drone.lidar()

        distances = np.full(self._beam_count, 1.0, dtype=np.float32)
        n = min(len(points), self._beam_count)
        if n > 0:
            raw = points[:n, 0]
            normalized = np.where(raw < 0, 1.0, np.clip(raw / self._beam_length, 0.0, 1.0))
            distances[:n] = normalized.astype(np.float32)

        to_goal = self._goal - self._drone.get_position()
        norm = np.linalg.norm(to_goal)
        direction = (to_goal / norm if norm > 0 else to_goal).astype(np.float32)

        return np.concatenate([distances, direction])
