import numpy as np
from gymnasium import spaces

from .base_env import FlightForgeNavEnvBase


class FlightForgeCameraEnv(FlightForgeNavEnvBase):
    """Goal navigation from RGB camera observations."""

    def __init__(self, image_width=640, image_height=480, **kwargs):
        super().__init__(**kwargs)

        self._image_shape = (image_height, image_width, 3)
        self.observation_space = spaces.Box(low=0, high=255, shape=self._image_shape, dtype=np.uint8)

    def _observe(self):
        image, _ = self._robot.rgb()

        if image.shape != self._image_shape:
            try:
                import cv2

                image = cv2.resize(np.ascontiguousarray(image), (self._image_shape[1], self._image_shape[0]))
            except ImportError:
                raise RuntimeError(
                    f"camera returned {image.shape}, expected {self._image_shape}; "
                    f"configure the camera resolution or install opencv-python for resizing"
                ) from None

        return np.ascontiguousarray(image, dtype=np.uint8)
