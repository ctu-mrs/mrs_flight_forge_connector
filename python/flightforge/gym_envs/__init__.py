"""Gymnasium environments for FlightForge.

Registered ids (after ``import flightforge.gym_envs`` or automatically via the
``gymnasium.envs`` entry point):

  - ``FlightForge/CameraNav-v0`` — RGB observation, continuous position-step
    actions, goal-reaching navigation
  - ``FlightForge/LidarNav-v0`` — lidar-distance observation, same task
"""


def register_envs():
    import gymnasium

    gymnasium.register(
        id="FlightForge/CameraNav-v0",
        entry_point="flightforge.gym_envs.camera_env:FlightForgeCameraEnv",
    )
    gymnasium.register(
        id="FlightForge/LidarNav-v0",
        entry_point="flightforge.gym_envs.lidar_env:FlightForgeLidarEnv",
    )


try:
    register_envs()
except ImportError:  # gymnasium not installed
    pass
