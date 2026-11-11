# flightforge: Python client for FlightForge, API 1.0

A pure-Python client for the FlightForge simulator, plus [Gymnasium](https://gymnasium.farama.org)
environments and a MuJoCo dynamics bridge. It speaks the FlatBuffers wire over one TCP port and needs
only `numpy` and `flatbuffers`; nothing is compiled.

## Install

```bash
pip install ./python               # the client
pip install "./python[all]"        # plus gymnasium, mujoco and opencv-python; or [gym], [mujoco], [vision]
```

JPEG and PNG frames decode with `opencv-python` (or Pillow). Without either, set a camera's encoding to
`Compression.NONE` and frames arrive raw.

## Frame and units

- One frame, the **ground frame**: right-handed, x forward, y left, z up, origin on the floor under the level's `UAV` PlayerStart.
- Metres, seconds, rad/s; orientations are unit quaternions `(x, y, z, w)`, built with `quat_from_rpy` or `Pose.at(x, y, z, yaw)`.
- Depth images are uint16 millimetres; field-of-view angles stay in degrees.

## Example

```python
import math
from flightforge import Pose, SensorType, Simulator

sim = Simulator()                                   # 127.0.0.1, FLIGHTFORGE_PORT or 8600
robot = sim.spawn_robot(Pose.at(0.0, 0.0, 2.0, yaw=math.pi / 2), "x500", label="uav1")

image, stamp = robot.rgb()                          # (H, W, 3) uint8 RGB
depth, stamp = robot.depth()                        # (H, W) uint16 mm
scan = robot.lidar_scan(labels=True)                # scan.points: (N, 3) float32 m, sensor frame, NaN no return
points = scan.in_ground()                           # the same points in the ground frame
robot.set_pose(Pose.at(5.0, 0.0, 2.0), collide=True)

camera = robot.add_sensor(SensorType.DEPTH_CAMERA, Pose.at(0.1, 0.0, -0.05))
robot.set_rgb_camera_config(sensor_id=camera, width=640, height=480)

sim.set_fixed_timestep(1e-3)
sim.set_lockstep(True)
sim.step(200)                                       # returns once every camera has its frames
sim.remove_robot(robot)
```

Every call raises `FlightForgeError` with the simulator's message when a request fails. Lidar points and raw
frames are numpy views into the received message, not copies.

`Simulator` owns one connection, which its robots share and which is safe to use from several threads.
`sim.spawn_robot(..., own_connection=True)` or `Robot(sim, robot_id, own_connection=True)` gives a robot its
own, so its requests run alongside the others'; the simulator serves each connection on its own thread.

Configs are wire tables with the schema's field names:

```python
config = robot.lidar_config()
config.range_m = 40.0
config.mount = Pose.at(0.0, 0.0, 0.1)
robot.set_lidar_config(config)
robot.set_rgb_camera_config(intrinsics={"use_custom": True, "fx": 500.0, "fy": 500.0, "cx": 319.5, "cy": 239.5})
```

## Gymnasium

`FlightForge/CameraNav-v0` and `FlightForge/LidarNav-v0`, registered through the `gymnasium.envs` entry point.
Both teleport with collision checks, so they need no dynamics model.

```python
import gymnasium
env = gymnasium.make("FlightForge/CameraNav-v0", render_mode="rgb_array")
observation, info = env.reset()
observation, reward, terminated, truncated, info = env.step(env.action_space.sample())
```

## MuJoCo

MuJoCo integrates, FlightForge renders. MuJoCo's world is right-handed, z up and in metres, so `origin` is all
that relates it to the ground frame.

```python
from flightforge import MujocoBridge

bridge = MujocoBridge(robot, "quadrotor.xml", body="drone", origin=(0.0, 0.0, 0.0))
bridge.data.ctrl[:] = thrusts
bridge.step()                                       # mj_step, then a fire-and-forget pose push
```

`python/examples/` has runnable scripts: calibrated cameras, an event camera in lockstep, instance
segmentation, a reprojection check, MuJoCo hover, parallel robots and the scene-editor loop.

## Tests

```bash
cd python && python3 -m pytest -q
```

The offline tests run a fake simulator on a local port. The live tests (`-m sim`) need a simulator on
`FLIGHTFORGE_PORT` and skip without one; `FF_ALLOW_WORLD_SWITCH=1` adds the one that switches the map.
