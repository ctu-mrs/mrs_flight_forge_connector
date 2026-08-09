# flightforge — Python API

Pythonic client, [Gymnasium](https://gymnasium.farama.org) environments and a
MuJoCo dynamics bridge for the FlightForge simulator (API 0.14).

## Install

Build the native bindings once, then install the package:

```bash
# in the connector checkout
cmake -S . -B build -DBUILD_PYTHON_LIB=ON && cmake --build build -j
pip install -e python[all]        # or [gym], [mujoco], [vision] selectively
```

The package finds the compiled modules in `build/` automatically; override
with `FLIGHTFORGE_NATIVE_PATH` if they live elsewhere.

## Layers

**Raw bindings** — everything the wire protocol offers, 1:1:

```python
from flightforge import DroneController, GameModeController
```

**Pythonic client** — raises on failure, numpy in and out:

```python
from flightforge import Simulator

sim = Simulator()                              # game mode on :8551
drone = sim.spawn_drone((0, 0, 200), "x500")

image, stamp = drone.rgb()                     # HxWx3 uint8
points, origin, stamp = drone.lidar()          # (N,4) distances + directions
events, stamp = drone.events()                 # (N,4) event camera batch
drone.add_device("realsense_d435i", offset=(12, 0, -2))
ids = sim.spawn_objects(open("scene.yaml").read())
```

**Gymnasium** — `FlightForge/CameraNav-v0`, `FlightForge/LidarNav-v0`
(auto-registered via the `gymnasium.envs` entry point):

```python
import gymnasium
env = gymnasium.make("FlightForge/CameraNav-v0", render_mode="rgb_array")
observation, info = env.reset()
observation, reward, terminated, truncated, info = env.step(env.action_space.sample())
```

Both environments teleport-with-collision-check, so they need no dynamics
model; wrap your own dynamics for realism, or:

**MuJoCo bridge** — MuJoCo integrates, FlightForge renders:

```python
from flightforge import Simulator, MujocoBridge

sim = Simulator()
drone = sim.spawn_drone((0, 0, 200), "x500")
bridge = MujocoBridge(drone, "quadrotor.xml", world_origin_uu=sim.world_origin())

bridge.data.ctrl[:] = thrusts
bridge.step()                # mj_step + pose push
image, stamp = drone.rgb()   # photorealistic render of the MuJoCo state
```

Any consumer of the MuJoCo state works the same way — MJX/JAX rollouts,
MATLAB-exported trajectories — as long as something fills `qpos`.

**Parallel multi-robot** — N drones in one world, mutually invisible, each
stepped and polled from its own thread (the bindings release the GIL during
network I/O, so sensor requests genuinely overlap):

```python
sim.set_mutual_visibility(False)   # every drone sees the world as if alone
```

**Scene editing** — build, rearrange, save and reload a scene over the wire
(poses in ROS metres/degrees relative to the world origin):

```python
ids = sim.spawn_objects(yaml_text)     # returns handles
sim.move_object(ids[0], (8, 2, 1.5), (0, 0, 90))
sim.objects()                          # {id: {asset, stencil, position, ...}}
yaml_text = sim.export_scene()         # round-trippable, stencils included
sim.assets(name_filter="cube")         # what the simulator can spawn
```

See `python/examples/` for runnable versions of all of the above
(`parallel_drones.py` for the multi-robot demo, `scene_editor.py` for the
scene-editing loop).

## Tests

```bash
# unit tests - run anywhere, no simulator (sim-marked tests skip cleanly)
pip install -e python[test]
pytest python/tests

# integration suite - start the simulator first, then:
pytest python/tests -m sim

# wire-protocol round-trip (C++, catches serialize-list mistakes)
cmake -B build -DBUILD_TESTS=ON && cmake --build build --target serialization_roundtrip
ctest --test-dir build -R serialization_roundtrip
```

The `sim` suite doubles as the runtime validation harness: cameras and
calibration round-trips, the scene-editor round-trip, mutual visibility,
sensor lifecycle and instance segmentation. The engine-side math (frame
conversions, distortion LUT inverse, noise determinism) is covered by the
plugin's automation tests - in the editor: Tools > Test Automation, filter
"FlightForge", or headless:

    UnrealEditor-Cmd <project.uproject> -ExecCmds="Automation RunTests FlightForge" -unattended -nopause
