"""N robots in one world, running and sensing in parallel.

Demonstrates the three pieces that make FlightForge a parallel multi-robot
trainer:

  1. ``Simulator.set_mutual_visibility(False)`` — every drone is hidden from
     every other drone's sensors, so each robot perceives the world as if it
     were alone: N robots in one world behave like N independent environments
     that share one rendering process.
  2. Each drone has its own server port and its own connection, and the
     bindings release the GIL during network I/O — sensor requests for
     different drones genuinely overlap from Python threads.
  3. Each robot's dynamics come from its own MuJoCo model, stepped in its own
     thread (``flightforge.MujocoBridge``); FlightForge just renders.

The drones spawn in a circle facing the center — the most incriminating
arrangement possible: with mutual visibility ON each camera would stare
straight at two other drones; with it OFF the saved images show empty air.

    pip install -e python[mujoco,vision]
    python3 python/examples/parallel_drones.py [num_drones]
"""

import concurrent.futures
import math
import sys
import time

import numpy as np

from flightforge import MujocoBridge, Simulator

NUM_DRONES = int(sys.argv[1]) if len(sys.argv) > 1 else 3
CIRCLE_RADIUS_UU = 400.0
ALTITUDE_UU = 200.0
HOVER_SECONDS = 3.0

QUADROTOR_MJCF = """
<mujoco model="minimal_quadrotor">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="drone" pos="{x} {y} 2">
      <freejoint/>
      <geom type="box" size="0.12 0.12 0.03" mass="1.2"/>
      <site name="thrust" pos="0 0 0"/>
    </body>
  </worldbody>
  <actuator>
    <motor site="thrust" gear="0 0 20 0 0 0"/>
  </actuator>
</mujoco>
"""


def spawn_ring(sim):
    """Drones on a circle, each yawed to face the center."""
    robots = []
    for i in range(NUM_DRONES):
        angle = 2.0 * math.pi * i / NUM_DRONES
        x = CIRCLE_RADIUS_UU * math.cos(angle)
        y = CIRCLE_RADIUS_UU * math.sin(angle)
        yaw_to_center = math.degrees(math.atan2(-y, -x))

        drone = sim.spawn_drone((x, y, ALTITUDE_UU), "x500")
        drone.set_pose((x, y, ALTITUDE_UU), (0.0, yaw_to_center, 0.0))

        mjcf = QUADROTOR_MJCF.format(x=x / 100.0, y=-y / 100.0)
        import mujoco

        bridge = MujocoBridge(drone, mujoco.MjModel.from_xml_string(mjcf), body="drone", substeps=5)

        robots.append((i, drone, bridge, yaw_to_center))
    return robots


def hover_worker(entry):
    """Owns one robot: steps its dynamics and grabs frames, in its own thread."""
    index, drone, bridge, yaw = entry

    target_z, kp, kd = 2.0, 8.0, 4.0
    frames = 0
    deadline = time.monotonic() + HOVER_SECONDS

    while time.monotonic() < deadline:
        z, vz = bridge.data.qpos[2], bridge.data.qvel[2]
        bridge.data.ctrl[:] = np.clip((1.2 * 9.81 + kp * (target_z - z) - kd * vz) / 20.0, 0.0, 1.0)
        bridge.step()

        image, _ = drone.rgb()
        frames += 1

    return index, frames, image


def main():
    sim = Simulator()
    print(f"API {sim.api_version[0]}.{sim.api_version[1]}, spawning {NUM_DRONES} drones in a ring")

    # the crux: nobody sees anybody
    sim.set_mutual_visibility(False)

    robots = spawn_ring(sim)

    # every robot runs in its own thread; the GIL is released inside the
    # network calls, so the per-drone sensor requests genuinely overlap
    start = time.monotonic()
    with concurrent.futures.ThreadPoolExecutor(max_workers=NUM_DRONES) as pool:
        results = list(pool.map(hover_worker, robots))
    elapsed = time.monotonic() - start

    total_frames = sum(f for _, f, _ in results)
    print(f"{NUM_DRONES} drones hovered {HOVER_SECONDS:.0f}s wall-clock in {elapsed:.1f}s")
    print(f"aggregate camera rate: {total_frames / elapsed:.1f} fps across all drones")
    for index, frames, _ in results:
        print(f"  drone {index}: {frames} frames ({frames / elapsed:.1f} fps)")

    # each camera pointed at two other drones - the saved views should show
    # empty air where they hover
    try:
        import cv2

        for index, _, image in results:
            path = f"drone_{index}_view.png"
            cv2.imwrite(path, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            print(f"  saved {path} (facing the ring center - other drones invisible)")
    except ImportError:
        print("install opencv-python to save the proof images")

    for _, drone, _, _ in robots:
        sim.remove_drone(drone)
        drone.close()
    sim.close()


if __name__ == "__main__":
    main()
