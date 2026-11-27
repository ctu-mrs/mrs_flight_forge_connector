"""N robots in one world, running and sensing in parallel.

Three pieces make FlightForge a parallel multi-robot trainer:

  1. Simulator.set_mutual_visibility(False) hides every robot from every other robot's sensors, so N robots in one
     world behave like N independent environments sharing one rendering process.
  2. Each robot gets its own connection (own_connection=True); the simulator serves every connection on its own
     thread and the sockets release the GIL, so sensor requests from Python threads genuinely overlap.
  3. Each robot's dynamics come from its own MuJoCo model, stepped in its own thread; FlightForge just renders.

The robots spawn on a circle facing the centre: with mutual visibility on each camera would stare at two other
robots; with it off the saved images show empty air.

    pip install -e python[mujoco,vision]
    python3 python/examples/parallel_drones.py [num_robots]
"""

import concurrent.futures
import math
import sys
import time

import numpy as np

from flightforge import MujocoBridge, Pose, Simulator

NUM_ROBOTS = int(sys.argv[1]) if len(sys.argv) > 1 else 3
CIRCLE_RADIUS = 4.0  # m
ALTITUDE = 2.0  # m
HOVER_SECONDS = 3.0

QUADROTOR_MJCF = """
<mujoco model="minimal_quadrotor">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="drone" pos="{x} {y} {z}" quat="{qw} 0 0 {qz}">
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
    """Robots on a circle, each yawed to face the centre."""
    import mujoco

    robots = []
    for i in range(NUM_ROBOTS):
        angle = 2.0 * math.pi * i / NUM_ROBOTS
        pose = Pose.at(CIRCLE_RADIUS * math.cos(angle), CIRCLE_RADIUS * math.sin(angle), ALTITUDE, angle + math.pi)

        robot = sim.spawn_robot(pose, "x500", label=f"uav{i + 1}", own_connection=True)

        x, y, z = pose.position
        qx, qy, qz, qw = pose.orientation
        bridge = MujocoBridge(robot, mujoco.MjModel.from_xml_string(QUADROTOR_MJCF.format(x=x, y=y, z=z, qw=qw, qz=qz)), body="drone", substeps=5)

        robots.append((i, robot, bridge))
    return robots


def hover_worker(entry):
    """Owns one robot: steps its dynamics and grabs frames, in its own thread."""
    index, robot, bridge = entry

    kp, kd = 8.0, 4.0
    frames = 0
    image = None
    deadline = time.monotonic() + HOVER_SECONDS

    while time.monotonic() < deadline:
        z, vz = bridge.data.qpos[2], bridge.data.qvel[2]
        bridge.data.ctrl[:] = np.clip((1.2 * 9.81 + kp * (ALTITUDE - z) - kd * vz) / 20.0, 0.0, 1.0)
        bridge.step()

        image, _ = robot.rgb()
        frames += 1

    return index, frames, image


def main():
    sim = Simulator()
    print(f"API {sim.api_version[0]}.{sim.api_version[1]}, spawning {NUM_ROBOTS} robots in a ring")

    sim.set_mutual_visibility(False)

    robots = spawn_ring(sim)

    start = time.monotonic()
    with concurrent.futures.ThreadPoolExecutor(max_workers=NUM_ROBOTS) as pool:
        results = list(pool.map(hover_worker, robots))
    elapsed = time.monotonic() - start

    total_frames = sum(frames for _, frames, _ in results)
    print(f"{NUM_ROBOTS} robots hovered {HOVER_SECONDS:.0f} s wall-clock in {elapsed:.1f} s")
    print(f"aggregate camera rate: {total_frames / elapsed:.1f} fps across all robots")
    for index, frames, _ in results:
        print(f"  robot {index}: {frames} frames ({frames / elapsed:.1f} fps)")

    try:
        import cv2

        for index, _, image in results:
            path = f"robot_{index}_view.png"
            cv2.imwrite(path, cv2.cvtColor(np.ascontiguousarray(image), cv2.COLOR_RGB2BGR))
            print(f"  saved {path} (facing the ring centre, the other robots invisible)")
    except ImportError:
        print("install opencv-python to save the images")

    sim.set_mutual_visibility(True)
    for _, robot, _ in robots:
        sim.remove_robot(robot)
        robot.close()
    sim.close()


if __name__ == "__main__":
    main()
