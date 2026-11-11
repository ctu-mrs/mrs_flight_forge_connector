"""Event camera driven in lockstep from a client-owned trajectory.

The simulator is put on a fixed 1 ms timestep and paused; the robot is handed a short yaw sweep as a trajectory;
the world is stepped 200 ticks (0.2 s of sim time, however long the GPU takes); the events of that window are
drained and summarised, and an accumulation image is written next to this script.

Usage: python event_camera_lockstep.py [device]   (default prophesee_evk4)
"""

import math
import sys
import time

import numpy as np

from flightforge import Pose, SensorType, Simulator, quat_from_rpy, quat_multiply

DT = 1e-3
TICKS = 200


def main():
    device = sys.argv[1] if len(sys.argv) > 1 else "prophesee_evk4"

    sim = Simulator()
    robot = sim.spawn_robot((0.0, 0.0, 3.0))

    sensors = robot.add_device(device, Pose.at(0.1, 0.0, 0.0))
    event_id = next(sensor.id for sensor in sensors if sensor.type == SensorType.EVENT_CAMERA)

    # the first drain both starts the stream and empties whatever was seen before
    robot.event_batch(event_id)

    applied = sim.set_fixed_timestep(DT)
    sim.set_lockstep(True)
    print(f"fixed timestep {applied * 1e3:.3f} ms, lockstep on")

    # 20 deg of yaw sampled at 100 Hz; the robot interpolates, so the 1 kHz sensor sees smooth motion
    t0, _ = sim.sim_time()
    start = robot.get_pose()
    ts = t0 + np.linspace(0.0, TICKS * DT, 21)
    yaws = np.radians(np.linspace(0.0, 20.0, ts.size))
    robot.set_trajectory(
        np.array([[t, *start.position, *quat_multiply(quat_from_rpy(yaw=yaw), start.orientation)] for t, yaw in zip(ts, yaws)])
    )

    wall = time.perf_counter()
    sim_time, ticks = sim.step(TICKS)
    wall = time.perf_counter() - wall

    batch = robot.event_batch(event_id)
    span = batch.t[-1] - batch.t[0] if len(batch) > 1 else 0.0

    print(f"stepped {ticks} ticks to t={sim_time:.3f} s in {wall:.2f} s wall")
    print(f"{len(batch)} events over {span * 1e3:.1f} ms, {batch.dropped} dropped")
    if len(batch):
        rate = len(batch) / max(span, DT)
        print(f"{rate / 1e6:.2f} Mev/s, {np.mean(batch.polarity > 0) * 100:.0f}% brighter")
        print(f"first stamps: {batch.t[:5] - batch.t[0]}")

    image = batch.accumulate()
    try:
        import cv2

        cv2.imwrite("events_lockstep.png", np.clip(128 + 40 * image, 0, 255).astype(np.uint8))
        print("wrote events_lockstep.png")
    except ImportError:
        pass

    sim.set_lockstep(False)
    sim.set_fixed_timestep(0.0)
    robot.set_trajectory([])
    sim.remove_robot(robot)
    print(f"swept {math.degrees(yaws[-1]):.0f} deg of yaw")


if __name__ == "__main__":
    main()
