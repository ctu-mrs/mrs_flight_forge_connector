"""Event camera driven in lockstep from a client-owned trajectory.

The simulator is put on a fixed 1 ms timestep and paused; the drone is handed a
short yaw sweep as a pose timeline; the world is stepped 200 ticks (0.2 s of sim
time, however long the GPU takes); the events of that window are drained and
summarised, and an accumulation image is written next to this script.

Usage: python event_camera_lockstep.py [device]   (default prophesee_evk4)
"""

import sys
import time

import numpy as np

from flightforge import Simulator

DT = 1e-3
TICKS = 200


def main():
    device = sys.argv[1] if len(sys.argv) > 1 else "prophesee_evk4"

    sim = Simulator()
    drone = sim.spawn_drone((0.0, 0.0, 300.0))

    sensors = drone.add_device(device, offset=(10.0, 0.0, 0.0))
    event_id = next(sensor_id for sensor_id, sensor_type in sensors if sensor_type == 6)

    # the first drain both starts the stream and empties whatever was seen before
    drone.event_batch(event_id)

    applied = sim.set_fixed_timestep(DT)
    sim.set_lockstep(True)
    print(f"fixed timestep {applied * 1e3:.3f} ms, lockstep on")

    # sweep 20 deg of yaw over the window, sampled at 100 Hz: the pawn interpolates
    # between these, so the 1 kHz sensor sees smooth motion
    t0, _ = sim.sim_time()
    ts = t0 + np.linspace(0.0, TICKS * DT, 21)
    yaw = np.linspace(0.0, 20.0, ts.size)
    position = drone.get_position()
    trajectory = np.column_stack([ts, np.tile(position, (ts.size, 1)), np.zeros(ts.size), yaw, np.zeros(ts.size)])
    drone.set_pose_trajectory(trajectory)

    wall = time.perf_counter()
    sim_time, ticks = sim.step(TICKS)
    wall = time.perf_counter() - wall

    batch = drone.event_batch(event_id)
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

        png = np.clip(128 + 40 * image, 0, 255).astype(np.uint8)
        cv2.imwrite("events_lockstep.png", png)
        print("wrote events_lockstep.png")
    except ImportError:
        pass

    sim.set_lockstep(False)
    sim.set_fixed_timestep(0.0)
    drone.set_pose_trajectory(np.zeros((0, 7)))
    sim.remove_drone(drone)


if __name__ == "__main__":
    main()
