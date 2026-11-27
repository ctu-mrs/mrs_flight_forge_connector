"""Instance segmentation via the SegId mesh pass.

Unlike the stencil segmentation (255 values, collisions), the instance camera renders every actor's 24-bit id
through a dedicated pass: pixel-exact, occlusion-correct, unlimited instances, and each frame comes with the
id -> actor map snapshotted at the same instant. A ring of cubes is spawned around the robot, 5 m out.

Validation idea: run this while the editor console runs FlightForge.DumpSegGroundTruth from the same viewpoint;
the two masks should segment identically.

Requires a running simulator:
    python3 python/examples/instance_segmentation.py
"""

import math

import numpy as np

from flightforge import Simulator


def ring_yaml(count=12, radius=5.0):
    lines = ["objects:"]
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        lines += [
            "  - asset: /Engine/BasicShapes/Cube",
            f"    position: [{radius * math.cos(angle):.2f}, {radius * math.sin(angle):.2f}, 1.0]",
        ]
    return "\n".join(lines)


def main():
    sim = Simulator()
    robot = sim.spawn_robot((0.0, 0.0, 1.5), "x500")

    sim.spawn_objects(ring_yaml())

    ids, stamp = robot.instance_segmentation()
    mapping = robot.instance_map()

    visible = np.unique(ids)
    visible = visible[visible != 0]

    print(f"frame {ids.shape} at t={stamp:.3f}: {len(visible)} visible instances")
    for instance_id in visible[:20]:
        pixels = int((ids == instance_id).sum())
        print(f"  id {instance_id}: {pixels} px  {mapping.get(int(instance_id), '<unmapped>')}")

    try:
        import cv2

        rng = np.random.default_rng(0)
        palette = rng.integers(64, 255, size=(int(ids.max()) + 1, 3), dtype=np.uint8)
        palette[0] = 0
        cv2.imwrite("instance_seg.png", palette[ids])
        print("saved instance_seg.png")
    except ImportError:
        pass

    sim.clear_objects()
    sim.remove_robot(robot)
    sim.close()


if __name__ == "__main__":
    main()
