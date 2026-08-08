"""Reprojection check for the camera intrinsics (camera realism Phase 2).

Spawns a grid of small cubes at known world positions, captures the
segmentation image, and compares each cube's observed pixel centroid against
where the pinhole model K·[R|t] says it should land. Run it twice — once on
the default FOV-driven projection, once after set_camera_intrinsics with an
off-center principal point — the error should stay sub-pixel-ish in both.

Requires a running simulator and opencv:
    pip install -e python[vision]
    python3 python/examples/intrinsics_check.py
"""

import math

import numpy as np

from flightforge import Simulator

GRID = [(x, y) for x in (4.0, 6.0, 8.0) for y in (-2.0, 0.0, 2.0)]
CUBE_Z = 1.0
CUBE_SCALE = 0.25
FIRST_STENCIL = 200


def spawn_grid_yaml():
    lines = ["objects:"]
    for index, (x, y) in enumerate(GRID):
        lines += [
            "  - asset: Cube",
            f"    position: [{x}, {y}, {CUBE_Z}]",
            f"    scale: {CUBE_SCALE}",
            f"    stencil: {FIRST_STENCIL + index}",
        ]
    return "\n".join(lines)


def camera_pose_looking_at_grid(drone):
    """Body frame: camera looks +X (forward). Drone at origin height CUBE_Z sees the grid head-on."""
    drone.set_pose((0.0, 0.0, 100.0 * CUBE_Z), (0.0, 0.0, 0.0))


def expected_pixel(K, point_ros):
    """ROS world point (m, relative to world origin) -> pixel, for a camera at
    (0, 0, CUBE_Z) looking along +X with zero rotation. Camera frame: z forward
    = world x, x right = world -y, y down = world -z."""
    px, py, pz = point_ros
    cam = np.array([-py, -(pz - CUBE_Z), px])
    uvw = K @ cam
    return uvw[:2] / uvw[2]


def observed_centroids(seg_image):
    """Centroids of the cube blobs, independent of how the segmentation material
    maps stencils to colors: everything that isn't the dominant background color
    is foreground, split into connected components."""
    import cv2

    flat = seg_image.reshape(-1, 3)
    colors, counts = np.unique(flat, axis=0, return_counts=True)
    background = colors[counts.argmax()]

    foreground = (seg_image != background).any(axis=2).astype(np.uint8)
    count, _, stats, centroids = cv2.connectedComponentsWithStats(foreground)

    return [np.array(centroids[label]) for label in range(1, count) if stats[label, cv2.CC_STAT_AREA] >= 4]


def run_check(drone, label):
    K, (width, height) = drone.camera_intrinsics()
    print(f"\n[{label}] K =\n{K.round(2)}  ({width}x{height})")

    seg, _ = drone.segmentation()
    observed = observed_centroids(seg)
    predictions = [expected_pixel(K, (x, y, CUBE_Z)) for x, y in GRID]

    # each blob claims its nearest prediction; unmatched predictions are off-screen or occluded
    errors = []
    for centroid in observed:
        distances = [np.linalg.norm(centroid - p) for p in predictions]
        index = int(np.argmin(distances))
        errors.append(distances[index])
        print(f"  blob at {centroid.round(1)} -> cube {index} predicted {predictions[index].round(1)}, error {distances[index]:.2f} px")

    if errors:
        print(f"[{label}] mean reprojection error: {np.mean(errors):.2f} px over {len(errors)} blobs "
              f"({len(GRID) - len(errors)} cubes unmatched)")
    else:
        print(f"[{label}] no cube blobs found - check the camera pose or the seg material")


def main():
    sim = Simulator()
    drone = sim.spawn_drone((0.0, 0.0, 100.0 * CUBE_Z), "x500")
    camera_pose_looking_at_grid(drone)

    sim.spawn_objects(spawn_grid_yaml())

    run_check(drone, "fov-derived intrinsics")

    # off-center principal point: the same scene must land where the new K says
    K, (width, height) = drone.camera_intrinsics()
    drone.set_camera_intrinsics(fx=K[0, 0] * 1.1, fy=K[1, 1] * 1.1, cx=width * 0.45, cy=height * 0.55)
    run_check(drone, "custom off-center intrinsics")

    sim.clear_objects()
    sim.remove_drone(drone)
    drone.close()
    sim.close()


if __name__ == "__main__":
    main()
