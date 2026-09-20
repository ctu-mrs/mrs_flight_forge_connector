"""Reprojection check for the camera intrinsics and distortion (Phases 2 + 3).

Spawns a grid of small cubes at known scene positions, reads the instance
segmentation, and compares each cube's pixel centroid against where the pinhole
model says it should land. Runs three times: FOV-derived projection, custom
off-centre intrinsics, and Brown-Conrady distortion. Each pass should stay at
about a pixel; the centroid of a rendered cube is not exactly its centre's
projection, so expect around 1 px rather than zero.

Two frames meet here and the example converts explicitly: scene objects live in
metres relative to the "UAV" PlayerStart (right-handed, x forward, y left, z up),
the drone in absolute Unreal centimetres. Cubes are told apart by instance id,
not colour: the ids that appear after spawning are the cubes, and the drone's
own airframe, which the pawn-mounted camera partly sees, is never mistaken for
one.

Requires a running simulator and opencv:
    pip install -e python[vision]
    python3 python/examples/intrinsics_check.py [game-mode-port]
"""

import sys
import time

import numpy as np

from flightforge import SensorType, Simulator

CAMERA_SCENE = np.array([0.0, 0.0, 1.5])
# heights vary with both axes so no cube hides behind a nearer one on the same ray
GRID = [(x, y, CAMERA_SCENE[2] + 0.25 * y + 0.15 * (x - 6.0)) for x in (4.0, 6.0, 8.0) for y in (-2.0, 0.0, 2.0)]
CUBE_SCALE = 0.25


def spawn_grid_yaml():
    lines = ["objects:"]
    for x, y, z in GRID:
        lines += [
            "  - asset: /Engine/BasicShapes/Cube",
            f"    position: [{x}, {y}, {z}]",
            f"    scale: {CUBE_SCALE}",
        ]
    return "\n".join(lines)


def expected_pixel(K, point_scene, dist=None):
    """Scene point (m) -> pixel for a camera at CAMERA_SCENE looking along +x with
    zero rotation. Camera frame: z forward = scene x, x right = scene -y,
    y down = scene -z. With dist = [k1 k2 k3 p1 p2] the forward Brown-Conrady
    model is applied before K."""
    px, py, pz = np.asarray(point_scene) - CAMERA_SCENE
    x, y, z = -py, -pz, px
    xn, yn = x / z, y / z

    if dist is not None:
        k1, k2, k3, p1, p2 = dist
        r2 = xn * xn + yn * yn
        radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
        xd = xn * radial + 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn)
        yd = yn * radial + p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn
        xn, yn = xd, yd

    return np.array([K[0, 0] * xn + K[0, 2], K[1, 1] * yn + K[1, 2]])


def settle(drone, sensor_id):
    """A projection change reaches the served image a few frames late: the target is
    resized and the warp LUT rebuilt on the game thread while frames already in
    flight carry the old calibration."""
    time.sleep(0.3)
    for _ in range(3):
        drone.instance_segmentation(sensor_id)


def cube_centroids(drone, sensor_id, cube_ids):
    ids, _ = drone.instance_segmentation(sensor_id)
    centroids = {}
    for cube in cube_ids:
        ys, xs = np.nonzero(ids == cube)
        if xs.size >= 4:
            centroids[cube] = np.array([xs.mean(), ys.mean()])
    return centroids


def run_check(drone, sensor_id, cube_ids, label, dist=None):
    K, (width, height) = drone.camera_intrinsics(sensor_id)
    print(f"\n[{label}] K =\n{K.round(2)}  ({width}x{height})")

    settle(drone, sensor_id)
    observed = cube_centroids(drone, sensor_id, cube_ids)
    predictions = [expected_pixel(K, point, dist) for point in GRID]

    # every cube is a distinct instance, so each blob claims its nearest prediction unambiguously
    errors = []
    for cube, centroid in observed.items():
        distances = [np.linalg.norm(centroid - p) for p in predictions]
        index = int(np.argmin(distances))
        errors.append(distances[index])
        print(f"  instance {cube} at {centroid.round(1)} -> cube {index} predicted {predictions[index].round(1)}, error {distances[index]:.2f} px")

    if errors:
        print(f"[{label}] mean reprojection error: {np.mean(errors):.2f} px, max {np.max(errors):.2f} px over {len(errors)} cubes "
              f"({len(GRID) - len(errors)} not visible)")
    else:
        print(f"[{label}] no cube instances visible - check the camera pose")
    return errors


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8551
    sim = Simulator(port=port)
    sim.clear_objects()

    camera_world = sim.scene_to_world(CAMERA_SCENE)
    drone = sim.spawn_drone(camera_world, "x500")
    drone.set_pose(camera_world, (0.0, 0.0, 0.0))

    sensor_id = drone.add_sensor(SensorType.INSTANCE_SEG_CAMERA)
    settle(drone, sensor_id)
    before = set(np.unique(drone.instance_segmentation(sensor_id)[0]))

    sim.spawn_objects(spawn_grid_yaml())
    settle(drone, sensor_id)
    cube_ids = sorted(set(np.unique(drone.instance_segmentation(sensor_id)[0])) - before - {0})
    print(f"{len(cube_ids)} cube instances in view of {len(GRID)} spawned")

    run_check(drone, sensor_id, cube_ids, "fov-derived intrinsics")

    K, (width, height) = drone.camera_intrinsics(sensor_id)
    drone.set_camera_intrinsics(fx=K[0, 0] * 1.1, fy=K[1, 1] * 1.1, cx=width * 0.45, cy=height * 0.55, sensor_id=sensor_id)
    run_check(drone, sensor_id, cube_ids, "custom off-center intrinsics")

    dist = [-0.28, 0.07, 0.0, 0.001, -0.0005]
    drone.set_camera_distortion(*dist, sensor_id=sensor_id)
    run_check(drone, sensor_id, cube_ids, "brown-conrady distorted", dist=dist)

    sim.clear_objects()
    sim.remove_drone(drone)
    drone.close()
    sim.close()


if __name__ == "__main__":
    main()
