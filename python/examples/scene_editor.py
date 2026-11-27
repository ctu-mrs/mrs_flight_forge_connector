"""The scene-editor loop: build a scene from code, rearrange it, save it, reload it.

Every operation runs over the wire against a live simulator. Poses are in the ground frame: metres, x forward,
y left, z up from the floor, orientations as quaternions; spawn configs take roll, pitch, yaw in degrees.

    pip install -e python
    python3 python/examples/scene_editor.py
"""

import math

from flightforge import Pose, Simulator

RING_RADIUS = 6.0  # m
NUM_CUBES = 8


def ring_yaml(radius, count):
    lines = ["frame: ground", "objects:"]
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        lines += [
            "  - asset: /Engine/BasicShapes/Cube",
            f"    position: [{radius * math.cos(angle):.3f}, {radius * math.sin(angle):.3f}, 1.0]",
            f"    orientation: [0.0, 0.0, {math.degrees(angle):.1f}]",
        ]
    return "\n".join(lines)


def main():
    sim = Simulator()

    names, truncated = sim.assets(name_filter="cube")
    print(f"spawnable assets matching 'cube': {names[:5]}{' ...' if truncated or len(names) > 5 else ''}")

    ids = sim.spawn_objects(ring_yaml(RING_RADIUS, NUM_CUBES))
    print(f"spawned {len(ids)} cubes in a ring")

    objects = sim.objects()
    for index, object_id in enumerate(ids):
        if index % 2:
            pose = objects[object_id]["pose"]
            sim.move_object(object_id, Pose(pose.position + [0.0, 0.0, 2.0], pose.orientation))
    print("lifted every second cube by 2 m")

    for object_id, info in sim.objects().items():
        print(f"  {object_id}: {info['asset']} at {info['position'].round(2)}, yaw {math.degrees(info['pose'].yaw):.0f} deg, stencil {info['stencil']}")

    scene_yaml = sim.export_scene()
    with open("scene.yaml", "w") as f:
        f.write(scene_yaml)
    print("scene saved to scene.yaml")

    print(f"cleared {sim.clear_objects()} objects")

    ids = sim.spawn_objects(scene_yaml)
    print(f"scene restored from the export: {len(ids)} objects")

    sim.clear_objects()
    sim.close()


if __name__ == "__main__":
    main()
