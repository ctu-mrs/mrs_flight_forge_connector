"""The scene-editor loop: build a scene from code, rearrange it, save it, reload it.

Everything runs over the wire against a live simulator — the same operations a
future in-editor "Minecraft mode" gizmo will use. All poses are right-handed
(ROS) metres and degrees relative to the world origin.

    pip install -e python
    python3 python/examples/scene_editor.py
"""

import math

from flightforge import Simulator

RING_RADIUS_M = 6.0
NUM_CUBES = 8


def ring_yaml(radius, count):
    lines = ["objects:"]
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        x, y = radius * math.cos(angle), radius * math.sin(angle)
        lines += [
            "  - asset: Cube",
            f"    position: [{x:.3f}, {y:.3f}, 1.0]",
            f"    orientation: [0.0, 0.0, {math.degrees(angle):.1f}]",
        ]
    return "\n".join(lines)


def main():
    sim = Simulator()

    names, truncated = sim.assets(name_filter="cube")
    print(f"spawnable assets matching 'cube': {names[:5]}{' …' if truncated or len(names) > 5 else ''}")

    ids = sim.spawn_objects(ring_yaml(RING_RADIUS_M, NUM_CUBES))
    print(f"spawned {len(ids)} cubes in a ring")

    # rearrange: lift every second cube into a second storey
    for index, object_id in enumerate(ids):
        if index % 2:
            pose = sim.objects()[object_id]
            position = pose["position"]
            position[2] += 2.0
            sim.move_object(object_id, position, pose["orientation"])
    print("lifted every second cube by 2 m")

    for object_id, info in sim.objects().items():
        print(f"  {object_id}: {info['asset']} at {info['position'].round(2)} stencil {info['stencil']}")

    # the export includes the moves and the stencils - it IS the scene
    scene_yaml = sim.export_scene()
    with open("scene.yaml", "w") as f:
        f.write(scene_yaml)
    print("scene saved to scene.yaml")

    removed = sim.clear_objects()
    print(f"cleared {removed} objects")

    ids = sim.spawn_objects(scene_yaml)
    print(f"scene restored from the export: {len(ids)} objects")

    sim.clear_objects()
    sim.close()


if __name__ == "__main__":
    main()
