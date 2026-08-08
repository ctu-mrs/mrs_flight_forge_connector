"""Tour of the API 0.14 features: devices, multi-sensor addressing, the event
and fisheye cameras, and YAML object spawning.

Run against a simulator built from the merge/pr12-5.7 branch:
    PYTHONPATH=../../build python3 showcase_api014.py
"""

from controllers import (
    Coordinates,
    DroneController,
    EventCameraConfig,
    GameModeController,
    Rotation,
    SensorType,
)

GAME_MODE_PORT = 8551

SPAWN_YAML = """
search_paths:
  - /Game/Worlds
objects:
  - asset: Cube
    position: {x: 5.0, y: 0.0, z: 1.0}
    orientation: {x: 0.0, y: 0.0, z: 45.0}
    scale: {x: 1.0, y: 1.0, z: 1.0}
"""


def main():
    game_mode = GameModeController("127.0.0.1", GAME_MODE_PORT)
    assert game_mode.ConnectSimple(), "cannot reach the simulator"

    ok, (major, minor) = game_mode.GetApiVersion()
    print(f"API {major}.{minor}")
    assert (major, minor) >= (0, 14), "this example needs API >= 0.14"

    # drone frames are plain mesh names; unknown names fall back to on-disk
    # .glb models (drone10/drone20 ship with the plugin)
    ok, port = game_mode.SpawnDroneAtLocation(Coordinates(0, 0, 200), "x500")
    assert ok, "spawn failed"
    print(f"drone on port {port}")

    drone = DroneController("127.0.0.1", port)
    assert drone.ConnectSimple()

    # --- device library: one call places a preconfigured real sensor ---
    ok, devices = drone.ListDevices()
    print("devices:", [f"{d.name}" for d in devices])

    ok, sensors = drone.AddDevice("realsense_d435i", Coordinates(12, 0, -2), Rotation(0, 0, 0), show_mesh=True)
    assert ok
    print("d435i created sensors:", [(s.id, s.type) for s in sensors])

    # the stereo pair of the D435i, addressed explicitly by its id
    stereo_id = next(s.id for s in sensors if s.type == int(SensorType.STEREO_CAMERA))
    ok, left, right, stamp = drone.GetStereoCameraData(sensor_id=stereo_id)
    print(f"stereo frame: {len(left)}+{len(right)} bytes at t={stamp:.3f}")

    # --- default-sensor addressing: -1 creates lazily on first use ---
    ok, image, stamp, size = drone.GetRgbCameraData()
    print(f"rgb frame: {size} bytes at t={stamp:.3f}")

    # --- event camera ---
    config = EventCameraConfig()
    config.width = 346
    config.height = 260
    config.contrast_threshold_pos = 0.25
    config.contrast_threshold_neg = 0.25
    drone.SetEventCameraConfig(config)

    ok, events, stamp = drone.GetEventCameraData()
    print(f"event batch: {len(events)} events, batch stamp t={stamp:.3f}")

    # --- fisheye ---
    ok, fisheye, stamp, size = drone.GetFisheyeCameraData()
    print(f"fisheye frame: {size} bytes at t={stamp:.3f}")

    ok, listed = drone.ListSensors()
    print("sensors now on the drone:", [(s.id, s.type) for s in listed])

    # --- scene objects from a YAML document ---
    ok, object_ids, error = game_mode.SpawnObjectsFromYaml(SPAWN_YAML)
    print(f"spawned objects: {object_ids} (error: '{error}')")

    if object_ids:
        game_mode.RemoveSpawnedObject(object_ids[0])
    game_mode.RemoveAllSpawnedObjects()

    drone.Disconnect()
    game_mode.Disconnect()


if __name__ == "__main__":
    main()
