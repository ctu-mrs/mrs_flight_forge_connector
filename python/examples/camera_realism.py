"""The full realistic-camera surface in one place: configure the RGB camera
from a real sensor's datasheet + calibration, then capture.

Every stage is independent — enable only what your study needs:

  intrinsics      exact pinhole K (fx, fy, cx, cy), off-center supported
  distortion      Brown-Conrady k1 k2 k3 p1 p2 (OpenCV convention)
  exposure        physical EV100 model: shutter, ISO, EV bias; motion blur
                  can be derived from the shutter time
  noise           shot + read (+ row banding), linearized domain, per-frame
                  deterministic
  rolling shutter first-order rotational warp from the camera's measured
                  angular velocity — spin the drone to see it

Requires a running simulator:
    python3 python/examples/camera_realism.py
"""

import time

from flightforge import Simulator

# a hand-written "datasheet": a 640x480 module with a slightly off-center
# principal point, mild barrel lens, 5 ms shutter and a 25 ms readout
CALIBRATION = {
    "K": dict(fx=460.0, fy=460.0, cx=316.2, cy=243.8),
    "dist": dict(k1=-0.21, k2=0.05, k3=0.0, p1=0.0004, p2=-0.0002),
    "noise": dict(shot_scale=3.0e-4, read_sigma=4.0e-3, row_sigma=1.0e-3),
    "readout_time": 0.025,
}


def main():
    sim = Simulator()
    drone = sim.spawn_drone((0.0, 0.0, 200.0), "x500")

    drone.set_camera_intrinsics(**CALIBRATION["K"])
    drone.set_camera_distortion(**CALIBRATION["dist"])
    drone.set_camera_noise(**CALIBRATION["noise"])
    drone.set_rolling_shutter(CALIBRATION["readout_time"])

    # physical exposure via the raw config (see showcase_api014.py for more)
    ok, config = drone.raw.GetRgbCameraConfig()
    if ok:
        config.exposure.manual = True
        config.exposure.shutter_time = 1.0 / 200.0
        config.exposure.iso = 200.0
        config.lens.motion_blur_from_shutter = True
        drone.raw.SetRgbCameraConfig(config)

    K, size = drone.camera_intrinsics()
    print(f"camera reports K =\n{K.round(2)}  at {size}")

    image, stamp = drone.rgb()
    print(f"static frame: {image.shape} at t={stamp:.3f}")

    # spin fast while grabbing a frame - the rolling shutter should shear verticals
    drone.set_pose_async((0.0, 0.0, 200.0), (0.0, 180.0, 0.0))
    time.sleep(0.05)
    image_spinning, _ = drone.rgb()

    try:
        import cv2

        cv2.imwrite("realistic_static.png", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        cv2.imwrite("realistic_spinning.png", cv2.cvtColor(image_spinning, cv2.COLOR_RGB2BGR))
        print("saved realistic_static.png / realistic_spinning.png")
    except ImportError:
        pass

    sim.remove_drone(drone)
    drone.close()
    sim.close()


if __name__ == "__main__":
    main()
