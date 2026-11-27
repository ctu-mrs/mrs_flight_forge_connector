"""The full realistic-camera surface in one place: configure the RGB camera from a real sensor's datasheet and
calibration, then capture.

Every stage is independent; enable only what your study needs:

  intrinsics       exact pinhole K (fx, fy, cx, cy), off-centre supported
  distortion       Brown-Conrady k1 k2 k3 p1 p2 (OpenCV convention)
  exposure         physical EV100 model: shutter, ISO, EV bias; motion blur can follow the shutter time
  noise            shot + read (+ row banding), linearised domain, deterministic per frame
  rolling shutter  first-order rotational warp from the camera's measured angular velocity; spin to see it

Requires a running simulator:
    python3 python/examples/camera_realism.py
"""

import math
import time

from flightforge import Pose, Simulator

# a 640x480 module: off-centre principal point, mild barrel lens, 5 ms shutter, 25 ms readout
CALIBRATION = {
    "K": dict(fx=460.0, fy=460.0, cx=316.2, cy=243.8),
    "dist": dict(k1=-0.21, k2=0.05, k3=0.0, p1=0.0004, p2=-0.0002),
    "noise": dict(shot_scale=3.0e-4, read_sigma=4.0e-3, row_sigma=1.0e-3),
    "readout_time": 0.025,
}


def main():
    with Simulator() as sim:
        robot = sim.spawn_robot((0.0, 0.0, 2.0), "x500")

        robot.set_camera_intrinsics(**CALIBRATION["K"])
        robot.set_camera_distortion(**CALIBRATION["dist"])
        robot.set_camera_noise(**CALIBRATION["noise"])
        robot.set_rolling_shutter(CALIBRATION["readout_time"])
        robot.set_rgb_camera_config(
            exposure=dict(manual=True, shutter_time=1.0 / 200.0, iso=200.0),
            lens=dict(motion_blur_from_shutter=True),
        )

        K, size = robot.camera_intrinsics()
        print(f"camera reports K =\n{K.round(2)}  at {size}")

        image, stamp = robot.rgb()
        print(f"static frame: {image.shape} at t={stamp:.3f}")

        # turn half a revolution while grabbing a frame: the rolling shutter shears verticals
        robot.set_pose_async(Pose.at(0.0, 0.0, 2.0, math.pi))
        time.sleep(0.05)
        image_spinning, _ = robot.rgb()

        try:
            import cv2

            cv2.imwrite("realistic_static.png", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            cv2.imwrite("realistic_spinning.png", cv2.cvtColor(image_spinning, cv2.COLOR_RGB2BGR))
            print("saved realistic_static.png and realistic_spinning.png")
        except ImportError:
            pass

        sim.remove_robot(robot)


if __name__ == "__main__":
    main()
