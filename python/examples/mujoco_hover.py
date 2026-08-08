"""MuJoCo dynamics driving the FlightForge renderer.

A minimal quadrotor (free body + four thrust actuators) hovers under a naive
altitude PD controller; every physics step is mirrored into FlightForge, which
renders it and serves the cameras.

    pip install -e python[mujoco,vision]
    python3 python/examples/mujoco_hover.py
"""

import numpy as np

from flightforge import MujocoBridge, Simulator

QUADROTOR_MJCF = """
<mujoco model="minimal_quadrotor">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="drone" pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.12 0.12 0.03" mass="1.2"/>
      <site name="rotor_fl" pos=" 0.1  0.1 0"/>
      <site name="rotor_fr" pos=" 0.1 -0.1 0"/>
      <site name="rotor_rl" pos="-0.1  0.1 0"/>
      <site name="rotor_rr" pos="-0.1 -0.1 0"/>
    </body>
  </worldbody>
  <actuator>
    <motor site="rotor_fl" gear="0 0 5 0 0 0"/>
    <motor site="rotor_fr" gear="0 0 5 0 0 0"/>
    <motor site="rotor_rl" gear="0 0 5 0 0 0"/>
    <motor site="rotor_rr" gear="0 0 5 0 0 0"/>
  </actuator>
</mujoco>
"""


def main():
    import mujoco

    sim = Simulator()
    drone = sim.spawn_drone((0.0, 0.0, 200.0), "x500")

    model = mujoco.MjModel.from_xml_string(QUADROTOR_MJCF)
    bridge = MujocoBridge(drone, model, body="drone", world_origin_uu=sim.world_origin(), substeps=5)

    target_altitude = 2.0  # meters
    kp, kd = 8.0, 4.0

    for step in range(2000):
        z = bridge.data.qpos[2]
        vz = bridge.data.qvel[2]
        thrust = np.clip(0.25 * (1.2 * 9.81 + kp * (target_altitude - z) - kd * vz) / 5.0, 0.0, 1.0)
        bridge.data.ctrl[:] = thrust

        bridge.step()

        if step % 200 == 0:
            image, stamp = drone.rgb()
            print(f"t={bridge.sim_time:6.2f}s  z={z:5.2f} m  frame {image.shape} @ {stamp:.3f}")

    sim.remove_drone(drone)
    drone.close()
    sim.close()


if __name__ == "__main__":
    main()
