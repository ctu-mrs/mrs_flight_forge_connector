"""Minimal Gymnasium loop against a running FlightForge simulator.

    pip install -e python[gym,vision]
    python3 python/examples/gymnasium_random_agent.py
"""

import gymnasium

import flightforge.gym_envs  # noqa: F401 - registers FlightForge/* ids


def main():
    env = gymnasium.make("FlightForge/CameraNav-v0", render_mode="rgb_array", max_steps=100)

    observation, info = env.reset()
    print(f"observation {observation.shape}, initial distance {info['distance']:.0f} uu")

    total_reward = 0.0
    for _ in range(100):
        action = env.action_space.sample()
        observation, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        if terminated or truncated:
            break

    print(f"episode ended: reward {total_reward:.1f}, distance {info['distance']:.0f} uu, crashed={info['crashed']}")
    env.close()


if __name__ == "__main__":
    main()
