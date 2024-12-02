from utils import get_game_mode_controller
from ueds_lidar_env import UEdsLidarEnv
from stable_baselines3 import PPO, DQN
from ueds_learn import UEdsLearn
from stable_baselines3.common.env_checker import check_env
from datetime import datetime
from configs import env_config
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor
from stable_baselines3.common.monitor import Monitor
from sb3_contrib import QRDQN
import math

def make_env(game_mode_controller, ue_env, model_name, rank):
    def _init():
        env = UEdsLidarEnv(game_mode_controller=game_mode_controller, **env_config[ue_env])
        check_env(env, warn=True)
        # env.reset()
        env = Monitor(env, f"./logs/{model_name}_{rank}")
        return env
    return _init

if __name__ == "__main__":
    game_mode_controller = get_game_mode_controller()

    ue_env_base = "lidar_obstacles_"
    now = int(datetime.now().timestamp())
    model_name = f"PPO_0_{ue_env_base}_{now}"

    num_cpu = 40
    env_fns = [make_env(game_mode_controller, f'{ue_env_base}{math.floor(i/(num_cpu/5)) + 1}', model_name, i) for i in range(num_cpu)]
    env = SubprocVecEnv(env_fns)
    # env = create_env(game_mode_controller, ue_env)

    model = PPO(
        policy="MlpPolicy", 
        env=env,
        learning_rate=3e-4
        )

    learn_wrapper = UEdsLearn(
        model=model, 
        verbose=1, 
        model_name=model_name,
        tensorboard_log="./tensorboard_logs"
        )

    learn_wrapper.learn(time_steps=30000000, save_each_n_steps=200000)

    input("Training finished. Press Enter to continue...")

    learn_wrapper.showcase(10)
