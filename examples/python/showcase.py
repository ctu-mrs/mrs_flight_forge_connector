from utils import get_game_mode_controller
from ueds_lidar_env import UEdsLidarEnv
from ueds_camera_env import UEdsCameraEnv
from stable_baselines3 import PPO, DQN
from ueds_learn import UEdsLearn
from stable_baselines3.common.env_checker import check_env
from datetime import datetime
from configs import env_config
import os

if __name__ == "__main__":
    game_mode_controller = get_game_mode_controller()

    ue_env = "camera_obstacle_3"

    env = UEdsCameraEnv(game_mode_controller=game_mode_controller, **env_config[ue_env], draw_move_line=True)
    check_env(env, warn=True)
    env.reset()

    models_dir = "./models"
    # model_name = "PPO_0_lidar_obstacles__1684021259_30000000.zip"
    model_name = "PPO_0_camera_obstacle__1684752094_4400000.zip"
    path = os.path.abspath(os.path.join(os.path.dirname(__file__), f'{models_dir}/{model_name}'))

    custom_objects = {
        "learning_rate": 0.0,
        "lr_schedule": lambda _: 0.0,
        "clip_range": lambda _: 0.0,
    }
    model = PPO.load(path=path, env=env, custom_objects=custom_objects)

    learn_wrapper = UEdsLearn(
        model=model, 
        verbose=1, 
        model_name=model_name,
        )

    learn_wrapper.showcase(1)
    env.close()
