from utils import get_game_mode_controller
from controllers import CameraCaptureModeEnum
from ueds_camera_env import UEdsCameraEnv
from stable_baselines3 import PPO, DQN
from ueds_learn import UEdsLearn
from stable_baselines3.common.env_checker import check_env
from datetime import datetime
from configs import env_config
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor
from stable_baselines3.common.monitor import Monitor
from sb3_contrib import QRDQN
import math

def get_goal_direction(img):
    lower = np.array([150, 230, 100])
    upper = np.array([200, 255, 190])

    mask = cv2.inRange(img, lower, upper)
    
    bbox = Image.fromarray(mask).getbbox()
    if bbox is not None:
        
        bbox_height = (bbox[1] + bbox[3]) / 2
        bbox_width = (bbox[0] + bbox[2]) / 2
        
        height_mid = img.shape[0] / 2
        weight_mid = img.shape[1] / 2
        
        distance = np.array([(bbox_height - height_mid) / height_mid, (bbox_width - weight_mid) / weight_mid])
        
        return distance
    return None

def make_env(game_mode_controller, ue_env, model_name, rank):
    def _init():
        env = UEdsCameraEnv(game_mode_controller=game_mode_controller, **env_config[ue_env])
        check_env(env, warn=True)
        # env.reset()
        env = Monitor(env, f"./logs/{model_name}_{rank}")
        return env
    return _init

if __name__ == "__main__":
    game_mode_controller = get_game_mode_controller()
    res = game_mode_controller.SetCameraCaptureMode(CameraCaptureModeEnum.CAPTURE_ON_MOVEMENT)
    assert res is True

    ue_env_base = "camera_obstacle_"
    now = int(datetime.now().timestamp())
    model_name = f"PPO_0_{ue_env_base}_{now}"

    num_cpu = 8
    env_fns = [make_env(game_mode_controller, f'{ue_env_base}{math.floor(i/(num_cpu/2)) + 1}', model_name, i) for i in range(num_cpu)]
    env = SubprocVecEnv(env_fns)
    # env = create_env(game_mode_controller, ue_env)

    model = PPO(
        policy="MlpPolicy", 
        env=env,
        learning_rate=0.0001
        )

    learn_wrapper = UEdsLearn(
        model=model, 
        verbose=1, 
        model_name=model_name,
        tensorboard_log="./tensorboard_logs"
        )

    learn_wrapper.learn(time_steps=3000000, save_each_n_steps=200000)

    input("Training finished. Press Enter to continue...")

    learn_wrapper.showcase(10)

    # init_steps = 3000000
    # steps = 200000
    # model = PPO.load(f'./models/PPO_0_camera_obstacle__1684752094_{init_steps}.zip')
    # model.set_env(env)
    # model.tensorboard_log = "./tensorboard_logs"
    # for i in range(10):
    #     model.learn(total_timesteps=steps, tb_log_name="PPO_0_camera_obstacle__1684752094", reset_num_timesteps=False)
    #     model.save(f'./models/PPO_0_camera_obstacle__1684752094_{init_steps + (i+1)*steps}')

# if __name__ == '__main__':
#     game_mode_controller = get_game_mode_controller()
#     res = game_mode_controller.SetCameraCaptureMode(CameraCaptureModeEnum.CAPTURE_ON_MOVEMENT)
#     assert res is True

#     env = UEdsCameraEnv(game_mode_controller=game_mode_controller, **env_config['camera_obstacle_1'])
#     env.reset()

#     while True:
#         print('Select action:')
#         action = int(input())
#         obs, reward, done, info = env.step(action)
#         print(f'OBS: {obs}')
#         print(f"Reward: {reward}")
#         print(f"Done: {done}")
#         print(f"Info: {info}")
#         print('------------------')