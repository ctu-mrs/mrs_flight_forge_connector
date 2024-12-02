from stable_baselines3.common.on_policy_algorithm import OnPolicyAlgorithm
from stable_baselines3.common.evaluation import evaluate_policy

class UEdsLearn:
    def __init__(
            self,
            model: OnPolicyAlgorithm,
            tensorboard_log=None,
            models_dir="./models",
            model_name="PPO_0",
            verbose=0
    ):
        self.model = model
        self.model.verbose = verbose
        if tensorboard_log is not None:
            self.model.tensorboard_log = tensorboard_log
        self.model_name = model_name
        self.models_dir = models_dir
        self.verbose = verbose

    def learn(self, time_steps=20000, save_each_n_steps=5000):
        assert time_steps % save_each_n_steps == 0

        for i in range(1, int(time_steps/save_each_n_steps)+1):
            steps = i*save_each_n_steps
            self.model.learn(total_timesteps=save_each_n_steps, tb_log_name=self.model_name, reset_num_timesteps=False)
            print(f'Learned {steps} steps. Saving...')
            self.model.save(f'{self.models_dir}/{self.model_name}_{steps}')
            print(f'Learned {steps} steps. Saved.')

    def showcase(self, episodes=10):
        mean_reward, std_reward = evaluate_policy(model=self.model, env=self.model.get_env(), n_eval_episodes=episodes)
        print("mean_reward:", mean_reward, "std_reward:", std_reward)
