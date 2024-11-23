"""
This module is used to manage the gym environment and
train the model using PPO.
"""
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor

from Dog_RL.RL_training_main import CustomDogEnv
from Dog_RL.PPOConfig import PPOConfig
from Dog_RL.custom_callback import CustomCallback

class GymManager:
    """
    GymManager class
    """
    @staticmethod
    def gym_env_register(node):
        """
        Register the custom gym environment.

        Args:
            node: AI_dog_node instance

        Returns:
            gym.Env: gym environment instance
        """
        gym.register(
            id = CustomDogEnv.ENV_NAME,
            entry_point = "Dog_RL.RL_training_main:CustomDogEnv",
        )
        return gym.make("CustomDogEnv-v0", node = node)

    @staticmethod
    def load_or_create_model_ppo(env, ppo_mode : str = "forward"):
        """
        Load or create the model for PPO.

        Args:
            env: gym environment instance
            ppo_mode: str, default is "forward"

        Returns:
            PPO: PPO instance
        """
        # Load or create the model
        if ppo_mode == "forward":
            try:
                model = PPO.load(PPOConfig.LOAD_MODEL_PATH)
                env = Monitor(env)
                model.set_env(env)
                print(f"Model loaded successfully from {PPOConfig.LOAD_MODEL_PATH}")
                print(f"Model learning rate: {model.lr_schedule(1.0)}")
                print(f"Model policy network: {model.policy}")
            except FileNotFoundError:
                # env = Monitor(env, filename="./logs")
                model = PPO("MlpPolicy",
                            env, verbose = 1, learning_rate = PPOConfig.LEARNING_RATE,
                            n_steps = PPOConfig.N_STEPS, batch_size = PPOConfig.BATCH_SIZE,
                            n_epochs = PPOConfig.N_EPOCHS,device = "cuda")
                # policy_kwargs=PPOConfig.POLICY_KWARGS

                print("Model is not found. Train a new model.")
        return model

    def train_model_ppo(self, env):
        """
        Train the model using PPO.
        """
        # Train the model using PPO
        model = self.load_or_create_model_ppo(
            env=env, ppo_mode = "forward"
        )
        custom_callback = CustomCallback(PPOConfig.SAVE_MODEL_PATH, PPOConfig.SAVE_MODEL_FREQUENCE)
        model.learn(
            total_timesteps = PPOConfig.TOTAL_TIME_STEPS,
            callback = custom_callback,
            log_interval = 1,
        )
