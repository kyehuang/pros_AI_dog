"""
This module is used to manage the gym environment and
train the model using PPO.
"""
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
import rclpy
import threading

from gym_env.rl_training_main import CustomDogEnv
from gym_env.ppo_config import PPOconfig
from gym_env.custom_callback import CustomCallback
from ros_receive_and_processing.AI_dog_node import AI_dog_node

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
            entry_point = "gym_env.rl_training_main:CustomDogEnv",
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
                model = PPO.load(PPOconfig.LOAD_MODEL_PATH)
                env = Monitor(env)
                model.set_env(env)
                print(f"Model loaded successfully from {PPOconfig.LOAD_MODEL_PATH}")
                print(f"Model learning rate: {model.lr_schedule(1.0)}")
                print(f"Model policy network: {model.policy}")
            except FileNotFoundError:
                # env = Monitor(env, filename="./logs")
                model = PPO("MlpPolicy",
                            env, verbose = 1, learning_rate = PPOconfig.LEARNING_RATE,
                            n_steps = PPOconfig.N_STEPS, batch_size = PPOconfig.BATCH_SIZE,
                            n_epochs = PPOconfig.N_EPOCHS,device = "cuda")
                # policy_kwargs=PPOconfig.POLICY_KWARGS

                print("Model is not found. Train a new model.")
        return model

    @staticmethod
    def init_ai_dog_node():
        """
        Initialize AI_dog_node

        Returns:
            AI_dog_node: AI_dog_node instance
            thread: threading.Thread instance
        """
        rclpy.init()
        node = AI_dog_node()
        thread = threading.Thread(target = rclpy.spin, args = (node,))
        thread.start()
        return node, thread

    def train_model_ppo(self, env):
        """
        Train the model using PPO.
        """
        # Train the model using PPO
        model = self.load_or_create_model_ppo(
            env=env, ppo_mode = "forward"
        )
        custom_callback = CustomCallback(PPOconfig.SAVE_MODEL_PATH, PPOconfig.SAVE_MODEL_FREQUENCE)
        model.learn(
            total_timesteps = PPOconfig.TOTAL_TIME_STEPS,
            callback = custom_callback,
            log_interval = 1,
        )
