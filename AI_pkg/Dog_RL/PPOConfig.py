class PPOConfig:
    """
    The PPOConfig class encapsulates all configuration parameters for training a PPO model
    for a spider robot. 
    This includes settings related to model creation, loading, training, and environment configuration.

    Attributes:
        LEARNING_RATE (float): The learning rate for the PPO model.
        N_STEPS (int): The number of steps to run in each environment per update.
        BATCH_SIZE (int): The mini-batch size for each gradient update.
        N_EPOCHS (int): The number of epochs for optimizing the PPO model.

        LOAD_MODEL_PATH (str): The file path for loading the PPO model.
        SAVE_MODEL_PATH (str): The file path for saving the PPO model.

        SAVE_MODEL_FREQUENCE (int): The frequency (in timesteps) at which the model is saved during training.
        TOTAL_TIME_STEPS (int): The total number of timesteps for training the PPO model. 
        RESET_TOWARD_ANGLE_THRESHOLD (float) : The training step terminated threshold for training forward PPO. Represent the MAX angle tolarence between spider toward vector and z axis.

        DISTANCE_REWARD_WEIGHT (float) : The weight for calculating distance reward.
        TIME_PENALTY_WEIGHT (float) : The weight for calculating time penalty.
        ANGLE_PENALTY_WEIGHT (float) : The weight for calculating angle offset penalty.

    Note: 
        - n_updates = total_timesteps // (n_steps * n_envs)
    
    """
    
    # Create PPO model
    LEARNING_RATE: float = 0.001
    N_STEPS: int = 1024
    BATCH_SIZE: int = 64
    N_EPOCHS: int = 10

    # Load PPO model
    LOAD_MODEL_PATH: str = "./Model/PPO_spider_2024-09-11.pt"
    SAVE_MODEL_PATH: str = "./Model/PPO_spider_2024-09-11.pt"

    # training PPO model
    SAVE_MODEL_FREQUENCE: int = 1024 * 16
    TOTAL_TIME_STEPS: int = 1024 * 128 * 256 * 8
    RESET_TOWARD_ANGLE_THRESHOLD: float = 25.0 # degrees

    DISTANCE_REWARD_WEIGHT: float = 25.0 # The weight for calculating distance reward.
    TIME_PENALTY_WEIGHT: float = 0.15  # The weight for calculating time penalty.
    ANGLE_PENALTY_WEIGHT: float = 1.0 # The weight for calculating angle offset penalty.




