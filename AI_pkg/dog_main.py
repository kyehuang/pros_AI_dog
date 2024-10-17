import rclpy
import threading
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor

from ros_receive_and_processing.AI_dog_node import AI_dog_node
from keyboard_control.keyboard_dog import keyboard_dog
from Dog_RL.RL_training_main import CustomDogEnv
from Dog_RL.PPOConfig import PPOConfig
from Dog_RL.custom_callback import CustomCallback


def init_AI_dog_node():    
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

def print_usage():
    print("modes:")
    print(" 1 -- control the dog by pressing the keyboard")
    print(" 2 -- control the dog by PPO")

def gym_env_register(node):
        gym.register(
            id = CustomDogEnv.ENV_NAME,
            entry_point = "Dog_RL.RL_training_main:CustomDogEnv",
        )
        return gym.make("CustomDogEnv-v0", node = node)

def load_or_create_model_PPO(env, PPO_Mode : str = "forward"):
    # Load or create the model
    if PPO_Mode == "forward":
        try:
            model = PPO.load(PPOConfig.LOAD_MODEL_PATH)
            env = Monitor(env)
            model.set_env(env)
            print(f"Model loaded successfully from {PPOConfig.LOAD_MODEL_PATH}")
            print(f"Model learning rate: {model.lr_schedule(1.0)}")
            print(f"Model policy network: {model.policy}")
        except FileNotFoundError:
            model = PPO("MultiInputPolicy",
                        env, verbose = 1, learning_rate = PPOConfig.LEARNING_RATE,
                        n_steps = PPOConfig.N_STEPS, batch_size = PPOConfig.BATCH_SIZE, 
                        n_epochs = PPOConfig.N_EPOCHS, device = "cuda")

            print("Model is not found. Train a new model.")


    return model

def train_model_PPO(env):
    # Train the model using PPO
    model = load_or_create_model_PPO(
        env, PPO_Mode = "forward"
    )
    custom_callback = CustomCallback(PPOConfig.SAVE_MODEL_PATH, PPOConfig.SAVE_MODEL_FREQUENCE)
    model.learn(
        total_timesteps = PPOConfig.TOTAL_TIME_STEPS,
        callback = custom_callback,
        log_interval = 1,
    )



def main(mode):
    # Initialize AI_dog_node
    node, ros_thread = init_AI_dog_node()

    # Control the dog by pressing the keyboard
    if mode == "1":
        keyboard_dog(node).run()           
    elif mode == "2":
        env = gym_env_register(node)
        train_model_PPO(env)
    else:
        print("Please type the correct numbers.")

    # Shutdown AI_dog_node
    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()
    print("[INFO] AI_dog_node has stopped.")


if __name__ == "__main__":
    print_usage()
    mode = input("Enter mode: ")
    main(mode)