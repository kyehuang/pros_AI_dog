"""
This script is the main script for controlling the dog by pressing the keyboard or using PPO.
"""
from keyboard_control.keyboard_dog import KeyboardDog
from utils.gym_manger import GymManager

from gym_env.dual_leg_front_raise.ppo_config_front_raise import PPOconfigFrontRaise

def print_usage():
    """
    Print the usage of this script.
    """
    print("modes:")
    print(" 1 -- control the dog by pressing the keyboard")
    print(" 2 -- control the dog by PPO (DualLegLiftDogEnv)")
    print(" 3 -- control the dog by PPO (DualLegFrontRaiseDogEnv)")

def main(mode):
    """
    The main function for controlling the dog by pressing the keyboard or using PPO.
    """
    # Initialize AI_dog_node
    node, ros_thread = GymManager().init_ai_dog_node()

    # Control the dog by pressing the keyboard
    if mode == "1":
        keyboard_dog = KeyboardDog(node)
        keyboard_dog.run()
    elif mode == "2":
        env = GymManager().gym_env_register(node)
        GymManager().train_model_ppo(env)
    elif mode == "3":
        env = GymManager().gym_env_register(node,
                            env_name = "DualLegFrontRaiseDogEnv",
                            env_path = "gym_env.dual_leg_front_raise.rl_training_main")
        GymManager().train_model_ppo(env, ppo_config=PPOconfigFrontRaise())
    else:
        print("Please type the correct numbers.")

    # Shutdown AI_dog_node
    GymManager().shutdown_ai_dog_node(node, ros_thread)
    print("[INFO] AI_dog_node has stopped.")


if __name__ == "__main__":
    print_usage()
    chosen_mode = input("Enter mode: ")
    main(chosen_mode)
