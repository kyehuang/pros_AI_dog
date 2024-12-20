"""
This script is the main script for controlling the dog by pressing the keyboard or using PPO.
"""
import threading
import rclpy
from ros_receive_and_processing.AI_dog_node import AI_dog_node
from keyboard_control.keyboard_dog import KeyboardDog
from utils.gym_manger import GymManager


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

def print_usage():
    """
    Print the usage of this script.
    """
    print("modes:")
    print(" 1 -- control the dog by pressing the keyboard")
    print(" 2 -- control the dog by PPO")

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
    else:
        print("Please type the correct numbers.")

    # Shutdown AI_dog_node
    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()
    print("[INFO] AI_dog_node has stopped.")


if __name__ == "__main__":
    print_usage()
    chosen_mode = input("Enter mode: ")
    main(chosen_mode)
