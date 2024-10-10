import rclpy
import threading

from ros_receive_and_processing.AI_dog_node import AI_dog_node
from keyboard_control.keyboard_dog import keyboard_dog

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

def main(mode):
    # Initialize AI_dog_node
    node, ros_thread = init_AI_dog_node()

    # Control the dog by pressing the keyboard
    if mode == "1":
        keyboard_dog(node).run()           
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