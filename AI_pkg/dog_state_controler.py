"""
This file is used to control the Spot robot using the keyboard
"""
import time
import curses

from utils.gym_manger import GymManager
from IK.spot_state import spot_state_creater
from IK.spot_leg import SpotLeg
from ros_receive_and_processing import ai_dog_node


class DogStateController:
    """
    This class is used to control the Spot robot using the keyboard
    """
    def __init__(self, dog_node : ai_dog_node):
        # Initialize the AI_dog_node
        self.__node = dog_node
        self.__stdscr = None
        self.__key_count = 0

        # Initialize the joint lengths and angles
        self.joint_lengths = [0.0801, 0.1501, 0.1451]
        self.angles = [0, 0, 0]
        self.spot_leg = SpotLeg(self.joint_lengths, self.angles)

        # Initialize the base position and rotation
        self.base_position = [0, 0, 0.20]
        self.base_rotation = [0, 0, 0]
        self.base_translation = [0.3740, 0.1670, 0]

        # update flag
        self.__update_flag = True

        # key mapping
        self.key_position_mapping = {
            'w': [ 0.01, 0, 0],  # spot robot moves forward
            's': [-0.01, 0, 0], # spot robot moves backward
            'a': [0,  0.005, 0],  # spot robot moves left
            'd': [0, -0.005, 0], # spot robot moves right
            'r': [0,  0, 0.01],  # spot robot moves up
            'f': [0, 0, -0.01], # spot robot moves down
        }

        self.key_rotation_mapping = {
            't': [1, 0, 0],  # spot robot rotates around x axis
            'g': [-1, 0, 0], # spot robot rotates around x axis
            'y': [0, 1, 0],  # spot robot rotates around y axis
            'h': [0, -1, 0], # spot robot rotates around y axis
            'u': [0, 0, 1],  # spot robot rotates around z axis
            'j': [0, 0, -1], # spot robot rotates around z axis
        }


    def update(self):
        """
        Update the robot's state based on keyboard input
        """
        try:
            while self.__update_flag:
                input_char = self.__stdscr.getch()

                if input_char != curses.ERR:
                    self.__key_count += 1

                    if input_char == ord('x'):
                        self.__update_flag = False

                    elif chr(input_char) in self.key_position_mapping:
                        self.base_position[0] += self.key_position_mapping[chr(input_char)][0]
                        self.base_position[1] += self.key_position_mapping[chr(input_char)][1]
                        self.base_position[2] += self.key_position_mapping[chr(input_char)][2]
                    elif chr(input_char) in self.key_rotation_mapping:
                        self.base_rotation[0] += self.key_rotation_mapping[chr(input_char)][0]
                        self.base_rotation[1] += self.key_rotation_mapping[chr(input_char)][1]
                        self.base_rotation[2] += self.key_rotation_mapping[chr(input_char)][2]

                    elif input_char == ord('z'):
                        self.base_position = [0, 0, 0.20]
                        self.base_rotation = [0, 0, 0]
                    self.__print_basic_info(input_char)
                    self.__publish_spot_state()
                else:
                    self.__print_basic_info('')
                    time.sleep(0.1)
        except KeyboardInterrupt:
            # Handle keyboard interrupt
            print("[INFO] Keyboard interrupt detected. Exiting...")
            self.__update_flag = False
        finally:
            curses.endwin()

    def run(self):
        """
        Run the main loop for controlling the Spot robot
        """
        # Initialize the curses screen
        self.__stdscr = curses.initscr()
        print("[INFO] Keyboard control has started.")
        try:
            self.update()
        finally:
            pass
        # End the curses screen
        curses.endwin()
        print("[INFO] Keyboard control has stopped.")

    def __print_basic_info(self, key):
        # Clear the screen
        self.__stdscr.clear()

        self.__stdscr.move(0, 0)
        # Print a string at the current cursor position
        self.__stdscr.addstr(f"{self.__key_count:5d} Key '{chr(key)}' pressed!")

        # show receive data
        self.__stdscr.move(1, 0)
        self.__stdscr.addstr(f"state: {self.base_position}")
        self.__stdscr.move(2, 0)
        self.__stdscr.addstr(f"rotation: {self.base_rotation}")

    def __publish_spot_state(self):
        """
        Publish the state of the Spot robot
        """
        # Calculate the joint angles using inverse kinematics
        joint_angles = spot_state_creater(self.spot_leg, self.base_position,
                                          self.base_rotation, self.base_translation)

        # Publish the joint angles to the AI_dog_node
        self.__node.publish_spot_actions(joint_angles)
        # Print the joint angles
        self.__stdscr.move(3, 0)
        self.__stdscr.addstr(f"joint angles: {joint_angles}")


if __name__ == "__main__":
    # Initialize AI_dog_node
    node, ros_thread = GymManager().init_ai_dog_node()

    # Create an instance of DogStateController
    dog_controller = DogStateController(node)
    # Run the controller
    dog_controller.run()

    # Shutdown AI_dog_node
    GymManager().shutdown_ai_dog_node(node, ros_thread)
    print("[INFO] AI_dog_node has stopped.")
