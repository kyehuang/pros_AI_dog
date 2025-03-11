"""
This file is used to control the Spot robot using the keyboard
"""
import curses
import time
import copy
import dataclasses

from ros_receive_and_processing import ai_dog_node
from keyboard_control.keyboard_action import KeyboardAction
from keyboard_control.keyboard_config import KeyboardConfig

_STEP_MAPPING = {
    '1': KeyboardAction.FORWARD_STEP_1,
    '2': KeyboardAction.FORWARD_STEP_2,
    '3': KeyboardAction.FORWARD_STEP_3,
    '4': KeyboardAction.FORWARD_STEP_4,
    '5': KeyboardAction.FORWARD_STEP_5,
    'u': KeyboardAction.FORWARD_STEP_u,
    'j': KeyboardAction.FORWARD_STEP_j
}

@dataclasses.dataclass
class KeyboardDog:
    """
    This class is used to control the Spot robot using the keyboard
    """
    def __init__(self, node: ai_dog_node):
        # AI_dog_node instance
        self.__node = node # AI_dog_node instance
        self.__stdscr = None # curses instance
        self.__key_count = 0 # key count

        # joint position
        self.__joint_pos = copy.copy(KeyboardConfig.JOINT_INIT_POS) # joint position
        # 0 = LF, RB : first joint
        # 1 = LF, RB : second joint
        # 2 = LF, RB : third joint
        # 3 = RF, LB : first joint
        # 4 = RF, LB : second joint
        # 5 = RF, LB : third joint
        self.__time_interval = 0.02 # time interval

        self.key_mapping = {
            ord('1'): lambda: self.__handle_key_generic('1'),
            ord('2'): lambda: self.__handle_key_generic('2'),
            ord('3'): lambda: self.__handle_key_generic('3'),
            ord('4'): lambda: self.__handle_key_generic('4'),
            ord('5'): lambda: self.__handle_key_generic('5'),
            ord('u'): lambda: self.__handle_key_generic('u'),
            ord('j'): lambda: self.__handle_key_generic('j')
        }

        # update flag
        self.__update_flag = True

    def __update(self):
        """
        Update the Spot actions
    """
        try:
            while self.__update_flag:
                input_char = self.__stdscr.getch()

                if input_char != curses.ERR:
                    # show receive data
                    self.__key_count += 1

                    # determine the action based on the key pressed
                    if chr(input_char) in KeyboardConfig.KEY_ACTION_MAPPING:
                        joint, value = KeyboardConfig.KEY_ACTION_MAPPING[chr(input_char)]
                        self.__joint_pos[joint] += value /5
                        self.__joint_pos[joint + 6] += value /5

                    elif input_char == ord('z'):
                        self.__joint_pos = copy.copy(KeyboardConfig.JOINT_INIT_POS)
                    elif input_char in self.key_mapping:
                        self.key_mapping[input_char]()
                    elif input_char == ord('m'):
                        self.__joint_pos = copy.copy(KeyboardConfig.JOINT_INIT_POS)
                        self.__node.reset_unity()

                    elif input_char == ord('x'):
                        self.__update_flag = False
                    # publish spot actions
                    self.__publish_spot_actions_symmetry()
                    self.__print_basic_info(input_char)


                else:
                    self.__print_basic_info(' ')
                    time.sleep(0.1)

        except KeyboardInterrupt:
            print("[INFO] Keyboard control has stopped.")
        finally:
            curses.endwin()

    def __publish_spot_actions_symmetry(self):
        """
        Publish spot actions

        Args:
            joint_pos (list): List of joint position

        Returns:
            list: List of motor position
        """
        motor_pos = copy.copy(self.__joint_pos)

        for i in range(12):
            motor_pos[i] = float(motor_pos[i])

        self.__node.publish_spot_actions(motor_pos)

    def __print_basic_info(self, key):
        # Clear the screen
        self.__stdscr.clear()

        self.__stdscr.move(0, 0)
        # Print a string at the current cursor position
        self.__stdscr.addstr(f"{self.__key_count:5d} Key '{chr(key)}' pressed!")

        # show receive data
        self.__stdscr.move(1, 0)
        self.__stdscr.addstr(f"state: {self.__joint_pos}")

    def __handle_key_generic(self, char):
        """Handle a generic key press."""
        step_sequence = _STEP_MAPPING.get(char)
        if not step_sequence:
            # Do nothing or handle unknown keys as needed
            return
        # You can set `times` dynamically if needed
        times = 1
        for _ in range(times):
            self.__perform_forward_step(step_sequence)

    def __perform_forward_step(self, step_sequence):
        """Helper method to iterate over a step sequence and publish actions."""
        for state in step_sequence:
            # self.__publish_spot_actions_symmetry()
            self.__node.publish_spot_actions(state)
            time.sleep(self.__time_interval)
            self.__stdscr.refresh()
        self.__joint_pos = state

    def run(self):
        """
        Run keyboard control
        """
        # Initialize curses
        self.__stdscr = curses.initscr()
        print("[INFO] Keyboard control has started.")
        try:
            self.__update()
        finally:
            pass

        print("[INFO] Keyboard control has stopped.")
