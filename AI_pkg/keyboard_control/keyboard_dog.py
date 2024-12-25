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
        self.__time_interval = 0.1 # time interval

        self.key_mapping = {
            ord('o'): self.__handle_key_o,
            ord('p'): self.__handle_key_p,
            ord('i'): self.__handle_key_i,
            ord('k'): self.__handle_key_k,
            ord('l'): self.__handle_key_l,
            ord('j'): self.__handle_key_j,
            ord('1'): self.__handle_key_1,
            ord('2'): self.__handle_key_2,
            ord('3'): self.__handle_key_3
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
                    self.__print_basic_info(input_char)

                    # determine the action based on the key pressed
                    if chr(input_char) in KeyboardConfig.KEY_ACTION_MAPPING:
                        joint, value = KeyboardConfig.KEY_ACTION_MAPPING[chr(input_char)]
                        self.__joint_pos[joint] += value

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
        motor_pos = [0.0] * 12
        # LF, RB : first joint
        motor_pos[0] = self.__joint_pos[0]
        motor_pos[6] = self.__joint_pos[0]
        # LF, RB : second joint
        motor_pos[1] = self.__joint_pos[1]
        motor_pos[7] = self.__joint_pos[1]
        # LF, RB : third joint
        motor_pos[2] = self.__joint_pos[2]
        motor_pos[8] = self.__joint_pos[2]

        # RF, LB : first joint
        motor_pos[3] = self.__joint_pos[3]
        motor_pos[9] = self.__joint_pos[3]
        # RF, LB : second joint
        motor_pos[4] = self.__joint_pos[4]
        motor_pos[10] = self.__joint_pos[4]
        # RF, LB : third joint
        motor_pos[5] = self.__joint_pos[5]
        motor_pos[11] = self.__joint_pos[5]

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

    def __handle_key_o(self):
        times = 1

        for _ in range(times):
            for _, state in enumerate(KeyboardAction.FORWARD_STEP_1):
                self.__joint_pos = state
                # self.__publish_spot_actions_symmetry()
                self.__node.publish_spot_actions(state)
                time.sleep(self.__time_interval)
                self.__stdscr.refresh()

    def __handle_key_p(self):
        times = 1

        for _ in range(times):
            for _, state in enumerate(KeyboardAction.FORWARD_STEP_2):
                self.__joint_pos = state
                # self.__publish_spot_actions_symmetry()
                self.__node.publish_spot_actions(state)
                time.sleep(self.__time_interval)
                self.__stdscr.refresh()

    def __handle_key_i(self):
        times = 1

        for _ in range(times):
            for _, state in enumerate(KeyboardAction.FORWARD_STEP_3):
                self.__joint_pos = state
                # self.__publish_spot_actions_symmetry()
                self.__node.publish_spot_actions(state)
                time.sleep(self.__time_interval)
                self.__stdscr.refresh()

    def __handle_key_k(self):
        times = 1

        for _ in range(times):
            for _, state in enumerate(KeyboardAction.FORWARD_STEP_4):
                self.__joint_pos = state
                # self.__publish_spot_actions_symmetry()
                self.__node.publish_spot_actions(state)
                time.sleep(self.__time_interval)
                self.__stdscr.refresh()

    def __handle_key_l(self):
        times = 1

        for _ in range(times):
            for _, state in enumerate(KeyboardAction.FORWARD_STEP_5):
                self.__joint_pos = state
                # self.__publish_spot_actions_symmetry()
                self.__node.publish_spot_actions(state)
                time.sleep(self.__time_interval)
                self.__stdscr.refresh()

    def __handle_key_j(self):
        times = 1

        for _ in range(times):
            for _, state in enumerate(KeyboardAction.FORWARD_STEP_6):
                self.__joint_pos = state
                # self.__publish_spot_actions_symmetry()
                self.__node.publish_spot_actions(state)
                time.sleep(self.__time_interval)
                self.__stdscr.refresh()

    def __handle_key_1(self):
        times = 1

        for _ in range(times):
            for _, state in enumerate(KeyboardAction.FORWARD_STEP_7):
                self.__joint_pos = state
                # self.__publish_spot_actions_symmetry()
                self.__node.publish_spot_actions(state)
                time.sleep(self.__time_interval)
                self.__stdscr.refresh()

    def __handle_key_2(self):
        times = 1

        for _ in range(times):
            for _, state in enumerate(KeyboardAction.FORWARD_STEP_8):
                self.__joint_pos = state
                # self.__publish_spot_actions_symmetry()
                self.__node.publish_spot_actions(state)
                time.sleep(self.__time_interval)
                self.__stdscr.refresh()

    def __handle_key_3(self):
        times = 1

        for _ in range(times):
            for _, state in enumerate(KeyboardAction.FORWARD_STEP_9):
                self.__joint_pos = state
                # self.__publish_spot_actions_symmetry()
                self.__node.publish_spot_actions(state)
                time.sleep(self.__time_interval)
                self.__stdscr.refresh()
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
            curses.endwin()

        print("[INFO] Keyboard control has stopped.")
