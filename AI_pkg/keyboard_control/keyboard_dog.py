import curses
import threading
import time
import copy
from ros_receive_and_processing import AI_dog_node

class keyboard_dog:
    def __init__(self, node: AI_dog_node):        
        # AI_dog_node instance
        self.__node = node # AI_dog_node instance
        self.__stdscr = None # curses instance
        self.__key_count = 0 # key count

        # joint position        
        self.__joint_init_pos = [0.0, 25.0, 60.0, 0.0, 25.0, 60.0] # initial joint position
        self.__motor_pos = [0, 40, -50, 0, -25, 60, 0, -25, 60, 0, 25, -60]
        self.__joint_pos = copy.copy(self.__joint_init_pos) # joint position
        # 0 = LF, RB : first joint
        # 1 = LF, RB : second joint
        # 2 = LF, RB : third joint
        # 3 = RF, LB : first joint
        # 4 = RF, LB : second joint
        # 5 = RF, LB : third joint

        # update flag
        self.__update_flag = True

        # init routine
        self.__init_dog_routine()

    def __update(self):
        try:            
            while self.__update_flag:
                c = self.__stdscr.getch()

                if c != curses.ERR:
                    # show receive data
                    self.__key_count += 1
                    self.__print_basic_info(c)

                    # determine the action based on the key pressed
                    if c == ord('q'):
                        self.__joint_pos[0] += 5.0
                    elif c == ord('a'):
                        self.__joint_pos[0] -= 5.0
                    elif c == ord('w'):
                        self.__joint_pos[1] += 5.0
                    elif c == ord('s'):
                        self.__joint_pos[1] -= 5.0
                    elif c == ord('e'):
                        self.__joint_pos[2] += 5.0
                    elif c == ord('d'):
                        self.__joint_pos[2] -= 5.0
                    elif c == ord('r'):
                        self.__joint_pos[3] += 5.0
                    elif c == ord('f'):
                        self.__joint_pos[3] -= 5.0
                    elif c == ord('t'):
                        self.__joint_pos[4] += 5.0
                    elif c == ord('g'):
                        self.__joint_pos[4] -= 5.0
                    elif c == ord('y'):
                        self.__joint_pos[5] += 5.0
                    elif c == ord('h'):
                        self.__joint_pos[5] -= 5.0
                    elif c == ord('z'):
                        self.__joint_pos = copy.copy(self.__joint_init_pos)
                    elif c == ord('o'):
                        self.__handle_key_o()

                    elif c == ord('x'):
                        self.__update_flag = False
                    # publish spot actions 
                    self.__publish_spot_actions()

                else:
                    self.__print_basic_info(' ')  
                    time.sleep(0.1)

        except KeyboardInterrupt:
            print("[INFO] Keyboard control has stopped.")
            pass
        finally:
            curses.endwin()
    
    def __publish_spot_actions(self):
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
        motor_pos[6] = -self.__joint_pos[0]
        # LF, RB : second joint
        motor_pos[1] = self.__joint_pos[1]
        motor_pos[7] = -self.__joint_pos[1]
        # LF, RB : third joint
        motor_pos[2] = -self.__joint_pos[2]
        motor_pos[8] = self.__joint_pos[2]
        
        # RF, LB : first joint
        motor_pos[3] = self.__joint_pos[3]
        motor_pos[9] = -self.__joint_pos[3]
        # RF, LB : second joint
        motor_pos[4] = -self.__joint_pos[4]
        motor_pos[10] = self.__joint_pos[4]
        # RF, LB : third joint
        motor_pos[5] = self.__joint_pos[5]
        motor_pos[11] = -self.__joint_pos[5]
        
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
        
    def __init_dog_routine(self):
        """
        Initialize dog routine
        """
        # init dog routine
        # self.__forward_First_Time_displacement   = [[0, 10, -50, 0, 0, 0, 0, -10, 50, 0, 0, 0],
        #                                           [0, -10, 20, 0, 0, 0, 0, 10, -20, 0, 0, 0],
        #                                           [0, -10, 20, 0, 0, 0, 0, 10, -20, 0, 0, 0],
        #                                           [0, -20, 20, 0, 0, 0, 0, 20, -20, 0, 0, 0]]

        # self.__forward_routine_left_displacement = [[0, 10, -50, 0, -10, 5, 0, -10, 50, 0, 10, -5],
        #                                           [0, -10, 20, 0, -10, 5, 0, 10, -20, 0, 10, -5],
        #                                           [0, -10, 20, 0, -10, 0, 0, 10, -20, 0, 10, 0],
        #                                           [0, -20, 20, 0, 0, 0, 0, 20, -20, 0, 0, 0]]

        # self.__forward_routine_right_displacement = [[0, 10, -5, 0, -10, 50, 0, -10, 5, 0, 10, -50],
        #                                           [0, 10, -5, 0, 10, -20, 0, -10, 5, 0, -10, 20],
        #                                           [0, 10, 0, 0, 10, -20, 0, -10, 0, 0, -10, 20],
        #                                           [0, 0, 0, 0, 20, -20, 0, 0, 0, 0, -20, 20]]

        # self.__forward_First_Time_displacement   = [[0.0, 20.0, 60.0, 0.0, 20.0, 60.0],
        #                                             [0.0, 20.0, 60.0, 0.0, 20.0, 60.0],
        #                                             [0.0, 20.0, 60.0, 0.0, 20.0, 60.0],
        #                                             [0.0, 20.0, 60.0, 0.0, 20.0, 60.0]]

        # self.__forward_routine_left_displacement = [[0.0, 20.0, 60.0, 0.0, 20.0, 60.0],
        #                                             [0.0, 10.0, 80.0, 0.0, 20.0, 60.0],
        #                                             [0.0,  5.0, 90.0, 0.0, 20.0, 60.0],
        #                                             [0.0, 15.0, 90.0, 0.0, 20.0, 60.0],
        #                                             [0.0, 20.0, 60.0, 0.0, 20.0, 60.0]]

        # self.__forward_routine_right_displacement = [[0.0, 20.0, 60.0, 0.0, 20.0, 60.0],
        #                                              [0.0, 20.0, 60.0, 0.0, 10.0, 80.0],
        #                                              [0.0, 20.0, 60.0, 0.0,  5.0, 90.0],
        #                                              [0.0, 20.0, 60.0, 0.0, 15.0, 90.0],
        #                                              [0.0, 20.0, 60.0, 0.0, 20.0, 60.0]]
        # self.__joint_init_pos = [0.0, 20.0, 60.0, 0.0, 25.0, 60.0]
        self.__forward_1 = [[0.0, 25.0, 60.0, 0.0, 25.0, 60.0],
                            [0.0, 10.0, 80.0, 0.0, 25.0, 60.0],
                            [0.0,  5.0, 90.0, 0.0, 25.0, 60.0],                                                    
                            [0.0, 25.0, 60.0, 0.0, 25.0, 60.0], 
                            [0.0, 25.0, 60.0, 0.0, 25.0, 60.0],
                            [0.0, 25.0, 60.0, 0.0, 10.0, 80.0],
                            [0.0, 25.0, 60.0, 0.0,  5.0, 90.0],                                                     
                            [0.0, 25.0, 60.0, 0.0, 25.0, 60.0]]

        self.__forward_2 = [[0.0, 25.0, 60.0, 0.0, 25.0, 60.0],
                            [0.0, 25.0, 60.0, 0.0, 10.0, 80.0],
                            [0.0, 25.0, 60.0, 0.0,  5.0, 90.0],                                                     
                            [0.0, 25.0, 60.0, 0.0, 25.0, 60.0],
                            [0.0, 25.0, 60.0, 0.0, 25.0, 60.0],
                            [0.0, 10.0, 80.0, 0.0, 25.0, 60.0],
                            [0.0,  5.0, 90.0, 0.0, 25.0, 60.0],                                                    
                            [0.0, 25.0, 60.0, 0.0, 25.0, 60.0]]

        
        

    def __handle_key_o(self):        
        times = 10

        for _ in range(times):
            for i in range(len(self.__forward_2)):                                                            
                self.__joint_pos = self.__forward_2[i]
                self.__publish_spot_actions()
                time.sleep(0.1)
            
                
            # if(self.__flag_First_move == True):
            #     for i in range(len(self.__forward_routine_left_displacement)):                                                            
            #         self.__joint_pos = self.__forward_First_Time_displacement[i]
            #         self.__publish_spot_actions()
            #     self.__flag_First_move = False
            # elif(self.__flag_which_leg == False):
            #     for i in range(len(self.__forward_routine_left_displacement)):                                                            
            #         self.__joint_pos = self.__forward_routine_left_displacement[i]
            #         self.__publish_spot_actions()
            #     self.__flag_which_leg = True
            # else:
            #     for i in range(len(self.__forward_routine_left_displacement)):                                                            
            #         self.__joint_pos = self.__forward_routine_right_displacement[i]
            #         self.__publish_spot_actions()
            #     self.__flag_which_leg = False
                                                                                                
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
