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
        # self.__forward_1 = [[0.0, 25.0, 60.0, 0.0, 25.0, 60.0],
        #                     [0.0, 10.0, 80.0, 0.0, 25.0, 60.0],
        #                     [0.0,  5.0, 90.0, 0.0, 25.0, 60.0],
        #                     [0.0, 25.0, 60.0, 0.0, 25.0, 60.0],
        #                     [0.0, 25.0, 60.0, 0.0, 25.0, 60.0],
        #                     [0.0, 25.0, 60.0, 0.0, 10.0, 80.0],
        #                     [0.0, 25.0, 60.0, 0.0,  5.0, 90.0],
        #                     [0.0, 25.0, 60.0, 0.0, 25.0, 60.0]]

        # self.__forward_2 = [[0.0, 25.0, 60.0, 0.0, 25.0, 30],
        #                     [0.0, 30.0, 100.0, 0.0, 25.0, 30],
        #                     [0.0, 40.0, 100.0, 0.0, 25.0, 30],
        #                     [0.0, 45.0, 100.0, 0.0, 25.0, 30],
        #                     [0.0, 50.0, 100.0, 0.0, 25.0, 30],
        #                     [0.0, 55.0, 100.0, 0.0, 25.0, 30],
        #                     [0.0, 60.0, 100.0, 0.0, 25.0, 30],
        #                     [0.0, 65.0, 100.0, 0.0, 25.0, 30]]
        
        # self.__forward_2 = [[0.0, 25.0, 30, 0.0, 25.0, 30.0],
        #                     [0.0, 25.0, 30, 0.0, 30.0, 100.0],
        #                     [0.0, 25.0, 30, 0.0, 40.0, 100.0],
        #                     [0.0, 25.0, 30, 0.0, 45.0, 100.0],
        #                     [0.0, 25.0, 30, 0.0, 50.0, 100.0],
        #                     [0.0, 25.0, 30, 0.0, 55.0, 100.0],
        #                     [0.0, 25.0, 30, 0.0, 60.0, 100.0],
        #                     [0.0, 25.0, 30, 0.0, 65.0, 100.0]]

        # self.__forward_2 = [[0.0, 25.0, 30, 0.0, 25.0, 30.0],
        #                     [-5.0, 25.0, 30, 0.0, 30.0, 100.0],
        #                     [-5.0, 25.0, 30, 0.0, 40.0, 100.0],
        #                     [-11.0, 25.0, 30, 0.0, 45.0, 100.0],
        #                     [-11.0, 25.0, 30, 0.0, 50.0, 100.0],
        #                     [-11.0, 25.0, 30, 0.0, 55.0, 100.0],
        #                     [-11.0, 25.0, 30, 0.0, 60.0, 100.0],
        #                     [-11.0, 25.0, 30, 0.0, 65.0, 100.0]]
        # backwards_position = np.array([[-0.02, 0.0, -0.3],
        #                        [-0.04, 0.0, -0.3], #
        #                        [-0.08, 0.0, -0.3],
        #                        [-0.10, 0.0, -0.3], #
        #                        [-0.12, 0.0, -0.3],
        #                        [-0.14, 0.0, -0.3], #
        #                        [-0.15, 0.0, -0.3]])

        # forward_position = np.array([[-0.15, 0.0, -0.30],
        #                      [-0.15, 0.0, -0.25], #
        #                      [-0.15, 0.0, -0.20],
        #                      [-0.07, 0.0, -0.20], #
        #                      [-0.00, 0.0, -0.20],
        #                      [-0.00, 0.0, -0.25], #
        #                      [-0.00, 0.0, -0.30]])
        self.__forward_1 =  [
            [0, 43.006609640748906, 53.31268355531256, 0, 27.552302298190625, 68.0346955223863],
            [0, 47.48996841125273, 59.40094442916804, 0, 28.68324159815043, 67.77298221763472],
            [0, 51.97332718175656, 65.48920530302352, 0, 29.814180898110237, 67.51126891288315],
            [0, 56.456685952260386, 71.577466176879, 0, 30.945120198070043, 67.24955560813157],
            [0, 60.891758261165485, 76.570364400369, 0, 32.869235469020744, 66.19712121650089],
            [0, 65.32683057007058, 81.56326262385902, 0, 34.79335073997144, 65.14468682487022],
            [0, 69.76190287897568, 86.55616084734902, 0, 36.71746601092214, 64.09225243323954],
            [0, 65.95871717276037, 90.7376588338738, 0, 37.49244893552701, 63.299436728427985],
            [0, 62.15553146654507, 94.91915682039856, 0, 38.26743186013188, 62.506621023616425],
            [0, 58.35234576032977, 99.10065480692333, 0, 39.04241478473675, 61.71380531880487],
            [0, 52.53355938296733, 100.34947751270481, 0, 39.681301427300845, 60.73856913589153],
            [0, 46.71477300560489, 101.59830021848627, 0, 40.32018806986493, 59.76333295297818],
            [0, 40.895986628242454, 102.84712292426775, 0, 40.95907471242903, 58.78809677006484],
            [0, 38.22827039530749, 97.41699990373043, 0, 41.452498090032705, 57.62179367603314],
            [0, 35.560554162372526, 91.98687688319309, 0, 41.94592146763638, 56.45549058200144],
            [0, 32.89283792943756, 86.55675386265577, 0, 42.43934484524006, 55.28918748796974],
            [0, 29.885313052361795, 80.47092211845187, 0, 42.628896983132286, 54.630951256932384],
            [0, 26.877788175286028, 74.38509037424795, 0, 42.81844912102452, 53.972715025895035],
            [0, 23.87026329821026, 68.29925863004405, 0, 43.00800125891675, 53.31447879485768],
            [0, 27.552302298190625, 68.0346955223863, 0, 43.006609640748906, 53.31268355531256],
            [0, 28.68324159815043, 67.77298221763472, 0, 47.48996841125273, 59.40094442916804],
            [0, 29.814180898110237, 67.51126891288315, 0, 51.97332718175656, 65.48920530302352],
            [0, 30.945120198070043, 67.24955560813157, 0, 56.456685952260386, 71.577466176879],
            [0, 32.869235469020744, 66.19712121650089, 0, 60.891758261165485, 76.570364400369],
            [0, 34.79335073997144, 65.14468682487022, 0, 65.32683057007058, 81.56326262385902],
            [0, 36.71746601092214, 64.09225243323954, 0, 69.76190287897568, 86.55616084734902],
            [0, 37.49244893552701, 63.299436728427985, 0, 65.95871717276037, 90.7376588338738],
            [0, 38.26743186013188, 62.506621023616425, 0, 62.15553146654507, 94.91915682039856],
            [0, 39.04241478473675, 61.71380531880487, 0, 58.35234576032977, 99.10065480692333],
            [0, 39.681301427300845, 60.73856913589153, 0, 52.53355938296733, 100.34947751270481],
            [0, 40.32018806986493, 59.76333295297818, 0, 46.71477300560489, 101.59830021848627],
            [0, 40.95907471242903, 58.78809677006484, 0, 40.895986628242454, 102.84712292426775],
            [0, 41.452498090032705, 57.62179367603314, 0, 38.22827039530749, 97.41699990373043],
            [0, 41.94592146763638, 56.45549058200144, 0, 35.560554162372526, 91.98687688319309],
            [0, 42.43934484524006, 55.28918748796974, 0, 32.89283792943756, 86.55675386265577],
            [0, 42.628896983132286, 54.630951256932384, 0, 29.885313052361795, 80.47092211845187],
            [0, 42.81844912102452, 53.972715025895035, 0, 26.877788175286028, 74.38509037424795],
            [0, 43.00800125891675, 53.31447879485768, 0, 23.87026329821026, 68.29925863004405]]


    def __handle_key_o(self):
        times = 10

        for _ in range(times):
            for i in range(len(self.__forward_1)):
                self.__joint_pos = self.__forward_1[i]
                self.__publish_spot_actions()
                setp_length = 38
                scale = 1/3
                time.sleep((1 / setp_length)  * scale)
                # time.sleep(0.5)
            
                
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
