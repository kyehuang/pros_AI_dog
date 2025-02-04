"""
This file is used to trian the spot balancing using the reinforcement learning
"""
import logging
import time
import numpy as np

import gymnasium as gym
from gymnasium.spaces import Discrete, Box

from ros_receive_and_processing.ai_dog_node import AIDogNode
from gym_env.dual_leg_front_raise import observation_cal
from gym_env.dual_leg_front_raise import reward_cal
from keyboard_control.keyboard_action import KeyboardAction

logging.basicConfig(filename='episode_results.log', level=logging.INFO,
                    format='%(asctime)s - Episode: %(message)s')

def log_episode_results(total_eisode_reward, total_step = 0):
    """
    Log the episode results
    """
    msg = f"Total episode reward: {total_eisode_reward} Total steps: {total_step}"
    logging.info(msg)

class DualLegFrontRaiseDogEnv(gym.Env):
    """
    DualLegFrontRaise gym environment for the AI_dog_node
    """
    ENV_NAME = 'DogEnv-v0'

    def __init__(self, node : AIDogNode):
        super().__init__()
        self.__node = node # AI_dog_node instance

        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete action
        # self.action_space = Box(low=-10.0, high=10.0, shape=(1,), dtype=np.float32)
        self.action_space = Discrete(11)

        # Example for using image as input:
        self.observation_space = Discrete(169)

        self.cnt = 0
        self.total_eisode_reward = 0
        self.__pre_observation = None
        
        self.motor_init_states = [0.0, 135.0, 90.0, 0.0, 135.0, 90.0,
                                  0.0, 135.0, 90.0, 0.0, 135.0, 90.0]
        self.first_motor_angle = 0.0
        self.step_timestep  = 0.03
        self.targer_step = KeyboardAction.FORWARD_STEP_1.__len__()
        self.__start_time = time.time()

    def step(self, action):
        """
        Execute one time step within the environment
        """
        # Execute one time step within the environment
        # action: The action to be executed
        # Returns: observation, reward, done, info

        info = {}
        terminal = False
        done = False

        direction = action - 5
        scalar = 1.8
        self.first_motor_angle = scalar * direction
        action = float(action)
        
        new_moter_states = KeyboardAction.FORWARD_STEP_1[self.cnt]

        elisped_time = time.time() - self.__start_time
        if elisped_time < self.step_timestep:
            time.sleep(self.step_timestep - elisped_time)


        self.__node.publish_spot_actions(new_moter_states)
        self.__start_time = time.time()

        observation, state = observation_cal.get_observation(self.__node)

        # Calculate the reward
        reward = reward_cal.reward_cal(observation, self.__pre_observation, self.cnt)
        self.__pre_observation = observation

        # Check if the episode is done
        self.cnt += 1
        if self.cnt == self.targer_step:

            # print("Total episode reward: ", self.total_eisode_reward)
            reward += 100
            done = True
            # time.sleep(1)
        else:
            angle_x_raw = observation["spot_angle"][0]
            angle_x =  angle_x_raw if angle_x_raw < 180 else 360 - angle_x_raw

            angle_z_raw = observation["spot_angle"][2]
            angle_z = angle_z_raw if angle_z_raw < 180 else 360 - angle_z_raw

            angle = np.sqrt(angle_x**2 + angle_z**2)
            # print("step", self.cnt, "angle_x: ", angle_x, "angle_z: ", angle_z, "angle: ", angle)

            if (angle > 15) and self.cnt > 0:
                terminal = True
                done = True
                reward -= 500 * (1 - (self.cnt / self.targer_step))

        self.total_eisode_reward += reward
        if done:
            print("step: ", self.cnt, " total_eisode_reward: ", self.total_eisode_reward,
                  " motor_angle: ", self.first_motor_angle)
            log_episode_results(self.total_eisode_reward, self.cnt)
            self.cnt = 0
            self.total_eisode_reward = 0



        return state, reward, done, terminal, info

    def reset(self, seed = None, options = None):
        """
        Reset the state of the environment to an initial state
        """
        print("Resetting environment")
        print("+---------------------------------+")

        # Reset the environment
        self.make_unity_env_reset()
        time.sleep(1)

        # Get observation
        observation, state = observation_cal.get_observation(self.__node)
        self.__pre_observation = observation
        self.first_motor_angle = 0.0
        self.__start_time = time.time()

        return state, {}

    def make_unity_env_reset(self):
        """
        Reset the Unity environment
        """
        try:
            self.__node.reset_unity()
            for _ in range(20):
                self.__node.publish_spot_actions(self.motor_init_states)
                time.sleep(self.step_timestep)
            return True
        except ImportError as imp_err:
            print(f"Error in make_unity_env_reset: {imp_err}")
            return False
    