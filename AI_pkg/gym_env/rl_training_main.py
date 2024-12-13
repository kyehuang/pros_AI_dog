"""
This module defines a custom gym environment for the AI_dog_node.
"""
import logging
import time
import numpy as np

import gymnasium as gym
from gymnasium.spaces import Discrete
from ros_receive_and_processing import AI_dog_node
from gym_env import observation_cal
from gym_env import reward_cal


logging.basicConfig(filename='episode_results.log', level=logging.INFO,
                    format='%(asctime)s - Episode: %(message)s')
logging.basicConfig(filename='robot_log.txt', level=logging.INFO,
                    format='%(asctime)s - %(message)s')

def log_episode_results(total_eisode_reward, total_step = 0):
    """
    Log the episode results
    """
    msg = f"Total episode reward: {total_eisode_reward} Total steps: {total_step}"
    logging.info(msg)

def log_robot_info(step, angle_x, angle_z, reward):
    """
    Log the robot information
    """
    msg = f"step: {step}, angle_x: {angle_x}, angle_z: {angle_z}, reward: {reward}"
    logging.info(msg)

class CustomDogEnv(gym.Env):
    """
    Custom gym environment for the AI_dog_node
    """
    ENV_NAME = 'CustomDogEnv-v0'

    def __init__(self, node : AI_dog_node):
        super().__init__()
        self.__node = node # AI_dog_node instance

        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete action
        self.action_space = Discrete(5)

        # Example for using image as input:
        self.observation_space = Discrete(144)

        self.cnt = 0
        self.total_eisode_reward = 0
        self.__pre_observation = None

        self.motor_init_states = [0.0, 135.0, 90.0, 0.0, 135.0, 90.0, 
                                  0.0, 135.0, 90.0, 0.0, 135.0, 90.0]


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

        direction = action - 1
        speed = 1.8

        new_moter_states = [
                        np.float64(self.__pre_observation["motor_states"][0] + speed * direction),
                            25.0, -30.0, 0.0, -60.0, 120.0,
                        np.float64(self.__pre_observation["motor_states"][6] + speed * direction),
                            -25.0, 30.0, 0.0, 60.0, -120.0]
        self.__node.publish_spot_actions(new_moter_states)

        ## Get observation
        observation, state = observation_cal.get_observation(self.__node)
        time.sleep(0.1)
        # Calculate the reward
        reward = reward_cal.reward_cal(observation, self.__pre_observation, self.cnt)
        self.__pre_observation = observation

        # Check if the episode is done
        self.cnt += 1
        self.total_eisode_reward += reward
        if self.cnt % 25 == 0:
            print("Observation: ", observation)
        print("Action: ", action)

        if self.cnt == 50:
            print("Total episode reward: ", self.total_eisode_reward)
            reward += 1000
            done = True
        else:
            angle_x_raw = observation["spot_angle"][0]
            angle_x =  angle_x_raw if angle_x_raw < 180 else 360 - angle_x_raw

            angle_z_raw = observation["spot_angle"][2]
            angle_z = angle_z_raw if angle_z_raw < 180 else 360 - angle_z_raw
            if (angle_x > 12 or angle_z > 12) and self.cnt > 0:

                terminal = True
                done = True
                reward -= 1000

        if done:
            print("step: ", self.cnt, " total_eisode_reward: ", self.total_eisode_reward)
            log_episode_results(self.total_eisode_reward, self.cnt)
            self.cnt = 0
            self.total_eisode_reward = 0

        return state, reward, done, terminal, info

    def reset(self, seed = None, options = None):
        """
        Reset the state of the environment to an initial state
        """
        print("Resetting environment")

        # Reset the environment
        self.make_unity_env_reset()

        # Get observation
        observation, state = observation_cal.get_observation(self.__node)
        self.__pre_observation = observation

        return state, {}

    def make_unity_env_reset(self):
        """
        Reset the Unity environment
        """
        try:
            self.__node.reset_unity()
            for _ in range(10):
                self.__node.publish_spot_actions(self.motor_init_states)
                time.sleep(0.1)
            return True
        except Exception as e:
            print(f"Error in make_unity_env_reset: {e}")
            return False
    