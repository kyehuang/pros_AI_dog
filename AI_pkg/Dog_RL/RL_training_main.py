import numpy as np
import time
import gymnasium as gym
from gymnasium import spaces

from ros_receive_and_processing import AI_dog_node
from Dog_RL import observation_cal
from Dog_RL import reward_cal

import logging

logging.basicConfig(filename='episode_results.log', level=logging.INFO, 
                    format='%(asctime)s - Episode: %(message)s')

def log_episode_results(total_eisode_reward):
    logging.info(f"Total episode reward: {total_eisode_reward}")

class CustomDogEnv(gym.Env):
    ENV_NAME = 'CustomDogEnv-v0'

    def __init__(self, node : AI_dog_node):
        super(CustomDogEnv, self).__init__()
        self.__node = node # AI_dog_node instance

        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete action        
        self.action_space = spaces.Box(low=0, high=1, shape=(12,), dtype=np.float32)

        # Example for using image as input:
        self.observation_space = spaces.Dict({
            'motor_states': spaces.Box(low=-1, high=1, shape=(12,), dtype=np.float32),
            'spot_angle': spaces.Box(low=-1, high=1, shape=(3,), dtype=np.float32),
            # 'spot_pos': spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
            'distance_to_stright_line': spaces.Box(low=-np.inf, high=np.inf, shape=(1,), dtype=np.float32),
            "distance_to_goal": spaces.Box(low=-np.inf, high=np.inf, shape=(1,), dtype=np.float32),
            "angle_to_goal": spaces.Box(low=-np.inf, high=np.inf, shape=(1,), dtype=np.float32),
        })

        self.cnt = 0
        self.total_eisode_reward = 0
        self.__pre_observation = None


    def step(self, action):
        # Execute one time step within the environment
        # action: The action to be executed
        # Returns: observation, reward, done, info
        info = {}
        terminal = False
        done = False

        float_action = list(map(float, action))
        
        scaled_action = [a * 360 for a in float_action]

        self.__node.publish_spot_actions(scaled_action)
        time.sleep(0.05)
        
        observation = observation_cal.get_observation(self.__node)
        # Check if the observation is within the observation space
        self.observation_space.contains(observation)

        reward = reward_cal.reward_cal(observation, self.__pre_observation)
        self.__pre_observation = observation
        self.total_eisode_reward += reward

        self.cnt += 1
        if self.cnt % 100 == 0:
            print("Observation: ", observation)
            print("Action: ", action)
        
        if self.cnt == 500:
            print("Total episode reward: ", self.total_eisode_reward)
            log_episode_results(self.total_eisode_reward)
            self.cnt = 0
            self.total_eisode_reward = 0
            done = True

        if abs(observation["spot_angle"][2] - 90) < 10:
            terminal = True
            done = True
        

        return observation, reward, done, terminal, info

    def reset(self, seed = None, options = None):   
        # Reset the state of the environment to an initial state
        print("Resetting environment")

        # Reset the environment
        self.__node.reset_unity()

        time.sleep(0.5)

        # Get observation
        observation = observation_cal.get_observation(self.__node)
        self.__pre_observation = observation
        time.sleep(0.5)
        return observation, {}
