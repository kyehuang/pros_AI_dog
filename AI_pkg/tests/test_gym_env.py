import unittest
from gym_env.rl_training_main import CustomDogEnv
from utils.gym_manger import GymManager
import numpy as np

class TestDogEnv(unittest.TestCase):
    """
    Test the CustomDogEnv class.
    """
    @classmethod
    def setUpClass(cls):
        """
        Set up the test class.
        """
        print("\n[INFO] Setting up ROS node and initializing CustomDogEnv")
        cls.gym_manager = GymManager()
        cls.node, cls.ros_thread = cls.gym_manager.init_ai_dog_node()
        cls.dog_env = CustomDogEnv(cls.node)

    @classmethod
    def tearDownClass(cls):
        """
        Tear down the test class.
        """
        print("\n[INFO] Shutting down ROS node and cleaning up CustomDogEnv")
        cls.gym_manager.shutdown_ai_dog_node(cls.node, cls.ros_thread)

    def setUp(self):
        """
        Set up the test.
        """
        print("\n[INFO] Resetting the environment")
        self.dog_env.reset()

    def test_step(self):
        """
        Test the step method of the CustomDogEnv class.
        """
        print("\n[INFO] Running test_step")
        try:
            observation, reward, done, terminal, info = self.dog_env.step(Config().action)
            print(f"[DEBUG] Observation: {observation}, Reward: {reward}, Done: {done},Terminal:{terminal}, Info: {info}")
            self.assertIsNotNone(observation, "Observation should not be None")
            self.assertIsInstance(reward, (int, np.float32), "Reward should be a number")
            self.assertIsInstance(done, bool, "Done should be a boolean")
        except Exception as e:
            self.fail(f"Test failed due to exception in test_step: {e}")

    def test_make_unity_env_reset(self):
        """
        Test the make_unity_env_reset method of the CustomDogEnv class.
        """
        print("\n[INFO] Running test_make_unity_env_reset")
        try:
            result = self.dog_env.make_unity_env_reset()
            print(f"[DEBUG] Result from make_unity_env_reset: {result}")
            self.assertTrue(result, "make_unity_env_reset should return True")
        except Exception as e:
            self.fail(f"Test failed due to exception in test_make_unity_env_reset: {e}")

    def test_reset(self):
        """
        Test the reset method of the CustomDogEnv class.
        """
        print("\n[INFO] Running test_step")
        try:
            observation, reward, done, terminal, info = self.dog_env.step(Config().action)
            print(f"[DEBUG] Observation: {observation}, Reward: {reward}, Done: {done},Terminal:{terminal}, Info: {info}")
            self.assertIsNotNone(observation, "Observation should not be None")
            self.assertIsInstance(reward, (int, np.float32), "Reward should be a number")
            self.assertIsInstance(done, bool, "Done should be a boolean")
        except Exception as e:
            self.fail(f"Test failed due to exception in test_step: {e}")

class Config:
    """
    Configuration class for the test suite.
    """
    def __init__(self):
        self.action = 1
