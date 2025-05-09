"""
This script is used to test the spot_state_creater class
"""
import unittest

from IK.spot_state import spot_state_creater
from IK.spot_leg import SpotLeg
from IK.DH import get_foot_position

class TestDogStateController(unittest.TestCase):
    """
    Test the spot_state_creater class
    """
    @classmethod
    def setUpClass(cls):
        """
        Set up the test class
        """
        print("\n[INFO] Setting up test class")
        cls.joint_lengths = [1.0, 2.0, 2.0]
        cls.base_translation = [6, 4, 0]
        cls.spot_leg = SpotLeg(cls.joint_lengths, [0, 0, 0])

    def test_position_center(self):
        """
        Test the spot_state_creater class with the center position
        """
        base_position = [0, 0, 2]
        result = spot_state_creater(self.spot_leg, base_position, [0, 0, 0],
                                    self.base_translation, [0, 0])
        expected = [0.0, 150.0, 120.0] * 4
        self.assertEqual(result, expected)

        # Calculate the foot positions
        # using the forward kinematics function
        # to get the foot positions
        foot_position = get_foot_position(result, self.joint_lengths, self.base_translation)
        expected_foot_position = {
            'LF': [3.0, 3.0, -2.0],
            'RF': [3.0, -3.0, -2.0],
            'RB': [-3.0, -3.0, -2.0],
            'LB': [-3.0, 3.0, -2.0]
        }
        self.assertEqual(foot_position, expected_foot_position)




    def test_position_back(self):
        """
        Test the spot_state_creater class with the left position
        """
        base_position = [-1, 0, 2]
        result = spot_state_creater(self.spot_leg, base_position, [0, 0, 0],
                                    self.base_translation, [0, 0])
        expected = [0.0, 119.45, 112.02] * 4  # ← 你需要自己填入預期結果
        print(result)
        self.assertEqual(result, expected)

    def test_position_high(self):
        """
        Test the spot_state_creater class with the high position
        """
        base_position = [0, 0, 3]
        result = spot_state_creater(self.spot_leg, base_position, [0, 0, 0],
                                    self.base_translation, [0, 0])
        expected = [0.0, 131.41, 82.82] * 4  # ← 預期角度
        self.assertEqual(result, expected)

    def test_position_left(self):
        """
        Test the spot_state_creater class with the left position
        """
        base_position = [0, -1, 2]
        result = spot_state_creater(self.spot_leg, base_position, [0, 0, 0],
                                    self.base_translation, [0, 0])
        expected = [24.3, 138.59, 97.18, 30.0, 154.34, 128.68,
                    30.0, 154.34, 128.68, 24.3, 138.59, 97.18]
        self.assertEqual(result, expected)

    def test_position_right(self):
        """
        Test the spot_state_creater class with the right position
        """
        base_position = [0, 1, 2]
        result = spot_state_creater(self.spot_leg, base_position, [0, 0, 0],
                                    self.base_translation, [0, 0])
        expected = [-30.0, 154.34, 128.68, -24.3, 138.59, 97.18,
                    -24.3, 138.59, 97.18, -30.0, 154.34, 128.68]
        self.assertEqual(result, expected)

    def test_rotation_x(self):
        """
        Test the spot_state_creater class with the rotation around x axis
        """
        base_position = [0, 0, 2]
        base_rotation = [1, 0, 0]
        result = spot_state_creater(self.spot_leg, base_position,
                                    base_rotation, self.base_translation, [0, 0])
        expected = [0.01, 149.42, 118.84, -0.01, 150.57, 121.15,
                    -0.01, 150.57, 121.15, 0.01, 149.42, 118.84]
        self.assertEqual(result, expected)
