import unittest

from IK.DH import get_base_pose
from IK.spot_leg import SpotLeg
from IK.spot_state import spot_state_creater


class TestSpotBasePose(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """
        Set up the test class
        """
        print("[INFO] Setting up test class")
        # cls.joint_lengths = [1.0, 2.0, 2.0]
        # cls.base_translation = [6, 4, 0]
        cls.joint_lengths = [0.0801, 0.1501, 0.1451]
        cls.base_translation = [0.3740, 0.1670, 0]
        cls.spot_leg = SpotLeg(cls.joint_lengths, [0, 0, 0])

    def check_base_pose(self, base_position, base_rotation, tol_percent=5, tol_abs=1):
        """
        Test pose with mixed relative and absolute tolerance.
        - tol_percent: 相對誤差百分比 (%)
        - tol_abs: 針對 expected 為 0 時，允許的絕對誤差
        """
        joint_angle = spot_state_creater(self.spot_leg, base_position, base_rotation,
                                        self.base_translation, [0, 0])
        result_position, result_rotation = get_base_pose(joint_angle, self.joint_lengths, self.base_translation)

        print(f"Expected Position: {base_position}, Got: {result_position}")
        print(f"Expected Rotation: {base_rotation}, Got: {result_rotation}")

        # === 位置誤差檢查 ===
        for i in range(3):
            expected = base_position[i]
            actual = result_position[i]
            if abs(expected) > 1e-6:
                percent_error = abs(actual - expected) / abs(expected) * 100
                self.assertLessEqual(percent_error, tol_percent,
                                    f"Position axis {i} exceeds percent tolerance: {percent_error:.2f}%")
            else:
                abs_error = abs(actual - expected)
                self.assertLessEqual(abs_error, tol_abs,
                                    f"Position axis {i} exceeds absolute tolerance: {abs_error:.3f}")

        # === 角度誤差檢查 ===
        for i in range(3):
            expected = base_rotation[i]
            actual = result_rotation[i]
            if abs(expected) > 1e-6:
                percent_error = abs(actual - expected) / abs(expected) * 100
                self.assertLessEqual(percent_error, tol_percent,
                                    f"Rotation axis {i} exceeds percent tolerance: {percent_error:.2f}%")
            else:
                abs_error = abs(actual - expected)
                self.assertLessEqual(abs_error, tol_abs,
                                    f"Rotation axis {i} exceeds absolute tolerance: {abs_error:.3f}")


    def test_position_center_0_0_2(self):
        self.check_base_pose([0, 0, 0.2], [0, 0, 0])

    def test_position_center_0_0_1(self):
        self.check_base_pose([0, 0, 0.1], [0, 0, 0])

    def test_position_center_0_1_2(self):
        self.check_base_pose([0, 0.1, 0.2], [0, 0, 0])

    def test_position_center_1_0_2(self):
        self.check_base_pose([0.1, 0, 0.2], [0, 0, 0])

    def test_rotation_10_0_0(self):
        self.check_base_pose([0, 0, 0.2], [10, 0, 0])

    def test_rotation_0_10_0(self):
        self.check_base_pose([0, 0, 0.2], [0, 10, 0])

    def test_rotation_0_0_10(self):
        self.check_base_pose([0, 0, 0.2], [0, 0, 10])

    def test_rotation_10_10_0(self):
        self.check_base_pose([0, 0, 0.2], [10, 10, 0])

    def test_rotation_10_0_10(self):
        self.check_base_pose([0, 0, 0.2], [10, 0, 10])

    def test_rotation_0_10_10(self):
        self.check_base_pose([0, 0, 0.2], [0, 10, 10])

    def test_rotation_20_10_20(self):
        self.check_base_pose([0, 0, 0.2], [20, 10, 20])

    def test_rotation_10_10_10(self):
        self.check_base_pose([0, 0, 0.2], [10, 10, 10])

    def test_position_center_1_0_2_rotation_10_0_0(self):
        self.check_base_pose([0.1, 0, 0.2], [0, 0, 10])
    
    def test_position_center_1_01_2_rotation_10_0_0(self):
        self.check_base_pose([0.1, 0.01, 0.2], [0, 0, 10])
    
    def test_position_center_2_01_2_rotation_10_0_0(self):
        self.check_base_pose([0.2, 0.01, 0.1], [0, 0, 10])


if __name__ == "__main__":
    unittest.main()
