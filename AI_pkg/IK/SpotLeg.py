"""
Module for managing a single leg of a Spot-style quadruped robot.
"""
import dataclasses
import numpy as np


@dataclasses.dataclass
class SpotLeg():
    """
    Represents a single leg on a Spot-style quadruped robot.
    """
    joint_lengths: list[float]
    angles: list[float]

    def calculate_ik(
            self,
            target_position: list[float],
            base_postion: list[float]
        ) -> dict[str, float]:
        """
        Calculate the joint angles needed to move this leg from a base position
        to a given target position.

        Parameters:
        -----------
        target_position : list[float]
            [x, y, z] coordinates of the target position for the leg end-effector.
        base_position : list[float]
            [x, y, z] coordinates of the leg's base.

        Returns:
        --------
        dict[str, float]
            A dictionary containing the calculated joint angles (in degrees)
            for joint_1, joint_2, and joint_3.
        """
        # Calculate offset distances
        x = -(target_position[0] - base_postion[0])
        y = target_position[1] - base_postion[1]
        z = target_position[2] - base_postion[2]

        # Intermediate distances
        c = np.sqrt(z**2 + y**2)
        d = np.sqrt(c**2 - self.joint_lengths[0]**2)

        # Joint 1 angle
        joint_1_angle = np.arctan(d/self.joint_lengths[0]) + np.arctan(y / -z)

        # Joint 3 angle
        g = np.sqrt(x**2 + d**2)
        numerator = g**2 - self.joint_lengths[1]**2 - self.joint_lengths[2]**2
        denominator = -2 * self.joint_lengths[1] * self.joint_lengths[2]
        joint_3_angle = np.arccos(numerator/denominator)

       # Joint 2 angle
        joint_2_angle = (
            np.arctan(x/d)
            + np.arcsin(self.joint_lengths[2] * np.sin(joint_3_angle)/g)
        )

        return {"joint_1_angle": np.degrees(joint_1_angle),
                "joint_2_angle": np.degrees(joint_2_angle),
                "joint_3_angle": np.degrees(joint_3_angle)}

if __name__ == "__main__":
    # Example usage
    LF_joint_lengths = [1.0, 2.0, 2.0]
    LF_angles = [0.0, 0.0, 0.0]

    leg = SpotLeg(LF_joint_lengths, LF_angles)
    target = [0, 1, 1]
    base = [0, 0, 0]

    result = leg.calculate_ik(target_position=target, base_postion=base)
    print("Calculated IK angles:", result)
