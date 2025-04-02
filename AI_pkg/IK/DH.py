import numpy as np
from math import cos, sin, radians, degrees

from ik_main import calculate_ik
from SpotLeg import SpotLeg

def dh_matrix(theta, d, a, alpha):
    return np.array([
        [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(joint_angles, joint_lengths, base_translation):
    """
    Calculate the forward kinematics of a robot arm using Denavit-Hartenberg parameters.
    :param joint_angles: List of joint angles in radians
    :param joint_lengths: List of joint lengths
    :return: Transformation matrix representing the end effector position and orientation
    """
    T_base_0 = dh_matrix(0, 0, base_translation[0], radians(-90))
    T_base_1 = dh_matrix(radians(90), base_translation[1], 0, radians(90))
    T_joint_1 = dh_matrix(joint_angles[0] + radians(90), 0, joint_lengths[0], radians(-90))
    T_rot_1 = dh_matrix(radians(-90), 0, 0, radians(90))
    T_joint_2 = dh_matrix(-joint_angles[1], 0, joint_lengths[1], 0)
    T_joint_3 = dh_matrix(joint_angles[2], 0, joint_lengths[2], 0)

    return T_base_0 @ T_base_1 @ T_joint_1 @ T_rot_1 @ T_joint_2 @ T_joint_3

if __name__ == "__main__":
    # Example usage
    BaseTranslation = [3, 2, 0]
    JointAngles = [radians(0), radians(90), radians(90)]
    JointLengths = [1, 2, 2]
    end_position = forward_kinematics(JointAngles, JointLengths, BaseTranslation)
    
    print("joint Base Position:")
    print(BaseTranslation)
    print("Joint Angles:")
    print(Angles := [degrees(JointAngles[0]), degrees(JointAngles[1]), degrees(JointAngles[2])])
    print("End Position:")
    print(end_position[:3, 3])  # Extract the translation part of the transformation matrix

    spotLeg = SpotLeg(JointLengths, [0, 0, 0])

    test_angle = calculate_ik(
            np.array([end_position[:3, 3]]), spotLeg, BaseTranslation
        )
    print("IK Result:")
    print(test_angle)
