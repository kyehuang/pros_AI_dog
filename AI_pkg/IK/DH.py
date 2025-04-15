import numpy as np
from math import cos, sin, radians, degrees
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from IK.spot_leg import SpotLeg

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

def get_foot_position(joint_angles, joint_lengths, base_translation):
    """
    Calculate the foot positions of a quadruped robot given joint angles and lengths.
    :param joint_angles: List of joint angles in degrees
    :param joint_lengths: List of joint lengths
    :param base_translation: Base translation of the robot
    :return: List of foot positions for each leg
    """
    # Joint3 and Joint6 are the other side of the robot
    joint_angles[3] = joint_angles[3] + 180
    joint_angles[4] = -joint_angles[4]
    joint_angles[5] = -joint_angles[5]
    joint_angles[6] = joint_angles[6] + 180
    joint_angles[7] = -joint_angles[7]
    joint_angles[8] = -joint_angles[8]

    # Convert joint angles from degrees to radians
    joint_angles= [radians(joint_angle) for joint_angle in joint_angles]

    # Calculate the LF leg position
    LF_base_translation = [base_translation[0]/2, base_translation[1]/2, base_translation[2]/2]
    # Extract the translation part of the transformation matrix
    LF_end_position = forward_kinematics(joint_angles[0:3], joint_lengths, LF_base_translation)[:3,3]

    # Calculate the RF leg position
    RF_base_translation = [base_translation[0] / 2, -base_translation[1] / 2, base_translation[2] / 2]
    RF_end_position = forward_kinematics(joint_angles[3:6], joint_lengths, RF_base_translation)[:3, 3]

    # Calculate the RB leg position
    RB_base_translation = [-base_translation[0] / 2, -base_translation[1] / 2, base_translation[2] / 2]
    RB_end_position = forward_kinematics(joint_angles[6:9], joint_lengths, RB_base_translation)[:3, 3]

    # Calculate the LB leg position
    LB_base_translation = [-base_translation[0] / 2, base_translation[1] / 2, base_translation[2] / 2]
    LB_end_position = forward_kinematics(joint_angles[9:12], joint_lengths, LB_base_translation)[:3, 3]

    return {
        "LF": [round(x, 3) for x in LF_end_position.tolist()],
        "RF": [round(x, 3) for x in RF_end_position.tolist()],
        "RB": [round(x, 3) for x in RB_end_position.tolist()],
        "LB": [round(x, 3) for x in LB_end_position.tolist()]
    }

def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to a rotation matrix.
    :param roll: Rotation around the x-axis (in radians)
    :param pitch: Rotation around the y-axis (in radians)
    :param yaw: Rotation around the z-axis (in radians)
    :return: 3x3 rotation matrix
    """
    # Ensure angles are in radians
    roll = radians(roll)
    pitch = radians(pitch)
    yaw = radians(yaw)
    # Check if angles are in radians
    if not (isinstance(roll, float) and isinstance(pitch, float) and isinstance(yaw, float)):
        raise ValueError("Angles must be in radians")
    # Compute individual rotation matrices
    r_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    r_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    r_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Combined rotation matrix (ZYX order)
    r = r_z @ r_y @ r_x
    return r

if __name__ == "__main__":
    # Example usage: calculate the end position of a robot arm
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
    test_angle = spotLeg.calculate_ik(end_position[:3, 3], BaseTranslation)
    print("IK Result:")
    print(test_angle)

    # Example usage: Calculate the foot positions of a quadruped robot
    JointAngles = [0, 90, 90, 0, 90, 90, 0, 90, 90, 0, 90, 90]
    JointLengths = [1, 2, 2]
    BaseTranslation = [6, 4, 0]
    # JointAngles = [-13.28, 139.62, 101.61, 13.28, 139.62, 101.61, 13.28, 139.62, 101.61, -13.28, 139.62, 101.61]
    # JointLengths = [0.0801, 0.1501, 0.1451]
    # BaseTranslation = [0.3740, 0.1670, 0]
    print(get_foot_position(JointAngles, JointLengths, BaseTranslation))

    # Example usage: Convert Euler angles to a rotation matrix
    print("Euler to Rotation Matrix:")
    ROLL = 0
    PITCH = 0
    YAW = 90
    rotation_matrix = euler_to_rotation_matrix(ROLL, PITCH, YAW)
    print(rotation_matrix)
    point = np.array([1, 10, 50])
    rotated_point = rotation_matrix @ point
    print("Rotated Point:", rotated_point)
