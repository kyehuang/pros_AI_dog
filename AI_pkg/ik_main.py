"""
This is the main file for testing the Inverse Kinematics moudule.
"""
from IK.fabrik import fabrik
from IK import config
from IK.utils import calculate_individual_pitch_yaw_roll
import numpy as np

target = np.array([0.0, 0.0, -0.3])
joints = np.array([[0.0, 0.0, 0.0], [-0.2, 0.0, 0.00001], [-0.2, 0.0, -0.20001]])
joint_length = config.JOINT_LENGTH

def angle_to_motor_angle(angles):
    """
    this function is used to convert the angle to motor angle
    """
    offset = [-20, 20]

    motor_angles = [0.0, 0,0]
    motor_angles[0] = -angles[1][1]  - 90 + offset[0]
    motor_angles[1] = angles[1][1] - angles[2][1] + 180 + offset[1]
    return motor_angles


targets = np.array([[0.0, 0.0, -0.3], [0.025, 0.0, -0.275], [0.05, 0.0, -0.25], [0.05, 0.0, -0.3]])
joints = np.array([[0.0, 0.0, 0.0], [-0.2, 0.0, 0.00001], [-0.2, 0.0, -0.20001]])
for target in targets:
    joints = fabrik(target, joints, joint_length)
    print("angle" , calculate_individual_pitch_yaw_roll(joints))
    print("angle to motor angle", angle_to_motor_angle(calculate_individual_pitch_yaw_roll(joints)))
    print("joint" , joints)
    print("target" , target)
    print("\n")

