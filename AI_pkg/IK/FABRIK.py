"""
This module implements the fabrik algorithm for inverse kinematics.
"""
import numpy as np

JOINT_LENGTH = [0.2, 0.2]
JOINT_NUM = 2

def fabrik(target, joints, joint_length, tolerance=1e-3, max_iter=1000):
    """
    fabrik algorithm
    :param target: target position
    :param joints: joint positions
    :param joint_length: joint lengths
    :param tolerance: tolerance
    :param max_iter: maximum number of iterations
    :return: joint positions
    """
    # Check if the target is reachable
    if np.linalg.norm(target) > sum(joint_length):
        print("Target is unreachable")
        return joints

    # Define the initial joint positions
    joints = joints.copy()

    for _ in range(max_iter):
        # Backward reaching
        joints[-1] = target
        for i in range(len(joints) - 2, -1, -1):
            direction = (joints[i + 1] - joints[i]) / np.linalg.norm(joints[i + 1] - joints[i])
            joints[i] = joints[i + 1] - direction * joint_length[i]

        # Forward reaching
        joints[0] = np.array([0.0, 0.0, 0.0])
        for i in range(len(joints) - 1):
            direction = (joints[i] - joints[i + 1]) / np.linalg.norm(joints[i] - joints[i + 1])
            joints[i + 1] = joints[i] - direction * joint_length[i]

        # Check if the distance between the end effector and the target is less than the tolerance
        if np.linalg.norm(joints[-1] - target) < tolerance:
            break
    return joints
