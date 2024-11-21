"""
This is the main file for testing the Inverse Kinematics moudule.
"""
from IK.fabrik import fabrik
from IK import config
from IK.utils import calculate_individual_pitch_yaw_roll, angle_to_motor_angle
import numpy as np


def calculate_ik(positions, joints=None, joint_length=None):
    """
    this function is used to calculate the inverse kinematics

    Parameters:
        positions (list): the list of positions of the end effector

    Returns:
        list: the list of motor angles
    """
    # init the joints and joint_length
    if joints is None:
        joints = config.JOINTS
    if joint_length is None:
        joint_length = config.JOINT_LENGTH

    # calculate the motor angles
    motor_angles = []
    print("positions", positions)
    for position in positions:
        joints = fabrik(position, joints, joint_length, tolerance=1e-5, max_iter=1000)
        motor_angles.append(angle_to_motor_angle(calculate_individual_pitch_yaw_roll(joints)))
        print("joints", joints)
        print("motor angles", calculate_individual_pitch_yaw_roll(joints))
        print("=====================================")
    return motor_angles

def linear_interpolate_path(joint_angles, points_per_segment=10):
    """
    Perform forward linear interpolation on joint angle data without including the return path 
    from the last position back to the first.

    Parameters:
        joint_angles (list): A 2D list of initial joint angle data.
        points_per_segment (int): Number of interpolation points per segment.

    Returns:
        np.ndarray: Interpolated joint angle data (excluding the return path).
        int: Total number of interpolated points for subsequent use.
    """
    interpolated_path = []

    for i in range(len(joint_angles) - 1):
        start_angles = np.array(joint_angles[i])
        end_angles = np.array(joint_angles[i + 1])
        interpolated_segment = [
            start_angles + (end_angles - start_angles) * t / (points_per_segment + 1)
            for t in range(points_per_segment + 1)
        ]
        interpolated_path.extend(interpolated_segment)

    interpolated_path.append(joint_angles[-1])  # Append the final point
    return np.array(interpolated_path), len(interpolated_path)

# backwards_position = np.array([[ -0.0, 0.0, -0.3],
#                                [-0.07, 0.0, -0.3],
#                                [-0.10, 0.0, -0.3],
#                                [-0.15, 0.0, -0.3]])

# forward_position = np.array([[-0.15, 0.0, -0.30],
#                              [-0.15, 0.0, -0.20],
#                              [-0.0, 0.0, -0.20],
#                              [-0.00, 0.0, -0.30]])


backwards_position = np.array([[-0.02, 0.0, -0.30],
                               [-0.04, 0.0, -0.30], #
                               [-0.08, 0.0, -0.30],
                               [-0.10, 0.0, -0.30], #
                               [-0.12, 0.0, -0.30],
                               [-0.14, 0.0, -0.30], #
                               [-0.15, 0.0, -0.30]])

forward_position = np.array([[-0.15, 0.0, -0.30],
                             [-0.15, 0.0, -0.25], #
                             [-0.15, 0.0, -0.20],
                             [-0.07, 0.0, -0.20], #
                             [-0.00, 0.0, -0.20],
                             [-0.00, 0.0, -0.25], #
                             [-0.00, 0.0, -0.30]])

backward_motor_angles = calculate_ik(backwards_position)
forward_motor_angles = calculate_ik(forward_position)

backwards_interpolated, _ = linear_interpolate_path(backward_motor_angles, 2)
print("backwards motor angels", backward_motor_angles)
print("backwards interpolated", backwards_interpolated)

forward_interpolated, _ = linear_interpolate_path(forward_motor_angles, 2)
print("forward motor angels", forward_motor_angles)
print("forward interpolated", forward_interpolated)

complete_motor_angles = [
    [0, float(forward[0]), float(forward[1]), 0, float(ret[0]), float(ret[1])]
    for forward, ret in zip(backwards_interpolated, forward_interpolated)
]
complete_motor_angles_reverse = [
    [0, float(forward[0]), float(forward[1]), 0, float(ret[0]), float(ret[1])]
    for forward, ret in zip(forward_interpolated, backwards_interpolated)
]
print("complete motor angles + complete_motor_angles_reverse",
      complete_motor_angles + complete_motor_angles_reverse)
print("len of complete motor angles", len(complete_motor_angles + complete_motor_angles_reverse))

print("complete motor angles + complete_motor_angles_reverse",
      complete_motor_angles_reverse + complete_motor_angles)
print("len of complete motor angles", len(complete_motor_angles + complete_motor_angles_reverse))
# print("complete motor angles", complete_motor_angles)
# print("len of complete motor angles", len(complete_motor_angles))
