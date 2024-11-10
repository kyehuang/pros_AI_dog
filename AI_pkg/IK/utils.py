"""
This file contains the utility functions for the Inverse Kinematics moudule.
"""
import numpy as np

def calculate_individual_pitch_yaw_roll(joints):
    """
    Calculate pitch, yaw, and roll angles for each segment (A->B, B->C) in 3D space.
    :param joints: List of joint positions in 3D space (each joint is a numpy array)
    :return: Dictionary with pitch, yaw, and roll for each segment
    """
    segment_angles = {}

    for i in range(1, len(joints)):
        # Calculate vector for each segment
        vec = joints[i] - joints[i - 1]

        # Normalize the vector
        norm_vec = vec / np.linalg.norm(vec) if np.linalg.norm(vec) != 0 else vec

        # Calculate Pitch (angle in Y-Z plane)
        pitch = np.arctan2(norm_vec[2], norm_vec[1])

        # Calculate Yaw (angle in X-Z plane)
        yaw = np.arctan2(norm_vec[0], norm_vec[2])

        # Calculate Roll (angle in X-Y plane)
        roll = np.arctan2(norm_vec[1], norm_vec[0])

        # Convert angles to degrees
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)
        roll_deg = np.degrees(roll)

        segment_angles[i] = (pitch_deg, yaw_deg, roll_deg)

    return segment_angles


def calculate_position(orgin_position:np, angle:float, length:float):
    """
    Calculate the position of the next joint given the angle and length.
    :param orgin_position: orgin position
    :param angle: angle
    :param length: length
    :return: position
    """
    x = orgin_position[0] + length * np.cos(angle)
    y = orgin_position[1] + length * np.sin(angle)
    return np.array([x, y])

def calculate_feet_positions(height, setp_length):
    """
    Calculate the position of the feet given the height and step length.
    :param height: height
    :param setp_length: step length
    :return: position
    """
    feet_positions = [np.array([ 1.5 * setp_length, -height]),
                      np.array([ 0.5 * setp_length, -height]), 
                      np.array([-0.5 * setp_length, -height]),
                      np.array([-1.5 * setp_length, -height])]
    return feet_positions

def calculate_motor_positions(first_joint_position, angles, lengths):
    """
    Calculate the motor positions given the angles and lengths.
    :param first_joint_position: first joint position
    :param angles: angles
    :param lengths: lengths
    :return: motor positions
    """
    position = first_joint_position
    motor_positions = [position]
    for i in range(len(angles)):
        next_position = calculate_position(position, np.radians(angles[i]), lengths[i])
        position = next_position
        motor_positions.append(next_position)
    return motor_positions

def angle_normalize(angle):
    """
    Normalize the angle to the range of [-180, 180].
    :param angle: angle
    :return: normalized angle
    """
    return (angle + 180) % 360 - 180

def angle_transform_to_unity(angles, motor_offsets):
    """
    Transform the angles to the Unity coordinate system.
    :param angles: angles
    :return: transformed angles
    """
    # Unity Model motor have different direction and angle range
    # -------------------------------------      -----------
    # | Motor | Direction | Range    |                |
    # --------------------------------           +    |   -
    # | 1   | CW        |  -90 ~ 90  |                |
    # | 2   | CCW       |  -90 ~ 90  |             0 degree

    ## 1. The first motor is the thigh motor
    angle_1 = - (angles[0] - 270)

    ## 2. The second motor is the calf motor
    angle_2 = - ( angle_1 + angles[1] - 270)

    angles = [angle_1 + motor_offsets[0], angle_2 + motor_offsets[1]]
    for i in range(len(angles)):
        angles[i] = angle_normalize(angles[i])
    return angles
