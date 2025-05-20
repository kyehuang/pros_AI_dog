"""
This script calculates the motor angles for the Spot robot legs based on the base position,
base rotation, and base translation. It uses inverse kinematics to determine the angles needed
to position the legs at the desired end positions.
The script also includes a function to rotate a 3D point around the x, y, and z axes,
and a function to calculate the shoulder positions of the Spot robot legs based on the base
position, base rotation, and base translation.
"""
import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


from IK.spot_leg import SpotLeg

def rotate_point(point, angles_deg, order='xyz', rotate_axes=False):
    """
    Rotate a 3D point around x, y, z axes.

    Args:
        point: list or np.array of shape (3,), e.g. [x, y, z]
        angles_deg: list or tuple of angles in degrees, e.g. [45, 0, 0]
        order: rotation order, default 'xyz'
        rotate_axes: if True, rotate the coordinate system (use transpose of matrix)

    Returns:
        np.array: rotated point
    """
    point = np.array(point)
    angles_rad = np.radians(angles_deg)

    # Rotation matrices
    def r_x(theta):
        return np.array([
            [1, 0, 0],
            [0,  np.cos(theta), np.sin(theta)],
            [0, -np.sin(theta), np.cos(theta)]
        ])

    def r_y(theta):
        return np.array([
            [np.cos(theta), 0, -np.sin(theta)],
            [0, 1, 0],
            [np.sin(theta), 0, np.cos(theta)]
        ])

    def r_z(theta):
        return np.array([
            [ np.cos(theta), np.sin(theta), 0],
            [-np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])

    rot_map = {'x': r_x, 'y': r_y, 'z': r_z}
    rot = np.eye(3)

    for axis, theta in zip(order, angles_rad):
        r_axis = rot_map[axis](theta)
        rot = r_axis @ rot  # Apply in order

    if rotate_axes:
        rot = rot.T  # For rotating the coordinate system

    return rot @ point

def calculate_leg_end_positions(base_translation, joint_lengths, leg_end_offset=None):
    """
    Calculate the default or offset leg end positions.
    """
    # calculate the leg end positions
    lf = [ base_translation[0] / 2,
           base_translation[1] / 2 + joint_lengths[0],
           0]
    rf = [ base_translation[0] / 2,
          -base_translation[1] / 2 - joint_lengths[0],
           0]
    rb = [-base_translation[0] / 2,
          -base_translation[1] / 2 - joint_lengths[0],
           0]
    lb = [-base_translation[0] / 2,
           base_translation[1] / 2 + joint_lengths[0],
           0]

    # apply the leg end offset if provided
    if leg_end_offset is not None:
        lf = [lf[i] + leg_end_offset[0][i] for i in range(3)]
        rf = [rf[i] + leg_end_offset[1][i] for i in range(3)]
        rb = [rb[i] + leg_end_offset[2][i] for i in range(3)]
        lb = [lb[i] + leg_end_offset[3][i] for i in range(3)]

    return lf, rf, rb, lb

def spot_state_creater(spot_leg, base_position, base_rotation,
                       base_translation, base_tilt, leg_end_position=None):
    """
    Calculate the motor angles for the Spot robot legs based on the base position,
    base rotation, and base translation.
    """
    # Calculate the leg end positions
    lf_end_position, rf_end_position, rb_end_position, lb_end_position =calculate_leg_end_positions(
        base_translation, spot_leg.joint_lengths, leg_end_position)

    # Calculate the shoulder positions
    lf_shoulder, rf_shoulder, rb_shoulder, lb_shoulder = calculate_spot_shoulder_positon(
        base_position, base_rotation, base_translation, base_tilt)

    lf_target = delete_offset(lf_end_position, lf_shoulder)
    rf_target = delete_offset(rf_end_position, rf_shoulder)
    rb_target = delete_offset(rb_end_position, rb_shoulder)
    lb_target = delete_offset(lb_end_position, lb_shoulder)

    lf_target = rotate_point(lf_target, base_rotation, order='xyz', rotate_axes=False)
    rf_target = rotate_point(rf_target, base_rotation, order='xyz', rotate_axes=False)
    rb_target = rotate_point(rb_target, base_rotation, order='xyz', rotate_axes=False)
    lb_target = rotate_point(lb_target, base_rotation, order='xyz', rotate_axes=False)


    lf_angles = spot_leg.calculate_ik_left(lf_target, [0, 0, 0])
    rf_angles = spot_leg.calculate_ik_right(rf_target, [0, 0, 0])
    rb_angles = spot_leg.calculate_ik_right(rb_target, [0, 0, 0])
    lb_angles = spot_leg.calculate_ik_left(lb_target, [0, 0, 0])

    return lf_angles + rf_angles + rb_angles +  lb_angles

def delete_offset(target, base):
    """
    Delete the target position from the base position.
    """
    return [target[0] - base[0], target[1] - base[1], target[2] - base[2]]

def rotate_around_axis(v, axis, angle_degrees):
    """
    將向量 v 繞 axis 旋轉 angle_degrees
    """
    angle_rad = np.radians(angle_degrees)
    axis = axis / np.linalg.norm(axis)  # 單位化旋轉軸
    cos_theta = np.cos(angle_rad)
    sin_theta = np.sin(angle_rad)
    cross = np.cross(axis, v)
    dot = np.dot(axis, v)
    rotated_vector = (
        v * cos_theta +
        cross * sin_theta +
        axis * dot * (1 - cos_theta)
    )
    return rotated_vector

def calculate_spot_shoulder_positon(base_position, base_rotation, base_translation, base_tilt=None):
    """
    Calculate the shoulder positions of the Spot robot legs based on the base position,
    base rotation, and base translation.
    """
    # Rotate the shoulder offset
    lf_offset = [ base_translation[0] / 2,
                           base_translation[1] / 2,
                           0]
    rf_offset = [ base_translation[0] / 2,
                          -base_translation[1] / 2,
                           0]
    rb_offset = [ -base_translation[0] / 2,
                           -base_translation[1] / 2,
                           0]
    lb_offset = [ -base_translation[0] / 2,
                           base_translation[1] / 2,
                           0]

    if base_tilt is not None:
        lf_rotated_vector = rotate_around_axis(
                                np.array(lf_offset), np.array(rf_offset), base_tilt[0])
        rb_rotated_vector = rotate_around_axis(
                                np.array(rb_offset), np.array(rf_offset), base_tilt[0])

        rf_rotated_vector = rotate_around_axis(
                                np.array(rf_offset), np.array(lf_rotated_vector), base_tilt[1])
        lb_rotated_vector = rotate_around_axis(np.array(lb_offset),
                                np.array(lf_rotated_vector), base_tilt[1])

        lf_offset = lf_rotated_vector
        rb_offset = rb_rotated_vector

        rf_offset = rf_rotated_vector
        lb_offset = lb_rotated_vector


    lf_shoulder_offset = rotate_point(lf_offset, base_rotation, order='xyz', rotate_axes=True)
    rf_shoulder_offset = rotate_point(rf_offset, base_rotation, order='xyz', rotate_axes=True)
    rb_shoulder_offset = rotate_point(rb_offset, base_rotation, order='xyz', rotate_axes=True)
    lb_shoulder_offset = rotate_point(lb_offset, base_rotation, order='xyz', rotate_axes=True)

    # Calculate the leg shoulder positions
    lf_shoulder = [base_position[0] + lf_shoulder_offset[0],
                     base_position[1] + lf_shoulder_offset[1],
                     base_position[2] + lf_shoulder_offset[2]]
    rf_shoulder = [base_position[0] + rf_shoulder_offset[0],
                    base_position[1] + rf_shoulder_offset[1],
                     base_position[2] + rf_shoulder_offset[2]]
    rb_shoulder = [base_position[0] + rb_shoulder_offset[0],
                    base_position[1] + rb_shoulder_offset[1],
                     base_position[2] + rb_shoulder_offset[2]]
    lb_shoulder = [base_position[0] + lb_shoulder_offset[0],
                    base_position[1] + lb_shoulder_offset[1],
                    base_position[2] + lb_shoulder_offset[2]]

    return lf_shoulder, rf_shoulder, rb_shoulder, lb_shoulder

if __name__ == "__main__":
    SpotPosition = [0, 0, 2]
    JointLengths = [1.0, 2.0, 2.0]
    BaseTranslation = [6, 4, 0]
    p = [0, 2, -2]

    # 同樣的旋轉，但這次是旋轉點（物體）
    rotated = rotate_point(p, [0, 0, 90], order='xyz', rotate_axes=True)
    print("旋轉座標軸後的座標:", rotated)

    # 只旋轉 x 軸 45 度，座標系旋轉
    rotated_obj = rotate_point(p, [0, 0, 90], order='xyz', rotate_axes=False)
    print("旋轉點本身後的座標:", rotated_obj)

    # Create a SpotLeg instance
    SpotLeg = SpotLeg(JointLengths, [0.0, 0.0, 0.0])
    BasePosition = [0, 0, 2]
    BaseRotation = [1, 0, 0]
    BaseTilt = [0, 0]
    # Calculate the end position of the leg
    motor_angles = spot_state_creater(SpotLeg, BasePosition, BaseRotation, BaseTranslation, BaseTilt)
    print("Motor angles for each leg:", motor_angles)
