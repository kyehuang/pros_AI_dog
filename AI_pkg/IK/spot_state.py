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

def spot_state_creater(spot_leg, base_position, base_rotation, base_translation, base_tilt):
    """
    Calculate the motor angles for the Spot robot legs based on the base position,
    base rotation, and base translation.
    """
    # Define the leg end position of the Spot robot
    lf_end_position = [ base_translation[0] / 2,
                        base_translation[1] / 2 + spot_leg.joint_lengths[0],
                        0]
    rf_end_position = [ base_translation[0] / 2,
                       -base_translation[1] / 2 - spot_leg.joint_lengths[0],
                        0]
    rb_end_position = [-base_translation[0] / 2,
                       -base_translation[1] / 2 - spot_leg.joint_lengths[0],
                        0]
    lb_end_position = [-base_translation[0] / 2,
                        base_translation[1] / 2 + spot_leg.joint_lengths[0],
                        0]

    # Calculate the shoulder positions
    lf_shoulder, rf_shoulder, rb_shoulder, lb_shoulder = calculate_spot_shoulder_positon(
        base_position, base_rotation, base_translation)

    lf_shoulder[2] += base_tilt[0]
    rf_shoulder[2] += base_tilt[1]
    rb_shoulder[2] -= base_tilt[0]
    lb_shoulder[2] -= base_tilt[1]

    lf_angles = spot_leg.calculate_ik_left(lf_end_position, lf_shoulder)
    rf_angles = spot_leg.calculate_ik_right(rf_end_position, rf_shoulder)
    rb_angles = spot_leg.calculate_ik_right(rb_end_position, rb_shoulder)
    lb_angles = spot_leg.calculate_ik_left(lb_end_position, lb_shoulder)

    return lf_angles + rf_angles + rb_angles +  lb_angles

def calculate_spot_shoulder_positon(base_position, base_rotation, base_translation):
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
