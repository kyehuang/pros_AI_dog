import numpy as np
from math import cos, sin, radians, degrees

from IK.SpotLeg import SpotLeg

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
    def rx(theta):
        return np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta),  np.cos(theta)]
        ])

    def ry(theta):
        return np.array([
            [ np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])

    def rz(theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0, 0, 1]
        ])

    rot_map = {'x': rx, 'y': ry, 'z': rz}
    r = np.eye(3)

    for axis, theta in zip(order, angles_rad):
        r_axis = rot_map[axis](theta)
        r = r_axis @ r  # Apply in order

    if rotate_axes:
        r = r.T  # For rotating the coordinate system

    return r @ point

def spot_state_creater(spot_leg:SpotLeg, base_position, base_rotation, base_translation):
    # Define the leg end position of the Spot robot
    lf_end_position = [ base_translation[0] / 2,
                        base_translation[1] / 2 + spot_leg.joint_lengths[0],
                        0]
    rf_end_position = [ base_translation[0] / 2,
                       -base_translation[1] / 2 + spot_leg.joint_lengths[0], # because the y axis is inverted
                        0]
    rb_end_position = [-base_translation[0] / 2,
                       -base_translation[1] / 2 + spot_leg.joint_lengths[0], # because the y axis is inverted
                        0]
    lb_end_position = [-base_translation[0] / 2,
                        base_translation[1] / 2 + spot_leg.joint_lengths[0],
                        0]
    print("lf_end_position:", lf_end_position)
    print("rf_end_position:", rf_end_position)
    print("rb_end_position:", rb_end_position)
    print("lb_end_position:", lb_end_position)

    # Calculate the leg shoulder positions
    lf_shoulder = [base_position[0] + base_translation[0] / 2,
                   base_position[1] + base_translation[1] / 2,
                   base_position[2]]
    rf_shoulder = [base_position[0] + base_translation[0] / 2,
                     base_position[1] - base_translation[1] / 2,
                     base_position[2]]
    rb_shoulder = [base_position[0] - base_translation[0] / 2,
                     base_position[1] - base_translation[1] / 2,
                     base_position[2]]
    lb_shoulder = [base_position[0] - base_translation[0] / 2,
                        base_position[1] + base_translation[1] / 2,
                        base_position[2]]
    print("lf_shoulder:", lf_shoulder)
    print("rf_shoulder:", rf_shoulder)
    print("rb_shoulder:", rb_shoulder)
    print("lb_shoulder:", lb_shoulder)

    # Calculate the inverse kinematics for each leg
    lf_angles = spot_leg.calculate_ik(lf_end_position, lf_shoulder)
    rf_angles = spot_leg.calculate_ik(rf_end_position, rf_shoulder)
    rb_angles = spot_leg.calculate_ik(rb_end_position, rb_shoulder)
    lb_angles = spot_leg.calculate_ik(lb_end_position, lb_shoulder)

    return lf_angles + rf_angles + rb_angles +  lb_angles

if __name__ == "__main__":
    SpotPosition = [0, 0, 2]
    JointLengths = [1.0, 2.0, 2.0]
    BaseTranslation = [6, 4, 0]
    p = [0, 1, -2]

    # 只旋轉 x 軸 45 度，座標系旋轉
    rotated = rotate_point(p, [45, 0, 0], order='xyz', rotate_axes=True)
    print("旋轉座標軸後的座標:", rotated)

    # 同樣的旋轉，但這次是旋轉點（物體）
    rotated_obj = rotate_point(p, [45, 0, 0], order='xyz', rotate_axes=False)
    print("旋轉點本身後的座標:", rotated_obj)

    # Create a SpotLeg instance
    SpotLeg = SpotLeg(JointLengths, [0.0, 0.0, 0.0])
    BasePosition = [0, 0, 2]
    BaseRotation = [0, 0, 0]
    # Calculate the end position of the leg
    motor_angles = spot_state_creater(SpotLeg, BasePosition, BaseRotation, BaseTranslation)
    print("Motor angles for each leg:", motor_angles)
