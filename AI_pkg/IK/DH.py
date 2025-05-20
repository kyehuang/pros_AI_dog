import numpy as np
from math import cos, sin, radians, degrees
import sys
import os
from scipy.spatial.transform import Rotation as R

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from IK.spot_leg import SpotLeg
from IK.spot_state import calculate_spot_shoulder_positon, spot_state_creater
from IK.plane_to_euler import extract_euler_angles_from_plane

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

def compute_pose_from_rotation(feet_pts, shoulder_pts):
    feet = np.array(feet_pts)
    shoulders = np.array(shoulder_pts)

    # Step 1: 法向量
    normal_feet = compute_normal_vector(feet[0], feet[1], feet[3])

    # normal_feet = compute_normal_vector(feet[0], feet[3], feet[2])

    # Step 2: 旋轉矩陣
    target_normal = np.array([0, 0, 1])
    rotation, _ = R.align_vectors([target_normal], [normal_feet])

    # Step 3: 套用旋轉
    shoulders_rot = rotation.apply(shoulders)

    # Step 4: 計算 Yaw（XY 向量方向）
    vec = -shoulders_rot[0][:2] + shoulders_rot[3][:2]  # LF - LB
    yaw_rad = np.arctan2(vec[1], vec[0])
    yaw_deg = np.degrees(yaw_rad)

    # Step 5: 取 Roll、Pitch
    roll, pitch, _ = rotation.as_euler('xyz', degrees=True)

    return [round(roll, 3), round(pitch, 3), round(yaw_deg, 3)]

def rotate_to_ground_and_align_x(four_points, base_points=None):
    """
    將四個三維點構成的四邊形：
      1. 平面旋轉到 z=0。
      2. 第一條邊 (P1→P2) 對齊 x 軸。
    同時旋轉 base_points (如有提供)。

    Returns:
        tuple: (旋轉後的四邊形點, 旋轉後的 base_points 或 None)
    """
    def center_z(points, offset):
        points[:, 2] -= offset
        return points

    four_points = np.asarray(four_points)
    P1, P2, P3 = four_points[0], four_points[1], four_points[2]

    # === 第一階段：對齊平面到 z=0 ===
    normal = np.cross(P3 - P1, P2 - P1)
    normal /= np.linalg.norm(normal)
    target_normal = np.array([0, 0, 1])

    cross1 = np.cross(normal, target_normal)
    if np.linalg.norm(cross1) < 1e-6:
        rotation1 = R.identity()
    else:
        axis1 = cross1 / np.linalg.norm(cross1)
        angle1 = np.arccos(np.clip(np.dot(normal, target_normal), -1.0, 1.0))
        rotation1 = R.from_rotvec(axis1 * angle1)

    rotated = rotation1.apply(four_points)
    z_offset = rotated[0, 2] - 0
    print("z_offset:", z_offset)
    rotated = center_z(rotated, z_offset)

    # === 第二階段：讓邊對齊 x 軸 ===
    edge = rotated[0] - rotated[3]

    edge[2] = 0  # 只看 xy 平面方向
    edge /= np.linalg.norm(edge)

    target_dir = np.array([1, 0, 0])
    cross2 = np.cross(edge, target_dir)

    if np.linalg.norm(cross2) < 1e-6:
        rotation2 = R.identity()
    else:
        angle2 = np.arccos(np.clip(np.dot(edge, target_dir), -1.0, 1.0))
        axis2 = np.array([0, 0, 1])  # 第二階段僅繞 z 軸旋轉
        sign = np.sign(np.cross(edge, target_dir)[2])
        rotation2 = R.from_rotvec(axis2 * angle2 * sign)

    rotated = rotation2.apply(rotated)

    # 處理 base_points（如有）
    if base_points is not None:
        base_points = np.asarray(base_points)
        rotated_base = rotation1.apply(base_points)
        rotated_base = center_z(rotated_base, z_offset)
        rotated_base = rotation2.apply(rotated_base)

        return rotated, rotated_base

    return rotated, None

def is_on_same_plane(points):
    """
    Check if four points are on the same plane.
    :param points: List of four 3D points
    :return: True if points are coplanar, False otherwise
    """
    v1 = points[1] - points[0]
    v2 = points[2] - points[0]
    v3 = points[3] - points[0]

    # calculate the normal vector of the plane formed by the first three points
    normal_vector = np.cross(v1, v2)

    # check if the fourth point is coplanar with the first three points
    is_coplanar = np.isclose(np.dot(normal_vector, v3), 0)

    return is_coplanar

def compute_normal_vector(p1, p2, p3):
    """
    計算三個點構成平面的法向量並正規化
    """
    v1 = np.array(p2) - np.array(p1)
    v2 = np.array(p3) - np.array(p1)
    normal = np.cross(v1, v2)
    normal = normal / np.linalg.norm(normal)
    return normal

def vector_to_euler(normal_vector, base_vector=[0, 0, 1]):
    """
    計算將法向量轉到 [0, 0, 1] 的旋轉角度 (ROS: XYZ)
    """
    # 計算旋轉矩陣
    rotation = R.align_vectors([base_vector], [normal_vector])[0]
    euler_angles = rotation.as_euler('xyz', degrees=True)
    print("b",base_vector)
    return euler_angles

def get_aligned_feet_and_shoulders(joint_angles, joint_lengths, base_translations):
    """
    calculate the aligned feet and shoulders of a quadruped robot.
    :param joint_angles: List of joint angles in degrees
    :param joint_lengths: List of joint lengths
    :param base_translations: Base translation of the robot
    :return: Tuple of aligned feet and shoulders
    """
    # get the foot positions
    foot_pos = get_foot_position(joint_angles, joint_lengths, base_translations)
    feet = np.array([foot_pos["LF"], foot_pos["RF"], foot_pos["RB"], foot_pos["LB"]])

    # get the shoulder positions
    shoulders = np.array(calculate_spot_shoulder_positon(
        [0, 0, 0], [0, 0, 0], base_translations
    ))

    # rotate the feet and shoulders to the ground plane
    rotate_feet, rotate_shoulders = rotate_to_ground_and_align_x(feet, shoulders)

    return rotate_feet, rotate_shoulders

def get_base_pose(joint_angles, joint_lengths, base_translations, type=None):
    """
    Calculate the base pose of a quadruped robot given joint angles and lengths.
    """
    # get the aligned feet and shoulders
    rotate_feet, rotate_shoulders = get_aligned_feet_and_shoulders(
        joint_angles, joint_lengths, base_translations
    )

    # get the position offset
    center_feet = np.mean(rotate_feet, axis=0)
    center_shoulders = np.mean(rotate_shoulders, axis=0)
    position_offset = center_shoulders - center_feet
    position = [round(x, 3) for x in position_offset.tolist()]

    # get the rotation angles
    rotation = extract_euler_angles_from_plane(rotate_shoulders)

    # === Debug 輸出 ===
    # print("Shoulder Positions:", shoulders)
    # print("New Feet Positions:", rotated_feet)
    # print("New Shoulder Positions:", rotated_shoulders)
    # print("Center Feet:", center_feet)
    # print("Center Shoulders:", center_shoulders)
    # print("Position Offset:", position)
    # print("Euler Angles (deg):", rotation)

    return position, rotation

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
    test_angle = spotLeg.calculate_ik_left(end_position[:3, 3], BaseTranslation)
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

    baseRot = [8.88, 10, -4]
    JointAngles = spot_state_creater(
        spotLeg, [0, 0, 2], baseRot, BaseTranslation, [0, 0]
    )

    print(get_foot_position(JointAngles, JointLengths, BaseTranslation))
    JointAngles = spot_state_creater(
        spotLeg, [0, 0, 2], baseRot, BaseTranslation, [0, 0]
    )
    print(get_base_pose(JointAngles, JointLengths, BaseTranslation))