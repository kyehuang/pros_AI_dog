import numpy as np
from math import cos, sin, radians, degrees
import sys
import os
from scipy.spatial.transform import Rotation as R

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from IK.spot_leg import SpotLeg
from IK.spot_state import calculate_spot_shoulder_positon

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

def rotate_to_ground(four_points, base_points=None):
    """
    將四個三維點構成的四邊形旋轉到 z = 0 的平面上。
    如果提供 base_points，也會一起旋轉。
    
    Parameters:
        four_points (array-like): 4 個 3D 點 (Numpy array)，構成一個四邊形。
        base_points (array-like, optional): 可選的另一組 4 個 3D 點，也會被一起旋轉。
    
    Returns:
        tuple: (Rotated 四邊形點, Rotated base_points 或 None)
    """
    four_points = np.asarray(four_points)
    P1, P2, P3 = four_points[0], four_points[1], four_points[2]

    
    # 計算法向量並正規化
    normal_vector = np.cross(P3 - P1, P2 - P1)
    normal_vector /= np.linalg.norm(normal_vector)
    print("normal_vector:", normal_vector)

    # 目標法向量 [0, 0, 1]
    target_normal = np.array([0, 0, 1])

    # 若已與 z 平面平行，則不需旋轉
    if np.linalg.norm(np.cross(normal_vector, target_normal)) < 1e-6:
        print("四邊形已經貼合 z = 0 平面")
        return four_points, base_points

    # 計算旋轉軸與角度
    rotation_axis = np.cross(normal_vector, target_normal)
    rotation_axis /= np.linalg.norm(rotation_axis)
    angle = np.arccos(np.clip(np.dot(normal_vector, target_normal), -1.0, 1.0))

    # 建立旋轉矩陣
    rotation = R.from_rotvec(rotation_axis * angle)

    # 應用旋轉到四邊形
    rotated_points = rotation.apply(four_points)

    # 將 z 軸平移至 0（根據旋轉後第一個點）
    z_offset = rotated_points[0, 2]
    rotated_points[:, 2] -= z_offset

    # 如果有 base_points 也做一樣處理
    if base_points is not None:
        base_points = np.asarray(base_points)
        rotated_base = rotation.apply(base_points)
        rotated_base[:, 2] -= z_offset
        return rotated_points, rotated_base

    return rotated_points, None

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


def get_base_pose(joint_angles, joint_lengths, base_translations, type=None):
    """
    Calculate the base pose of a quadruped robot given joint angles and lengths.
    """
    # === Step 1: 計算四腳位置與肩膀位置 ===
    foot_pos = get_foot_position(joint_angles, joint_lengths, base_translations)
    feet = np.array([foot_pos["LF"], foot_pos["RF"], foot_pos["RB"], foot_pos["LB"]])

    shoulders = np.array(calculate_spot_shoulder_positon(
        [0, 0, 0], [0, 0, 0], base_translations
    ))

    # === Step 2: 將剛體旋轉到地面平面（z=0） ===
    rotated_feet, rotated_shoulders = rotate_to_ground(feet, shoulders)

    center_feet = np.mean(rotated_feet, axis=0)
    center_shoulders = np.mean(rotated_shoulders, axis=0)
    position_offset = center_shoulders - center_feet
    position = [round(x, 3) for x in position_offset.tolist()]

    # === Step 3: 檢查肩膀是否共平面（資料異常檢查） ===
    if not is_on_same_plane(shoulders):
        print("Shoulders are not on the same plane")
        return [0, 0, 0]

    # === Step 4: 計算 Pitch / Roll 角度（法向量差） ===
    normal_feet = compute_normal_vector(feet[0], feet[3], feet[2])
    normal_shoulders = compute_normal_vector(shoulders[0], shoulders[3], shoulders[2])

    euler_angles = vector_to_euler(normal_feet, normal_shoulders)
    rotation = [round(x, 3) for x in euler_angles.tolist()]

    # === Step 5: 計算 Yaw ===
    # 從左後到左前的方向向量（XY 平面）
    feet_vec = rotated_feet[3][:2] - rotated_feet[0][:2]  # LB - LF
    shoulder_vec = rotated_shoulders[3][:2] -  rotated_shoulders[0][:2]  # LBS - LFB 

    feet_vec /= np.linalg.norm(feet_vec)
    shoulder_vec /= np.linalg.norm(shoulder_vec)

    cos_theta = np.dot(feet_vec, shoulder_vec)
    sin_theta = feet_vec[1]  # 即使方向只取 y 分量也可搭配 atan2
    yaw_rad = np.arctan2(sin_theta, cos_theta)
    yaw_deg = np.degrees(yaw_rad)

    rotation[2] = round(yaw_deg, 3).tolist()

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
