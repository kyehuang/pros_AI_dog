import numpy as np
from scipy.optimize import minimize

from IK.spot_state import rotate_point

def rotate_x(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s,  c]
    ])

def rotate_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c]
    ])

def apply_rotation(angles):
    theta_x, theta_y = angles
    v = np.array([0, 0, 1])
    rotated = rotate_y(theta_y) @ rotate_x(theta_x) @ v
    return rotated

def loss(angles, target):
    rotated = apply_rotation(angles)
    return np.linalg.norm(rotated - target)

def solve_angles(n_target):
    n_target = np.array(n_target) / np.linalg.norm(n_target)  # normalize
    result = minimize(loss, x0=[0, 0], args=(n_target,), bounds=[(-np.pi, np.pi)]*2)
    theta_x, theta_y = np.degrees(result.x)
    return theta_x, theta_y

def compute_normal_vector(p1, p2, p3):
    """
    計算三個點構成平面的法向量並正規化
    """
    v1 = np.array(p2) - np.array(p1)
    v2 = np.array(p3) - np.array(p1)
    normal = np.cross(v1, v2)
    normal = normal / np.linalg.norm(normal)
    return normal

def angle_between_vectors(v1, v2):
    v1 = np.array(v1)
    v2 = np.array(v2)
    
    # 單位化向量
    v1_u = v1 / np.linalg.norm(v1)
    v2_u = v2 / np.linalg.norm(v2)
    
    # 計算夾角（弧度）
    dot_product = np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)
    angle_rad = np.arccos(dot_product)
    
    # 轉成角度
    angle_deg = np.degrees(angle_rad)
    return angle_deg

def signed_angle_3d(v1, v2, normal):
    v1_u = v1 / np.linalg.norm(v1)
    v2_u = v2 / np.linalg.norm(v2)
    normal_u = normal / np.linalg.norm(normal)

    angle_rad = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    cross = np.cross(v1_u, v2_u)
    sign = np.sign(np.dot(cross, normal_u))
    signed_angle = angle_rad * sign
    return np.degrees(signed_angle)

def get_euler_angles(rotated_base_plane):
    """
    計算旋轉後的基準平面與原始平面之間的歐拉角
    """
    # 這裡假設 rotated_base_plane 是一個 3x3 的旋轉矩陣
    # 你可以根據需要修改這個函數
    normal_vector_plane = compute_normal_vector(
        rotated_base_plane[0], rotated_base_plane[2], rotated_base_plane[1]
    )

    theta_x, theta_y = solve_angles(normal_vector_plane)

    x_axis_plane = rotated_base_plane[1] - rotated_base_plane[2]
    x_axis = np.array([1, 0, 0])
    rotate_x_axis = rotate_point(x_axis, [theta_x, theta_y], order='xy', rotate_axes=True)
    theta_z = signed_angle_3d(rotate_x_axis, x_axis_plane, normal_vector_plane)
    
    return theta_x, -theta_y, theta_z
# Example usage

def extract_euler_angles_from_plane(plane_points):
    """
    根據一個由三個或四個點構成的平面，推導其與世界座標對齊所需的 XYZ（Roll, Pitch, Yaw）角度。
    假設 plane_points 是形如 [[p1], [p2], [p3], [p4]] 的列表（四個 3D 點）。

    回傳：
        tuple: (roll, pitch, yaw) in degrees
    """
    # 計算法向量
    normal_vector = compute_normal_vector(plane_points[0], plane_points[2], plane_points[1])

    # 解 roll/pitch
    roll, pitch = solve_angles(normal_vector)

    # 解 yaw（使 x 軸對齊）
    local_x_axis = np.array(plane_points[1]) - np.array(plane_points[2])
    world_x_axis = np.array([1, 0, 0])

    rotated_world_x = rotate_point(world_x_axis, [roll, pitch], order='xy', rotate_axes=True)
    yaw = signed_angle_3d(rotated_world_x, local_x_axis, normal_vector)

    return roll, pitch, yaw