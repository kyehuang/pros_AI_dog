"""
This is the main file for testing the Inverse Kinematics moudule.
"""
from IK.fabrik import fabrik
from IK import config
from IK.utils import calculate_individual_pitch_yaw_roll
import numpy as np


# joints  = np.array([[ 0, 0, 0],
#             [0, 0, 1],
#             [0, 0, 2]])
# print(calculate_individual_pitch_yaw_roll(joints))
# exit()
def angle_to_motor_angle(angles):
    """
    this function is used to convert the angle to motor angle
    """
    offset = [-20, 20]

    motor_angles = [0.0, 0,0]
    motor_angles[0] = angles[1][1]  + 180 + offset[0]
    motor_angles[1] = (-(-angles[1][1] + angles[2][1]) + 360 - offset[1]) % 360
    return motor_angles

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

def interpolate_forward(motor_angles, num_points=10):
    """
    對馬達角度數據進行正向的線性插值，不包括最後一組到第一組的回程。
    
    參數：
        motor_angles (list): 初始馬達角度數據的二維列表。
        num_points (int): 每個區間要插值的點數。
        
    返回：
        np.ndarray: 插值後的馬達角度數據（不包含回程）。
        int: 總插值點數，用於後續回程插值。
    """
    interpolated_angles = []

    for i in range(len(motor_angles) - 1):
        start = np.array(motor_angles[i])
        end = np.array(motor_angles[i + 1])
        interpolated_section = [start + (end - start) * t / (num_points + 1) for t in range(num_points + 1)]        
        interpolated_angles.extend(interpolated_section)  # 去掉重複的點
        print("interpolated section", interpolated_angles)

    interpolated_angles.append(motor_angles[-1])  # 添加最後一點
    return np.array(interpolated_angles)[:, :-1]

def interpolate_return(motor_angles, total_points):
    """
    計算從最後一組回到第一組的線性插值。
    
    參數：
        motor_angles (list): 初始馬達角度數據的二維列表。
        total_points (int): 插值的總點數（來自正向插值）。
        
    返回：
        np.ndarray: 回程插值的馬達角度數據。
    """
    start = np.array(motor_angles[-1])
    end = np.array(motor_angles[0])
    return_interpolated = [start + (end - start) * t / (total_points - 1) for t in range(total_points)]

    return np.array(return_interpolated)[:, :-1]

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

# print("motor angles", motor_angles)
# interpolated_forward = interpolate_forward(motor_angles, 1)
# print("interpolated forward", interpolated_forward)

# interpolated_forward = linear_interpolate_path(motor_angles, 1)
# print("interpolated forward", interpolated_forward)

# interpolated_return = interpolate_return(motor_angles, len(interpolated_forward))
# print("interpolated return", interpolated_return)
# print("len of two interpolated", len(interpolated_forward), len(interpolated_return))

def interpolate(motor_angles, num_points=10):
    """
    對馬達角度數據進行插值，包括正向和回程。
    
    參數：
        motor_angles (list): 初始馬達角度數據的二維列表。
        num_points (int): 每個區間要插值的點數。
        
    返回：
        np.ndarray: 插值後的馬達角度數據，包含正向和回程。
    """
    # 計算正向插值
    interpolated_forward = interpolate_forward(motor_angles, num_points)

    # 計算回程插值
    interpolated_return = interpolate_return(motor_angles, len(interpolated_forward))

    # 合併正向和回程插值並轉換為float
    complete_motor_angles = [
        [0] + [float(angle) for angle in forward] + [0] + [float(angle) for angle in ret]
        for forward, ret in zip(interpolated_forward, interpolated_return)
    ]
    complete_motor_angles_other = [
        [0] + [float(angle) for angle in forward] + [0] + [float(angle) for angle in ret]
        for forward, ret in zip(interpolated_return, interpolated_forward)
    ]
    
    return complete_motor_angles + complete_motor_angles_other

# interpolated_angles = interpolate(motor_angles, 2)
# print("interpolated angles", interpolated_angles)
# print("len of interpolated angles", len(interpolated_angles))

backwards_position = np.array([[ -0.0, 0.0, -0.3],
                               [-0.07, 0.0, -0.3],
                               [-0.10, 0.0, -0.3],
                               [-0.15, 0.0, -0.3]])

forward_position = np.array([[-0.15, 0.0, -0.30],
                             [-0.15, 0.0, -0.20],
                             [-0.0, 0.0, -0.20],
                             [-0.00, 0.0, -0.30]])


# backwards_position = np.array([[-0.02, 0.0, -0.3],
#                                [-0.04, 0.0, -0.3], #
#                                [-0.08, 0.0, -0.3],
#                                [-0.10, 0.0, -0.3], #
#                                [-0.12, 0.0, -0.3],
#                                [-0.14, 0.0, -0.3], #
#                                [-0.15, 0.0, -0.3]])

# forward_position = np.array([[-0.15, 0.0, -0.30],
#                              [-0.15, 0.0, -0.25], #
#                              [-0.15, 0.0, -0.20],
#                              [-0.07, 0.0, -0.20], #
#                              [-0.00, 0.0, -0.20],
#                              [-0.00, 0.0, -0.25], #
#                              [-0.00, 0.0, -0.30]])

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
