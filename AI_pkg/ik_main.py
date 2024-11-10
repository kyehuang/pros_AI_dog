"""
This is the main file for testing the Inverse Kinematics moudule.
"""
from IK.fabrik import fabrik
from IK import config
from IK.utils import calculate_individual_pitch_yaw_roll
import numpy as np

target = np.array([0.0, 0.0, -0.3])
joints = np.array([[0.0, 0.0, 0.0], [-0.2, 0.0, 0.00001], [-0.2, 0.0, -0.20001]])
joint_length = config.JOINT_LENGTH

def angle_to_motor_angle(angles):
    """
    this function is used to convert the angle to motor angle
    """
    offset = [-20, 20]

    motor_angles = [0.0, 0,0]
    motor_angles[0] = -angles[1][1]  - 90 + offset[0]
    motor_angles[1] = -angles[1][1] + angles[2][1] - 180 - offset[1]
    return motor_angles


targets = np.array([[0.0, 0.0, -0.3], [0.025, 0.0, -0.275], [0.05, 0.0, -0.25], [0.05, 0.0, -0.3]])
joints = np.array([[0.0, 0.0, 0.0], [-0.2, 0.0, 0.00001], [-0.2, 0.0, -0.20001]])

motor_angles = []
for target in targets:
    joints = fabrik(target, joints, joint_length)
    # print("angle" , calculate_individual_pitch_yaw_roll(joints))
    motor_angles.append(angle_to_motor_angle(calculate_individual_pitch_yaw_roll(joints)))
    # print("angle to motor angle", angle_to_motor_angle(calculate_individual_pitch_yaw_roll(joints)))
    # print("joint" , joints)
    # print("target" , target)
    # print("\n")

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
        interpolated_section = [start + (end - start) * t / (num_points - 1) for t in range(num_points)]
        interpolated_angles.extend(interpolated_section[:-1])  # 去掉重複的點        

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


print("motor angles", motor_angles)
interpolated_forward = interpolate_forward(motor_angles, 2)
print("interpolated forward", interpolated_forward)
interpolated_return = interpolate_return(motor_angles, len(interpolated_forward))
print("interpolated return", interpolated_return)
print("len of two interpolated", len(interpolated_forward), len(interpolated_return))

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

interpolated_angles = interpolate(motor_angles, 7)
print("interpolated angles", interpolated_angles)
print("len of interpolated angles", len(interpolated_angles))
