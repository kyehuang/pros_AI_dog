"""
This module is used to calculate the reward based on the action and observation
"""
import copy
def reward_cal(data_list: dict, prev_data_list: dict, step: int):
    """
    Calculate the reward based on the action and observation

    Args:
        data (dict): The current observation
        prev_data (dict): The previous observation
        step (int): The current step

    Returns:
        float: The reward
    """
    # Calculate the reward
    reward = 0

    data = copy.deepcopy(data_list)
    prev_data = copy.deepcopy(prev_data_list)

    # Calculate the reward based on the angle_x and angle_z
    angle_x = data["spot_angle"][0] if data["spot_angle"][0] < 180 else 360 - data["spot_angle"][0]
    angle_z = data["spot_angle"][2] if data["spot_angle"][2] < 180 else 360 - data["spot_angle"][2]

    if prev_data["spot_angle"][0] < 180:
        angle_x_prev = prev_data["spot_angle"][0]
    else:
        angle_x_prev = 360 - prev_data["spot_angle"][0]

    if prev_data["spot_angle"][2] < 180:
        angle_z_prev = prev_data["spot_angle"][2]
    else:
        angle_z_prev = 360 - prev_data["spot_angle"][2]

    angle_x_diff = abs(angle_x) - abs(angle_x_prev)
    angle_z_diff = abs(angle_z) - abs(angle_z_prev)

    cost = angle_x_diff + angle_z_diff
    if cost > 50:
        reward -= 50
    else:
        reward -= cost

    reward += 2 * step

    if angle_z < 3:
        reward += 5
    if angle_x < 3:
        reward += 5

    return reward
