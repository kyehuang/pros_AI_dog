"""
This module is used to calculate the reward based on the action and observation
"""

def reward_cal(data: dict, prev_data: dict, step: int):
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

    # print("data: ", data)
    # print("prev_data: ", prev_data)

    # print("decay_distance_to_goal: ", decay_distance_to_goal)
    # distance_to_stright_line = data["distance_to_stright_line"]



    angle_z = data["spot_angle"][2] if data["spot_angle"][2] < 180 else 360 - data["spot_angle"][2]

    reward =  - 1 * pow(angle_z, 2) + 1 * step

    if angle_z < 3:
        reward += 10
    return reward
