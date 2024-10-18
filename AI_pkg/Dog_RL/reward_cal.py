from ros_receive_and_processing.AI_dog_node import AI_dog_node

def reward_cal(data: dict, prev_data: dict):
    """
    Calculate the reward based on the action and observation

    Args:
        node (AI_dog_node): AI_dog_node instance
        action (list): The action to be executed
        observation (dict): The observation from the environment

    Returns:
        float: The reward
    """
    # Calculate the reward
    reward = 0

    # print("data: ", data)
    # print("prev_data: ", prev_data)
    
    decay_distance_to_goal = data["distance_to_goal"] - prev_data["distance_to_goal"]
    # print("decay_distance_to_goal: ", decay_distance_to_goal)
    distance_to_stright_line = data["distance_to_stright_line"]


    angle_x =  data["spot_angle"][0] if data["spot_angle"][0] < 180 else 360 - data["spot_angle"][0]
    # print("angle_x: ", angle_x)
    angle_z = data["spot_angle"][2] if data["spot_angle"][2] < 180 else 360 - data["spot_angle"][2]
    reward = - decay_distance_to_goal - 0.01 * abs(distance_to_stright_line) - 0.05 * angle_x - 0.05 * angle_z

    return reward