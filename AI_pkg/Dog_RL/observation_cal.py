import numpy as np

from ros_receive_and_processing.AI_dog_node import AI_dog_node

def get_observation(node : AI_dog_node):
    """
    Get observation from AI_dog_node
    """
    # reset observation
    
    node.reset_latest_data()
    
    # get observation
    latest_data_dict = node.get_latest_data()
    data_dict = {}
    # Convert and ensure correct data types and shapes
    data_dict["motor_states"] = np.array(latest_data_dict["motor_states"], dtype=np.float32)
    
    spot_states = np.array(latest_data_dict["spot_states"], dtype=np.float32)
    data_dict["spot_angle"] = spot_states[:3]
    spot_pos = spot_states[3:]
    
    target_pos = np.array(latest_data_dict["target_pos"], dtype=np.float32)
    data_dict["distance_to_stright_line"] = spot_pos[0]
    data_dict["distance_to_goal"] = pow(pow(target_pos[0] - spot_pos[0], 2) + pow(target_pos[2] - spot_pos[2], 2), 0.5)
    data_dict["angle_to_goal"] = np.arctan2(target_pos[2] - spot_pos[2], target_pos[0] - spot_pos[0])
    return data_dict

