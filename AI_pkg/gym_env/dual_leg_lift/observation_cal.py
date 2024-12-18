"""
This module is used to get observation from AI_dog_node.
"""
import copy
import numpy as np
from ros_receive_and_processing.ai_dog_node import AIDogNode

def get_observation(node : AIDogNode):
    """
    Get observation from AI_dog_node
    """
    # reset observation
    node.reset_latest_data()

    # get observation
    latest_data_dict = node.get_latest_data()
    data_dict = {}
    # Convert and ensure correct data types and shapes
    spot_states = np.array(latest_data_dict["spot_states"], dtype=np.float32)
    data_dict["spot_angle"] = spot_states[:3]

    angle_x_raw = copy.copy(data_dict["spot_angle"][0])
    angle_z_raw = copy.copy(data_dict["spot_angle"][2])

    angle_x = angle_x_raw if angle_x_raw < 180 else angle_x_raw - 360
    angle_z = angle_z_raw if angle_z_raw < 180 else angle_z_raw - 360

    angle_x += 12
    angle_z += 12


    # make the observation discrete
    obs_x = max(int(angle_x),  0)
    obs_x = min(int(obs_x), 24)
    obs_x = round(obs_x / 2)

    obs_z = max(int(angle_z),  0)
    obs_z = min(int(obs_z), 24)
    obs_z = round(obs_z / 2)
    obs = (obs_x * 13) + obs_z

    return data_dict, obs
