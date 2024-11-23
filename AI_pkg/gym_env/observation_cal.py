"""
This module is used to get observation from AI_dog_node.
"""
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

    angle_x_raw = data_dict["spot_angle"][0]
    angle_x = angle_x_raw if angle_x_raw < 180 else angle_x_raw - 360
    angle_x += 12

    angle_z_raw = data_dict["spot_angle"][2]
    angle_z = angle_z_raw if angle_z_raw < 180 else angle_z_raw - 360

    angle_z += 12


    # spot_pos = spot_states[3:]

    # target_pos = np.array(latest_data_dict["target_pos"], dtype=np.float32)
    # data_dict["distance_to_stright_line"] = spot_pos[0]

    obs_x = max(int(angle_x),  0)
    obs_x = min(int(obs_x), 24)
    obs_x = round(obs_x / 2)

    obs_z = max(int(angle_z),  0)
    obs_z = min(int(obs_z), 24)
    obs_z = round(obs_z / 2)

    print("obs_x: ", obs_x, " obs_z: ", obs_z, " angle_x: ", angle_x,
                " angle_z: ", angle_z," obxx*obsz: ", obs_x * obs_z)
    return data_dict, obs_x * obs_z
