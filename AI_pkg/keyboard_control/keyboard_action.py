"""
KeyboardAction class
"""
import dataclasses
import pickle

@dataclasses.dataclass
class KeyboardAction:
    """
    KeyboardAction class
    """
    filename = "keyboard_control/keyboard_actions/motor_data_2025-03-21.pkl"
    filename_ik_craw = "keyboard_control/keyboard_actions/ik_craw_2025-03-25.pkl"
    filename_ik_trot = "keyboard_control/keyboard_actions/ik_tort_2025-03-26.pkl"
    
    with open(filename, "rb") as f:
        loaded_data = pickle.load(f)
    with open(filename_ik_craw, "rb") as f:
        loaded_data_ik_craw = pickle.load(f)
    with open(filename_ik_trot, "rb") as f:
        loaded_data_ik_trot = pickle.load(f)
    
    FORWARD_STEP_1 = loaded_data["FORWARD_STEP_1"]

    FORWARD_STEP_2 = loaded_data_ik_trot["FORWARD_STEP_2"]
    FORWARD_STEP_3 = loaded_data_ik_trot["FORWARD_STEP_3"]
    
    FORWARD_STEP_4 = loaded_data_ik_trot["FORWARD_STEP_4"]
    FORWARD_STEP_5 = loaded_data_ik_trot["FORWARD_STEP_5"]
    FORWARD_STEP_6 = loaded_data["SPOT_DANCE_6"]
    
    # make dog height from 0.15 to 0.20
    FORWARD_STEP_j = loaded_data["motor_stand_up"]
    # make dog height form 0.25 to 0.15
    FORWARD_STEP_u = loaded_data["motor_stand_down"]
