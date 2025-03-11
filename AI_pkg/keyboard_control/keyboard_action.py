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
    filename = "keyboard_control/keyboard_actions/motor_data_2025-03-11.pkl"
    with open(filename, "rb") as f:
        loaded_data = pickle.load(f)
    
    FORWARD_STEP_1 = loaded_data["motor_front_lift_init"]

    FORWARD_STEP_2 = loaded_data["motor_front_stand_A"]
    FORWARD_STEP_3 = loaded_data["motor_front_lift_step_A"]
    
    FORWARD_STEP_4 = loaded_data["motor_front_stand_B"]
    FORWARD_STEP_5 = loaded_data["motor_front_lift_step_B"]
    
    # make dog height from 0.15 to 0.20
    FORWARD_STEP_j = loaded_data["motor_stand_up"]
    # make dog height form 0.25 to 0.15
    FORWARD_STEP_u = loaded_data["motor_stand_down"]
