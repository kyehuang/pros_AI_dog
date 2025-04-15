import numpy as np
import datetime
import pickle
from IK.spot_leg import SpotLeg
from ik_main import make_linear_interpolation, calculate_ik, create_motor_angles
def translate_step(step, length):
    for i in range(len(step)):
        step[i][0] += length

    print("Translated")
    print(step[0][0])
    print((step[-1][0] + step[0][0]) / 2)
    print(step[-1][0])
    return step
def main():
        spotLeg = SpotLeg([0.0701, 0.1501, 0.1451], [0, 0, 0])
        leg_front_lift_step = np.array([
            [-0.075, 0.0701, -0.200],
            [-0.075, 0.0701, -0.195],
            [-0.075, 0.0701, -0.190],
            [-0.075, 0.0701, -0.185],
            [-0.075, 0.0701, -0.180],
            [-0.075, 0.0701, -0.175],
            [-0.075, 0.0701, -0.170],
            [-0.075, 0.0701, -0.165],
            [-0.075, 0.0701, -0.160],
            [-0.075, 0.0701, -0.155],
            [-0.075, 0.0701, -0.15],
            [-0.060, 0.0701, -0.15],
            [-0.045, 0.0701, -0.15],
            [-0.030, 0.0701, -0.15],
            [-0.015, 0.0701, -0.15],
            [0.000, 0.0701, -0.15],
            [0.015, 0.0701, -0.15],
            [0.030, 0.0701, -0.15],
            [0.045, 0.0701, -0.15],
            [0.060, 0.0701, -0.15],
            [0.075, 0.0701, -0.15],
            [0.075, 0.0701, -0.17],
            [0.075, 0.0701, -0.19],
            [0.075, 0.0701, -0.20],
        ])
        
        leg_front_lift_step = translate_step(leg_front_lift_step, -0.055)

        leg_stand =  make_linear_interpolation(
                [-0.055, 0.0701, -0.20], 
                [-0.055, 0.0701, -0.20], 24)
        leg_front_stand_A = make_linear_interpolation(
                [0.020, 0.0701, -0.20], 
                [-0.055, 0.0701, -0.20], 24)
        leg_front_stand_B = make_linear_interpolation(
                [-0.055, 0.0701, -0.20], 
                [-0.130, 0.0701, -0.20], 24)

        motor_angle_front_lift_step = calculate_ik(
              leg_front_lift_step, spotLeg)
        motor_angle_stand = calculate_ik(leg_stand, spotLeg)
        motor_angle_front_stand_A = calculate_ik(
                leg_front_stand_A, spotLeg)
        motor_angle_front_stand_B = calculate_ik(
                leg_front_stand_B, spotLeg)
        
        FORWARD_STEP_2 = create_motor_angles(
                motor_angle_front_lift_step,
                motor_angle_stand,
                motor_angle_front_lift_step,
                motor_angle_stand)
        
        FORWARD_STEP_3 = create_motor_angles(
                motor_angle_front_stand_A,
                motor_angle_front_stand_B,
                motor_angle_front_stand_A,
                motor_angle_front_stand_B)
        
        FORWARD_STEP_4 = create_motor_angles(
                motor_angle_stand,
                motor_angle_front_lift_step,
                motor_angle_stand,
                motor_angle_front_lift_step)

        FORWARD_STEP_5 = create_motor_angles(
                motor_angle_front_stand_B,
                motor_angle_front_stand_A,
                motor_angle_front_stand_B,
                motor_angle_front_stand_A)
        
        data_dict = {
            "FORWARD_STEP_2": FORWARD_STEP_2,
            "FORWARD_STEP_3": FORWARD_STEP_3,
            "FORWARD_STEP_4": FORWARD_STEP_4,
            "FORWARD_STEP_5": FORWARD_STEP_5
        }

         # Capture current date as a string
        current_date = datetime.datetime.now().strftime("%Y-%m-%d")

        # Build a filename with the date embedded
        filename = f"ik_tort_{current_date}.pkl"
        path = "keyboard_control/keyboard_actions/"
        with open(path+filename, "wb") as f:
            pickle.dump(data_dict, f)

        print(f"Data saved to {filename}")

if __name__ == "__main__":
    main()
