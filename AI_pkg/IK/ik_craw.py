import numpy as np
import datetime
import pickle
from SpotLeg import SpotLeg
from ik_main import make_linear_interpolation, calculate_ik, create_motor_angles

def main():
        # Create a SpotLeg object
        spotLeg = SpotLeg([0.0701, 0.1501, 0.1451], [0, 0, 0])

        # Create a lift step 
        # stand: lift the leg up and stay at 0.20
        # down: lift the leg up and down the leg
        leg_front_lift_step_down = np.array([
            [-0.100, 0.0701, -0.220],
            [-0.100, 0.0701, -0.205],
            [-0.100, 0.0701, -0.200],
            [-0.100, 0.0701, -0.195],
            [-0.100, 0.0701, -0.190],
            [-0.100, 0.0701, -0.185],
            [-0.100, 0.0701, -0.180],
            [-0.100, 0.0701, -0.175],
            [-0.100, 0.0701, -0.170],
            [-0.100, 0.0701, -0.165],
            [-0.100, 0.0701, -0.160],
            [-0.100, 0.0701, -0.155],
            [-0.100, 0.0701, -0.15],
            [-0.08, 0.0701, -0.15],
            [-0.06, 0.0701, -0.15],
            [-0.04, 0.0701, -0.15],
            [-0.02, 0.0701, -0.15],
            [0.00, 0.0701, -0.15],
            [0.01, 0.0701, -0.15],
            [0.02, 0.0701, -0.15],
            [0.03, 0.0701, -0.15],
            [0.04, 0.0701, -0.15],
            [0.05, 0.0701, -0.15],
            [0.05, 0.0701, -0.17],
            [0.05, 0.0701, -0.19],
            [0.05, 0.0701, -0.20],
        ])
        leg_front_lift_step = np.array([
            [-0.150, 0.0701, -0.200],
            [-0.150, 0.0701, -0.200],
            [-0.150, 0.0701, -0.200],
            [-0.150, 0.0701, -0.195],
            [-0.150, 0.0701, -0.190],
            [-0.150, 0.0701, -0.185],
            [-0.150, 0.0701, -0.180],
            [-0.150, 0.0701, -0.175],
            [-0.150, 0.0701, -0.170],
            [-0.150, 0.0701, -0.165],
            [-0.150, 0.0701, -0.160],
            [-0.150, 0.0701, -0.155],
            [-0.150, 0.0701, -0.15],
            [-0.12, 0.0701, -0.15],
            [-0.09, 0.0701, -0.15],
            [-0.06, 0.0701, -0.15],
            [-0.03, 0.0701, -0.15],
            [0.00, 0.0701, -0.15],
            [0.01, 0.0701, -0.15],
            [0.02, 0.0701, -0.15],
            [0.03, 0.0701, -0.15],
            [0.04, 0.0701, -0.15],
            [0.05, 0.0701, -0.15],
            [0.05, 0.0701, -0.17],
            [0.05, 0.0701, -0.19],
            [0.05, 0.0701, -0.20],
        ])

        leg_down_to_stand_A = make_linear_interpolation(
                    [-0.05, 0.0701, -0.180],
                    [-0.10, 0.0701, -0.200], 26)
        leg_front_stand_A = make_linear_interpolation(
                    [-0.05, 0.0701, -0.200],
                    [-0.10, 0.0701, -0.200], 26)
        leg_front_stand_B = make_linear_interpolation(
                        [0.00, 0.0701, -0.20],
                        [-0.05, 0.0701, -0.20], 26)
        leg_front_stand_C = make_linear_interpolation(
                        [0.05, 0.0701, -0.20],
                        [0.00, 0.0701, -0.20], 26)
        leg_front_stand_D = make_linear_interpolation(
                        [-0.10, 0.0701, -0.20],
                        [-0.15, 0.0701, -0.20], 26)
        leg_stand_A_down = make_linear_interpolation(
                    [-0.10, 0.0701, -0.200],
                    [-0.10, 0.0701, -0.220], 26)
        leg_stand_C = make_linear_interpolation(
                        [0.00, 0.0701, -0.20],
                        [0.00, 0.0701, -0.20], 26)
        leg_stand_B_up = make_linear_interpolation(
                        [-0.05, 0.0701, -0.20],
                        [-0.05, 0.0701, -0.18], 26)
        leg_stand_step = make_linear_interpolation(
                        [-0.100, 0.0701, -0.20],
                        [-0.100, 0.0701, -0.20], 26)
        motor_angle_lift_step = calculate_ik(
              leg_front_lift_step, spotLeg)
        motor_angle_lift_step_down = calculate_ik(
              leg_front_lift_step_down, spotLeg)
        motor_angle_down_to_stand_A = calculate_ik(
                leg_down_to_stand_A, spotLeg)
        motor_angle_front_stand_A = calculate_ik(
                leg_front_stand_A, spotLeg)
        motor_angle_front_stand_B = calculate_ik(
                leg_front_stand_B, spotLeg)
        motor_angle_front_stand_C = calculate_ik(
                leg_front_stand_C, spotLeg)
        motor_angle_stand_C = calculate_ik(
                leg_stand_C, spotLeg)
        motor_angle_stand_D = calculate_ik(
                leg_front_stand_D, spotLeg)
        motor_angle_stand_A_down = calculate_ik(
                leg_stand_A_down, spotLeg)
        motor_angle_stand_B_up = calculate_ik(
                leg_stand_B_up, spotLeg)
        motor_angle_stand_step = calculate_ik(
                leg_stand_step, spotLeg)
        
        FORWARD_STEP_2 = create_motor_angles(
            motor_angle_lift_step_down,
            motor_angle_front_stand_B,
            motor_angle_down_to_stand_A,
            motor_angle_front_stand_C
        )
        
        FORWARD_STEP_3 = create_motor_angles(
            motor_angle_front_stand_C,
            motor_angle_front_stand_A,
            motor_angle_stand_D,
            motor_angle_front_stand_B 
        ) + create_motor_angles(
            motor_angle_stand_C,
            motor_angle_stand_A_down,
            motor_angle_lift_step,
            motor_angle_stand_B_up
        )

        FORWARD_STEP_4 = create_motor_angles(
            motor_angle_front_stand_B,
            motor_angle_lift_step_down,
            motor_angle_front_stand_C,
            motor_angle_down_to_stand_A,
        )

        FORWARD_STEP_5 = create_motor_angles(
            motor_angle_front_stand_A,
            motor_angle_front_stand_C,
            motor_angle_front_stand_B,
            motor_angle_stand_D,
        ) + create_motor_angles(
            motor_angle_stand_A_down,
            motor_angle_stand_C,
            motor_angle_stand_B_up,
            motor_angle_lift_step,
        )

        data_dict = {
            "FORWARD_STEP_2": FORWARD_STEP_2,
            "FORWARD_STEP_3": FORWARD_STEP_3,
            "FORWARD_STEP_4": FORWARD_STEP_4,
            "FORWARD_STEP_5": FORWARD_STEP_5
        }

        # create current date as a string
        current_date = datetime.datetime.now().strftime("%Y-%m-%d")

        # build a filename with the date embedded
        filename = f"ik_craw_{current_date}.pkl"
        path = "keyboard_control/keyboard_actions/"
        with open(path+filename, "wb") as f:
            pickle.dump(data_dict, f)
        
        print(f"Data saved to {filename}")
if __name__ == '__main__':
    main()
