"""
This module contains functions for calculating the inverse
kinematics of a leg
"""
import numpy as np
import pickle
import datetime
from IK.spot_leg import SpotLeg
import pandas as pd
import matplotlib.pyplot as plt
from typing import List, Union, Optional

def make_linear_interpolation(
        start_points: np.array,
        end_points: np.array,
        num_points: int,
) -> np.array:
    """
    Create a linear interpolation between two sets of points.

    Args:
        start_points (np.ndarray): The starting points (1D or 2D) of the interpolation.
        end_points (np.ndarray): The ending points (same shape as `start_points`).
        num_points (int): The number of points to generate from start to end.

    Returns:
        np.ndarray: An array containing the interpolated points.

    Raises:
        ValueError: If `start_points` and `end_points` have incompatible shapes.
    """
    # Convert inputs to np.ndarray in case lists or tuples were provided
    start_points = np.asarray(start_points)
    end_points = np.asarray(end_points)

    # Ensure shapes match for linear interpolation
    if start_points.shape != end_points.shape:
        raise ValueError(
            f"Incompatible shapes: {start_points.shape} vs {end_points.shape}"
        )

    # Generate linearly spaced points between start and end
    interp_points = np.linspace(start_points, end_points, num_points)

    return interp_points

def draw_leg_trajectory(
        leg_postion:np.array
)->None:
    """
    Draw the leg trajectory
    """
    fig = plt.figure()
    gait_ax = fig.add_subplot(111, projection='3d')

    gait_ax.plot(leg_postion[:, 0], leg_postion[:, 1], leg_postion[:, 2], marker='o')

    gait_ax.set_xlabel('X')
    gait_ax.set_ylabel('Y')
    gait_ax.set_zlabel('Z')

    plt.show()

def calculate_ik(
    positions: np.ndarray,
    leg: 'SpotLeg',  # Assuming 'SpotLeg' is a class type defined elsewhere
    base_position: Optional[Union[List[float], tuple]] = None
) -> List[List[float]]:
    """
    Calculate inverse kinematics (IK) angles for each position in `positions`.

    Args:
        positions (np.ndarray): An array of [x, y, z] coordinates.
        leg (SpotLeg): An instance of SpotLeg (or similar) that provides a `calculate_ik` method.
        base_position (list or tuple, optional): The base position offsets. Defaults to (0.0, 0.0, 0.0).
    
    Returns:
        List[List[float]]: A list of [joint1, joint2, joint3] angle sets (in degrees).
    """
    if base_position is None:
        # Use a tuple to avoid mutable default
        base_position = (0.0, 0.0, 0.0)

    ik_angles = []

    for pos in positions:
        joint_angle = leg.calculate_ik(pos, base_position)

        ik_angles.append([joint_angle[0], joint_angle[1], joint_angle[2]])

    return ik_angles


def create_motor_angles(motor_LF, motor_RF, motor_RB, motor_LB):
    """
    Create a list of motor angles by combining each pair of angles from
    `motor_LF` and `motor_RF` in a specific pattern.

    Each output element is a 12-element list of angles:
    [init_0, init_1, init_2, stand_0, stand_1, stand_2,
     init_0, init_1, init_2, stand_0, stand_1, stand_2]
    
    Args:
        motor_LF (list of list of float): 
            A list where each element is [init_0, init_1, init_2].
        motor_RF (list of list of float): 
            A list where each element is [stand_0, stand_1, stand_2].
        motor_RB (list of list of float): 
            A list where each element is [init_0, init_1, init_2].
        motor_LB (list of list of float): 
            A list where each element is [stand_0, stand_1, stand_2].
    
    Returns:
        list: A list of 12-element lists, each corresponding to the combined angles.
    """
    if len(motor_LF) != len(motor_RF) or len(motor_RF) != len(motor_RB) or len(motor_RB) != len(motor_LB):
        raise ValueError("All input lists must have the same length")
    combined_angles = [
        [
            motor_1[0], motor_1[1], motor_1[2],
            motor_2[0], motor_2[1], motor_2[2],
            motor_3[0], motor_3[1], motor_3[2],
            motor_4[0], motor_4[1], motor_4[2],
        ]
        for motor_1, motor_2, motor_3, motor_4 in zip(motor_LF, motor_RF, motor_RB, motor_LB)
    ]
    return combined_angles

# Example usage:
# motor_A = [[10, 20, 30], [15, 25, 35]]
# motor_B = [[40, 50, 60], [45, 55, 65]]
# result = create_motor_angles(motor_A, motor_B)
# print(result)
# [
#   [10, 20, 30, 40, 50, 60, 10, 20, 30, 40, 50, 60],
#   [15, 25, 35, 45, 55, 65, 15, 25, 35, 45, 55, 65]
# ]


def main():
    # Define leg position
    leg_front_lift_init = np.array([
        [0.0000, 0.0701, -0.225],
        [0.0000, 0.0701, -0.200],
        [0.0000, 0.0701, -0.190],
        [0.0000, 0.0701, -0.185],
        [0.0000, 0.0701, -0.180],
        [0.0000, 0.0701, -0.175],
        [0.0000, 0.0701, -0.170],
        [0.0000, 0.0701, -0.165],
        [0.0000, 0.0701, -0.160],
        [0.0000, 0.0701, -0.155],
        [0.0000, 0.0701, -0.15],
        [0.01, 0.0701, -0.15],
        [0.02, 0.0701, -0.15],
        [0.03, 0.0701, -0.15],
        [0.04, 0.0701, -0.15],
        [0.05, 0.0701, -0.15],
        [0.06, 0.0701, -0.15],
        [0.07, 0.0701, -0.15],
        [0.08, 0.0701, -0.15],
        [0.09, 0.0701, -0.15],
        [0.1, 0.0701, -0.15],
        [0.1, 0.0701, -0.16],
        [0.1, 0.0701, -0.17],
        [0.1, 0.0701, -0.18],
        [0.1, 0.0701, -0.19],
        [0.1, 0.0701, -0.20],
    ])
    leg_front_lift_step = np.array([
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
    leg_down_to_stand_A = make_linear_interpolation(
                        [-0.05, 0.0701, -0.175],
                        [-0.10, 0.0701, -0.20], 25)
    leg_down_to_stand_C = make_linear_interpolation(
                        [0.05, 0.0701, -0.175],
                        [0.00, 0.0701, -0.20], 25)
    leg_front_stand_A = make_linear_interpolation(
                        [-0.05, 0.0701, -0.20],
                        [-0.10, 0.0701, -0.20], len(leg_front_lift_step))
    leg_front_stand_B = make_linear_interpolation(
                        [0.00, 0.0701, -0.20],
                        [-0.05, 0.0701, -0.20], len(leg_front_lift_step))
    leg_front_stand_C = make_linear_interpolation(
                        [0.05, 0.0701, -0.20],
                        [0.00, 0.0701, -0.20], len(leg_front_lift_step))
    leg_stand = np.array([[0.0, 0.0701, -0.20]] * len(leg_front_lift_init))
    leg_stand_A = np.array([[-0.05, 0.0701, -0.20]] * 10)
    leg_stand_B = np.array([[0.00, 0.0701, -0.20]] * 10)
    leg_stand_C = np.array([[0.05, 0.0701, -0.20]] * 10)
    leg_stand_front = np.array([[0.1, 0.0701, -0.20]] * len(leg_front_lift_init))
    leg_stand_back = np.array([[-0.1, 0.0701, -0.20]] * len(leg_front_lift_init))
    leg_stand_front_up = make_linear_interpolation(
                        [-0.1, 0.0701, -0.20],
                        [-0.1, 0.0701, -0.225], 10)
    leg_stand_front_down_A = make_linear_interpolation(
                        [-0.05, 0.0701, -0.20],
                        [-0.05, 0.0701, -0.175], 10)
    leg_stand_front_down_C = make_linear_interpolation(
                        [0.05, 0.0701, -0.20],
                        [0.05, 0.0701, -0.175], 25)
    leg_front_stand_up = make_linear_interpolation(
                        [0.0, 0.0701, -0.20],
                        [0.0, 0.0701, -0.15], 100)
    leg_front_stand_down = make_linear_interpolation(
                        [0.0, 0.0701, -0.15],
                        [0.0, 0.0701, -0.20], 100)
    spotLeg = SpotLeg([0.0701, 0.1501, 0.1451], [0, 0, 0])

    # draw leg motion
    # draw_leg_trajectory(leg_front_lift_init)

    # calculate ik 
    motor_angle_front_lift_init = calculate_ik(
        leg_front_lift_init, spotLeg
    )
    motor_angle_stand = calculate_ik(
        leg_stand, spotLeg
    )
    motor_angle_stand_A = calculate_ik(
        leg_stand_A, spotLeg
    )
    motor_angle_stand_B = calculate_ik(
        leg_stand_B, spotLeg
    )
    motor_angle_stand_C = calculate_ik(
        leg_stand_C, spotLeg
    )
    motor_angle_stand_front = calculate_ik(
        leg_stand_front, spotLeg
    )
    motor_angle_stand_back = calculate_ik(
        leg_stand_back, spotLeg
    )
    motor_angle_front_up = calculate_ik(
        leg_stand_front_up, spotLeg
    )
    motor_angle_front_down_A = calculate_ik(
        leg_stand_front_down_A, spotLeg
    )
    motor_angle_front_down_C = calculate_ik(
        leg_stand_front_down_C, spotLeg
    )
    motor_angle_front_lift_step = calculate_ik(
        leg_front_lift_step, spotLeg
    )
    motor_angle_down_to_stand_A = calculate_ik(
        leg_down_to_stand_A, spotLeg
    )
    motor_angle_down_to_stand_C = calculate_ik(
        leg_down_to_stand_C, spotLeg
    )
    motor_angle_front_stand_A = calculate_ik(
        leg_front_stand_A, spotLeg
    )
    motor_angle_front_stand_B = calculate_ik(
        leg_front_stand_B, spotLeg
    )
    motor_angle_front_stand_C = calculate_ik(
        leg_front_stand_C, spotLeg
    )
    
    motor_angle_stand_up = calculate_ik(
        leg_front_stand_up, spotLeg
    )

    motor_angle_stand_down = calculate_ik(
        leg_front_stand_down, spotLeg
    )

    FORWARD_STEP_1 = create_motor_angles(
        motor_angle_front_lift_init,
        motor_angle_stand,
        motor_angle_stand,
        motor_angle_stand
    ) + create_motor_angles(
        motor_angle_stand_front,
        motor_angle_stand,
        motor_angle_front_lift_init,
        motor_angle_stand
    )
    
    FORWARD_STEP_2 = create_motor_angles(
        motor_angle_front_up,
        motor_angle_stand_B,
        motor_angle_front_down_A,
        motor_angle_stand_C
    ) + create_motor_angles(
        motor_angle_front_lift_step,
        motor_angle_front_stand_B,
        motor_angle_down_to_stand_A,
        motor_angle_front_stand_C
    )

    FORWARD_STEP_3 = create_motor_angles(
        motor_angle_front_down_C,
        motor_angle_stand_A,
        motor_angle_front_up,
        motor_angle_stand_B
    ) + create_motor_angles(
        motor_angle_down_to_stand_C,
        motor_angle_front_stand_A,
        motor_angle_front_lift_step,
        motor_angle_front_stand_B
    )

    FORWARD_STEP_4 = create_motor_angles(
        motor_angle_stand_B,
        motor_angle_front_up,
        motor_angle_stand_C,
        motor_angle_front_down_A
    ) + create_motor_angles(
        motor_angle_front_stand_B,
        motor_angle_front_lift_step,
        motor_angle_front_stand_C,
        motor_angle_down_to_stand_A
    )

    FORWARD_STEP_5 = create_motor_angles(
        motor_angle_stand_A,
        motor_angle_front_down_C,
        motor_angle_stand_B,
        motor_angle_front_up
    ) + create_motor_angles(
        motor_angle_front_stand_A,
        motor_angle_down_to_stand_C,
        motor_angle_front_stand_B,
        motor_angle_front_lift_step
    )

    SPOT_DANCE_6 = create_motor_angles(
        motor_angle_stand_down,
        motor_angle_stand_up,
        motor_angle_stand_down,
        motor_angle_stand_up
    ) + create_motor_angles(
        motor_angle_stand_up,
        motor_angle_stand_down,
        motor_angle_stand_up,
        motor_angle_stand_down
    ) 

    motor_stand_up = create_motor_angles(
        motor_angle_stand_up,
        motor_angle_stand_up,
        motor_angle_stand_up,
        motor_angle_stand_up
    )

    motor_stand_down = create_motor_angles(
        motor_angle_stand_down,
        motor_angle_stand_down,
        motor_angle_stand_down,
        motor_angle_stand_down
    )
    
    # print(motor_front_lift_init)
    # print(motor_front_stand_A)
    # print(motor_front_lift_step_A)
    # print(motor_front_stand_B)
    # print(motor_front_lift_step_B)
    # print(motor_stand_up)
    # print(motor_stand_down)

    data_dict = {
        "FORWARD_STEP_1": FORWARD_STEP_1,
        "FORWARD_STEP_2": FORWARD_STEP_2,
        "FORWARD_STEP_3": FORWARD_STEP_3,
        "FORWARD_STEP_4": FORWARD_STEP_4,
        "FORWARD_STEP_5": FORWARD_STEP_5,
        "SPOT_DANCE_6": SPOT_DANCE_6,
        "motor_stand_up": motor_stand_up,
        "motor_stand_down": motor_stand_down
    }

    # Capture current date as a string
    current_date = datetime.datetime.now().strftime("%Y-%m-%d")

    # Build a filename with the date embedded
    filename = f"motor_data_{current_date}.pkl"
    path = "keyboard_control/keyboard_actions/"
    with open(path+filename, "wb") as f:
        pickle.dump(data_dict, f)
    
    print(f"Data saved to {filename}")

if __name__ == "__main__":
    main()
