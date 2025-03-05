"""
This module contains functions for calculating the inverse kinematics of a leg
"""
import numpy as np
from Spot.leg import Leg
import pandas as pd
import matplotlib.pyplot as plt


def make_linear_interpolation(start_points: np.array,
                              end_points: np.array, num_points: int) -> np.array:
    """
    Create a linear interpolation between start and end points.

    Args:
        start_points (np.array): The starting points of the interpolation.
        end_points (np.array): The ending points of the interpolation.
        num_points (int): The number of points to interpolate between start and end.

    Returns:
        np.array: An array containing the interpolated points.
    """

    # Generate linear interpolation points
    leg = np.linspace(start_points, end_points, num_points)

    # Repeat the array to ensure consistent length
    repetitions = (num_points + 2) // num_points
    leg = np.tile(leg, (repetitions, 1))

    return leg

def rotate_list(lst, shift):
    """
    Rotates a 2D list by shifting elements downward.

    Args:
        lst (list of list of int/float): The input 2D list to rotate.
        shift (int): The number of positions to shift the list downward.

    Returns:
        list of list of int/float: The rotated 2D list.
    """
    array = np.array(lst)
    shift = shift % len(array)  # Handle cases where shift is greater than the list length
    rotated_array = np.roll(array, shift, axis=0)
    return rotated_array.tolist()

def store_list_as_csv(lst: list, file_path: str) -> str:
    """
    Store a list as a CSV file.

    Args:
        lst (list): The input list to store.
        file_path (str): The file path to store the list as a CSV file.
    """
    # Create a DataFrame from the list
    data_frame = pd.DataFrame(lst)

    # Save the DataFrame as a CSV file
    data_frame.to_csv(file_path, index=False)

    return file_path

def load_list_from_csv(file_path: str) -> list:
    """
    Load a list from a CSV file.

    Args:
        file_path (str): The file path to load the list from.

    Returns:
        list: The loaded list from the CSV file.
    """
    # Load the CSV file as a DataFrame
    data_frame = pd.read_csv(file_path)

    # Convert the DataFrame to a list
    lst = data_frame.values.tolist()

    return lst

def calculate_ik(positions:np.array, leg:Leg,
                          base_position:list = None) -> list:
    """
    calculate ik to motor angle
    """
    if base_position is None:
        base_position = [0, 0, 0]
    angles_list = []
    for pos in positions:
        angles = [0] * 3
        joint_angle = leg.calculate_ik(pos, base_position)

        angles[0] = round(float(joint_angle["joint_1_angle"] - 90), 2)
        angles[1] = round(float(180 - (90 - joint_angle["joint_2_angle"])), 2)
        angles[2] =  round(float(joint_angle["joint_3_angle"]), 2)

        angles_list.append(angles)
    return angles_list

def draw_leg_trajectory(leg_postion:np.array)->None:
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


# Define the leg positions for lifting and standing

# leg_lift = np.array([
#                     [0.0000, 0.1501, -0.3000],
#                     # [0.0000, 0.0355, -0.2875],
#                     [0.0000, 0.1501, -0.2750],
#                     [0.0000, 0.1501, -0.2500],
#                     [0.0250, 0.1501, -0.2375],
#                     [0.0375, 0.0355, -0.2250],
#                     [0.0500, 0.0355, -0.2125],
#                     [0.0625, 0.0355, -0.20],
#                     [0.0750, 0.0355, -0.20],
#                     [0.0875, 0.0355, -0.20],
#                     [0.1000, 0.0355, -0.20],
#                     [0.1125, 0.0355, -0.20],
#                     [0.1250, 0.0355, -0.20],
#                     [0.1375, 0.0355, -0.20],
#                     [0.1500, 0.0355, -0.20],
#                     [0.1750, 0.0355, -0.2150],
#                     [0.2000, 0.0355, -0.2250],
#                     [0.2250, 0.0355, -0.2500],
#                     [0.2500, 0.0355, -0.2750],
#                     [0.2500, 0.0355, -0.300],
#                     [0.2500, 0.0355, -0.300],
# ])

leg_INIT = np.array([
                    [0.0000, 0.0701, -0.25], #
                    [0.0000, 0.0701, -0.24],
                    [0.0000, 0.0701, -0.23],
                    [0.0000, 0.0701, -0.22],
                    [0.0000, 0.0701, -0.21],
                    [0.0000, 0.0701, -0.20], #
                    [0.0000, 0.0701, -0.19],
                    [0.0000, 0.0701, -0.18],
                    [0.0000, 0.0701, -0.17],
                    [0.0000, 0.0701, -0.16],
                    [0.0000, 0.0701, -0.15], #
                    [0.01, 0.0701, -0.15],
                    [0.02, 0.0701, -0.15],
                    [0.03, 0.0701, -0.15],
                    [0.04, 0.0701, -0.15],
                    [0.05, 0.0701, -0.15], #
                    [0.06, 0.0701, -0.15],
                    [0.07, 0.0701, -0.15],
                    [0.08, 0.0701, -0.15],
                    [0.09, 0.0701, -0.15],
                    [0.1, 0.0701, -0.15], #
                    [0.1, 0.0701, -0.17],
                    [0.1, 0.0701, -0.19],
                    [0.1, 0.0701, -0.21],
                    [0.1, 0.0701, -0.23],
                    [0.1, 0.0701, -0.25], #
                    ])

leg_stand = np.array([[0.0, 0.0701, -0.25]] * len(leg_INIT))
# 1: x axis, 2: y axis, 3: z axis

leg_LF = Leg("left Forward", [0.0701, 0.1501, 0.1451], [0, 0, 0])
# 1: first leg, 2: second leg 3: third leg

leg_stand_test = make_linear_interpolation([0.15, 0.0355, -0.3],
                                           [0.0, 0.0355, -0.3],len(leg_INIT))
# leg_stand_test = make_linear_interpolation([0.25, 0.0355, -0.3],
#                                            [0.0, 0.0355, -0.3],len(leg_lift))
typeA_INIT_L = make_linear_interpolation([0.1, 0.0701, -0.25], 
                                       [0.0, 0.0701, -0.25], 10)

typeA_INIT_R = make_linear_interpolation([0.0, 0.0701, -0.25], 
                                       [-0.1, 0.0701, -0.25], 10)

typeA_lift = np.array([
                    [-0.100, 0.0701, -0.25], #
                    [-0.100, 0.0701, -0.24],
                    [-0.100, 0.0701, -0.23],
                    [-0.100, 0.0701, -0.22],
                    [-0.100, 0.0701, -0.21],
                    [-0.100, 0.0701, -0.20], #
                    [-0.100, 0.0701, -0.19],
                    [-0.100, 0.0701, -0.18],
                    [-0.100, 0.0701, -0.17],
                    [-0.100, 0.0701, -0.16],
                    [-0.100, 0.0701, -0.15], #
                    [-0.08, 0.0701, -0.15],
                    [-0.06, 0.0701, -0.15],
                    [-0.04, 0.0701, -0.15],
                    [-0.02, 0.0701, -0.15],
                    [0.00, 0.0701, -0.15], #
                    [0.02, 0.0701, -0.15],
                    [0.04, 0.0701, -0.15],
                    [0.06, 0.0701, -0.15],
                    [0.08, 0.0701, -0.15],
                    [0.1, 0.0701, -0.15], #
                    [0.1, 0.0701, -0.17],
                    [0.1, 0.0701, -0.19],
                    [0.1, 0.0701, -0.21],
                    [0.1, 0.0701, -0.23],
                    [0.1, 0.0701, -0.25], #
                    ])

draw_leg_trajectory(leg_INIT)
draw_leg_trajectory(typeA_lift)
# draw_leg_trajectory(leg_stand_test)
# exit()

# Calculate the motor angles for lifting and standing

motor_INIT_angles = calculate_ik(leg_INIT, leg_LF)
leg_stand_test_angles = calculate_ik(leg_stand_test, leg_LF)
motor_stand_angles = calculate_ik(leg_stand, leg_LF)


# Type A
motor_typeA_INIT_L = calculate_ik(typeA_INIT_L, leg_LF)
motor_typeA_INIT_R = calculate_ik(typeA_INIT_R, leg_LF)
motor_typeA_LIFT = calculate_ik(typeA_lift, leg_LF)


# spot step 1
motor_angles = [
    [motor_1[0], motor_1[1], motor_1[2], motor_2[0], motor_2[1], motor_2[2],
     motor_1[0], motor_1[1], motor_1[2], motor_2[0], motor_2[1], motor_2[2]]
     for motor_1, motor_2 in zip(motor_INIT_angles, motor_stand_angles)
]

print("spot step INIT",motor_angles)


motor_angles = [
    [motor_1[0], motor_1[1], motor_1[2], motor_2[0], motor_2[1], motor_2[2],
     motor_1[0], motor_1[1], motor_1[2], motor_2[0], motor_2[1], motor_2[2]]
     for motor_1, motor_2 in zip(motor_typeA_INIT_L, motor_typeA_INIT_R)
]
print(motor_angles)

motor_angles = [
    [motor_1[0], motor_1[1], motor_1[2], motor_2[0], motor_2[1], motor_2[2],
     motor_1[0], motor_1[1], motor_1[2], motor_2[0], motor_2[1], motor_2[2]]
     for motor_1, motor_2 in zip(motor_stand_angles, motor_typeA_LIFT)
]
print(motor_angles)

motor_angles = [
    [motor_1[0], motor_1[1], motor_1[2], motor_2[0], motor_2[1], motor_2[2],
     motor_1[0], motor_1[1], motor_1[2], motor_2[0], motor_2[1], motor_2[2]]
     for motor_1, motor_2 in zip(motor_typeA_INIT_R, motor_typeA_INIT_L)
]
print(motor_angles)

motor_angles = [
    [motor_1[0], motor_1[1], motor_1[2], motor_2[0], motor_2[1], motor_2[2],
     motor_1[0], motor_1[1], motor_1[2], motor_2[0], motor_2[1], motor_2[2]]
     for motor_1, motor_2 in zip(motor_typeA_LIFT, motor_stand_angles)
]
print(motor_angles)
# Store the motor angles as a CSV file
# motor_angles_file_path = store_list_as_csv(motor_angles,
#                      "AI_pkg/Spot/action/motor_angles_step1.csv")

# spot step 2
# motor_angles = [
#     [motor_1[0], motor_1[1], motor_1[2], motor_2[0], motor_2[1], motor_2[2],
#      motor_1[0], motor_1[1], motor_1[2], motor_2[0], motor_2[1], motor_2[2]]
#      for motor_1, motor_2 in zip(leg_stand_test_angles, motor_lift_angles)
# ]
# print("spot step LEFT",motor_angles)

# Store the motor angles as a CSV file
# motor_angles_file_path = store_list_as_csv(motor_angles,
#                       "AI_pkg/Spot/action/motor_angles_step2.csv")

# spot step 3
# motor_angles = [
#     [motor_1[0], motor_1[1], motor_1[2], motor_2[0], motor_2[1], motor_2[2],
#      motor_1[0], motor_1[1], motor_1[2], motor_2[0], motor_2[1], motor_2[2]]
#      for motor_1, motor_2 in zip(motor_lift_angles, leg_stand_test_angles)
# ]
# print("spot step RIGHT",motor_angles)

# Store the motor angles as a CSV file
# motor_angles_file_path = store_list_as_csv(motor_angles,
#                       "AI_pkg/Spot/action/motor_angles_step3.csv")
