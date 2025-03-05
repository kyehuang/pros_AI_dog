"""
This file contains a function that calculates the angle between the initial and rotated vectors.
"""
import numpy as np

def rotation_matrix_x(angle_x: float) -> np.array:
    """
    Calculate the rotation matrix around the x-axis.

    Parameters:
    angle_x (float): The rotation angle (degree).

    Returns:
    np.array: The rotation matrix.
    """
    rad = np.radians(angle_x)
    return np.array([
        [1, 0, 0],
        [0, np.cos(rad), -np.sin(rad)],
        [0, np.sin(rad), np.cos(rad)]
    ])

def rotation_matrix_y(angle_y: float) -> np.array:
    """
    Calculate the rotation matrix around the y-axis.

    Parameters:
    angle (float): The rotation angle (degree).

    Returns:
    np.array: The rotation matrix.
    """
    rad = np.radians(angle_y)
    return np.array([
        [np.cos(rad), 0, np.sin(rad)],
        [0, 1, 0],
        [-np.sin(rad), 0, np.cos(rad)]
    ])

def calculate(angle_x: float, angle_y: float) -> float:
    """
    Calculate the rotated vector and the angle between the initial and rotated vectors.

    Parameters:
    angle_x (float): The rotation angle around the x-axis (degree).
    angle_y (float): The rotation angle around the y-axis (degree).

    Returns:
    angle (float): The angle between the initial and rotated vectors (degree).
    """
    initial_vector = np.array([0, 0, 1])

    r_x_matrix = rotation_matrix_x(angle_x)
    r_y_matrix = rotation_matrix_y(angle_y)

    r_matrix = np.dot(r_y_matrix, r_x_matrix)

    rotated_vector = np.dot(r_matrix, initial_vector)

    dot_product = np.dot(initial_vector, rotated_vector)

    magnitude_initial = np.linalg.norm(initial_vector)
    magnitude_rotated = np.linalg.norm(rotated_vector)

    cos_theta = dot_product / (magnitude_initial * magnitude_rotated)
    theta = np.degrees(np.arccos(cos_theta))

    return theta

if __name__ == "__main__":
    # Test the function
    ANGLE_X = 45
    ANGLE_Y = 0

    angle = calculate(ANGLE_X, ANGLE_Y)

    print(f"The angle between the initial and rotated vectors is {angle:.2f} degrees.")
