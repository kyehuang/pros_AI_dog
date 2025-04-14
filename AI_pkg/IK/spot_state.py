import numpy as np
from math import cos, sin, radians, degrees

from SpotLeg import SpotLeg

def rotate_point(point, angles_deg, order='xyz', rotate_axes=False):
    """
    Rotate a 3D point around x, y, z axes.

    Args:
        point: list or np.array of shape (3,), e.g. [x, y, z]
        angles_deg: list or tuple of angles in degrees, e.g. [45, 0, 0]
        order: rotation order, default 'xyz'
        rotate_axes: if True, rotate the coordinate system (use transpose of matrix)

    Returns:
        np.array: rotated point
    """
    point = np.array(point)
    angles_rad = np.radians(angles_deg)

    # Rotation matrices
    def rx(theta):
        return np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta),  np.cos(theta)]
        ])

    def ry(theta):
        return np.array([
            [ np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])

    def rz(theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0, 0, 1]
        ])

    rot_map = {'x': rx, 'y': ry, 'z': rz}
    r = np.eye(3)

    for axis, theta in zip(order, angles_rad):
        r_axis = rot_map[axis](theta)
        r = r_axis @ r  # Apply in order

    if rotate_axes:
        r = r.T  # For rotating the coordinate system

    return r @ point

if __name__ == "__main__":
    SpotPosition = [0, 0, 2]
    JointLengths = [1.0, 2.0, 2.0]
    BaseTranslation = [6, 4, 0]
    p = [0, 1, -2]

    # 只旋轉 x 軸 45 度，座標系旋轉
    rotated = rotate_point(p, [45, 0, 0], order='xyz', rotate_axes=True)
    print("旋轉座標軸後的座標:", rotated)

    # 同樣的旋轉，但這次是旋轉點（物體）
    rotated_obj = rotate_point(p, [45, 0, 0], order='xyz', rotate_axes=False)
    print("旋轉點本身後的座標:", rotated_obj)
