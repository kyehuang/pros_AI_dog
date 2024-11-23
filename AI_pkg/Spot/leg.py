"""
this module is for the leg of the Spot robot
"""
import numpy as np

class Leg():
    """
    This class defines the leg of the Spot robot
    """
    def __init__(self, leg_type: str,
                 joint_lengths: list, angles: list):
        self.leg_type = leg_type
        self.joint_lengths = joint_lengths
        self.angles = angles


def ik(leg:Leg, target_position:list, base_postion:list):
    """
    This method is used to calculate leg position
    """
    x = target_position[0] - base_postion[0]
    y = target_position[1] - base_postion[1]
    z = target_position[2] - base_postion[2]

    print(x, y, z)
    ### calaculate first joint 1 angle
    c = np.sqrt(z * z + y * y)
    d = np.sqrt(c * c - leg.joint_lengths[0] * leg.joint_lengths[0])
    joint_1_angle = np.arctan(d/leg.joint_lengths[0]) + np.arctan(y/z)
    print(np.degrees(joint_1_angle))

if __name__ == "__main__":
    LF_joint_lengths = [0.036, 0.210, 0.211]
    LF_type = "left"
    LF_angles = [0, 0, 0]

    leg1 = Leg(LF_type, LF_joint_lengths, LF_angles)
    targe = [0, 0.036, -0.3]
    base =  [0, 0, 0]
    ik(leg1, target_position=targe, base_postion=base)
