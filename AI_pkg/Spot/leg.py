"""
this module is for the leg of the Spot robot
"""
import dataclasses
import numpy as np



@dataclasses.dataclass
class Leg():
    """
    This class defines the leg of the Spot robot
    """
    def __init__(self, leg_type: str,
                 joint_lengths: list, angles: list):
        self.leg_type = leg_type
        self.joint_lengths = joint_lengths
        self.angles = angles


def calculate_ik(leg:Leg, target_position:list, base_postion:list):
    """
    This method is used to calculate leg position
    """
    x = target_position[0] - base_postion[0]
    y = target_position[1] - base_postion[1]
    z = target_position[2] - base_postion[2]

    # print(x, y, z)
    ### calaculate first joint 1 angle
    c = np.sqrt(z * z + y * y)
    d = np.sqrt(c * c - leg.joint_lengths[0] * leg.joint_lengths[0])
    joint_1_angle = np.arctan(d/leg.joint_lengths[0]) + np.arctan(y/-z)
    # print(np.degrees(joint_1_angle))

    ### calaculate first joint 3 angle
    g = np.sqrt(x * x + d * d)
    numerator = g * g - leg.joint_lengths[1] * leg.joint_lengths[1] - leg.joint_lengths[2] * leg.joint_lengths[2]
    denominator = 2 * leg.joint_lengths[1] * leg.joint_lengths[2]
    joint_3_angle = np.arccos(numerator/denominator)
    # print(np.degrees(joint_3_angle))

    ### calaculate first joint 2 angle

    joint_2_angle = np.arctan(x/d) + np.arcsin(leg.joint_lengths[2] * np.sin(joint_3_angle)/g)
    # print(np.degrees(np.arcsin(leg.joint_lengths[2] * np.sin(joint_3_angle)/g)))

    return {"joint_1_angle": np.degrees(joint_1_angle),
            "joint_2_angle": np.degrees(joint_2_angle),
            "joint_3_angle": np.degrees(joint_3_angle)}

if __name__ == "__main__":
    LF_joint_lengths = [1, 2, 2]
    LF_TYPE = "left"
    LF_angles = [0, 0, 0]

    leg1 = Leg(LF_TYPE, LF_joint_lengths, LF_angles)
    targe = [1, 1, -2.8]
    base =  [0, 0, 0]
    result = calculate_ik(leg1, target_position=targe, base_postion=base)
    print(result)
