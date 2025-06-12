"""
This script generates a graph of nodes representing the possible 
positions and orientations of the Spot robot.
Each node contains information about its position, rotation, 
joint angles, and connections to neighboring nodes.
"""
import time
from collections import deque
from tqdm import tqdm

import numpy as np

from IK.spot_state import spot_state_creater
from IK.spot_leg import SpotLeg


MAX_ROTATION = [40, 30, 15]
MAX_LEVEL = 15
JOINT_LIMITS = {
    "base": 30,
    "knee": 160
}

JOINT_LENGTHS = [0.0801, 0.1501, 0.1451]  # lengths of the Spot robot's joints in meters
BASE_TRANSLATION = [0.3740, 0.1670, 0]
SPOT_LEG = SpotLeg(JOINT_LENGTHS, [0, 0, 0])

reversed_key_mapping = {
    "up": "down", "down": "up",
    "left": "right", "right": "left",
    "front": "back", "back": "front",
    "rx_plus": "rx_minus", "rx_minus": "rx_plus",
    "ry_plus": "ry_minus", "ry_minus": "ry_plus",
    "rz_plus": "rz_minus", "rz_minus": "rz_plus",
    "tilt_lf_rb_plus": "tilt_lf_rb_minus",
    "tilt_lf_rb_minus": "tilt_lf_rb_plus",
    "tilt_rf_lb_plus": "tilt_rf_lb_minus",
    "tilt_rf_lb_minus": "tilt_rf_lb_plus"
}
mapping = {
    "up": None, "down": None,
    "left": None, "right": None,
    "front": None, "back": None,
    "rx_plus": None, "rx_minus": None,
    "ry_plus": None, "ry_minus": None,
    "rz_plus": None, "rz_minus": None,
    "tilt_lf_rb_plus": None,
    "tilt_lf_rb_minus": None,
    "tilt_rf_lb_plus": None,
    "tilt_rf_lb_minus": None,
}

class SpotNode:
    """
    A class representing a node in the Spot robot's graph.
    Each node contains information about its position, rotation, joint angles,
    and connections to neighboring nodes.
    """
    def __init__(
            self, base_position, base_rotation,
            base_tilt=None, joint_angle=None,
            ):
        self.base_position = base_position
        self.base_rotation = base_rotation
        if base_tilt is None:
            base_tilt = [0, 0]

        self.base_tilt = base_tilt
        self.joint_angle = joint_angle or [0] * 12
        self.is_visited = False  # Flag to indicate if the node has been visited
        self.neighbors = mapping.copy()

    def connect(self, direction, node):
        """
        Connect this node to another node in a specified direction.
        The connection is bidirectional, meaning both nodes will reference each other.
        """
        if direction not in self.neighbors:
            raise ValueError(f"Invalid direction: {direction}")
        self.neighbors[direction] = node
        node.neighbors[reversed_key_mapping[direction]] = self

    def __str__(self):
        return f"SpotNode(pos={self.base_position}, rot={self.base_rotation})"

class SpotGraph:
    """
    A class representing a graph of Spot nodes.
    This graph is used to represent the possible positions and orientations of the Spot robot.
    """
    def __init__(self):
        self.node_map = {}

    def add_node(self, node):
        """
        Add a node to the graph. The node is identified by its base position and rotation.
        If a node with the same position and rotation already exists, it will be replaced.
        """
        key = (tuple(node.base_position), tuple(node.base_rotation), tuple(node.base_tilt))
        if key not in self.node_map:
            self.node_map[key] = node

    def get_node(self, base_position, base_rotation, base_tilt):
        """
        Get a node from the graph based on its base position and rotation.
        """
        return self.node_map.get((tuple(base_position), tuple(base_rotation), tuple(base_tilt)))


async def generate_spot_graph(max_level=15, multiple=1, key_mapping= None, leg_end_position=None):
    """
    Generate a graph of Spot nodes based on the Spot robot's joint angles and positions.
    """
    root = SpotNode([0, 0, 0.2], [0, 0, 0])
    root.joint_angle = spot_state_creater(SPOT_LEG,
                            root.base_position,
                            root.base_rotation,
                            BASE_TRANSLATION,
                            root.base_tilt,
                            leg_end_position)

    graph = SpotGraph()
    graph.add_node(root)
    queue = deque([(root, 0)])

    pbar = tqdm(desc="生成節點", unit=" nodes")
    start_time = time.time()

    while queue:
        node, level = queue.popleft()

        if level >= max_level:
            continue
        for direction, offset in key_mapping.items():
            config = {
                "direction": direction,
                "offset": offset,
                "multiple": multiple,
                "leg_end_position": leg_end_position
            }
            new_node = create_valid_node(
                node, graph, config
            )

            if new_node:
                graph.add_node(new_node)
                queue.append((new_node, level + 1))
                pbar.update(1)

    pbar.close()
    print(f"Graph generated in {time.time() - start_time:.2f} seconds")
    print(f"Total nodes: {len(graph.node_map)}")
    return graph

def compute_new_pose(node, offset, multiple):
    """
    Compute the new position, rotation, and tilt of a Spot node based on the current node,
    offset, and multiple factor.
    The new position and rotation are rounded to 3 decimal places.
    """
    new_pos = [round(node.base_position[i] + offset[i] * multiple, 3) for i in range(3)]
    new_rot = [round(node.base_rotation[i] + offset[i + 3] * multiple, 3) for i in range(3)]
    new_tilt = [round(node.base_tilt[i] + offset[i + 6] * multiple, 3) for i in range(2)]
    return {"pos": new_pos, "rot": new_rot, "tilt": new_tilt}

def create_valid_node(node, graph, config):
    """
    Create a new Spot node based on the current node, direction, and offset.
    The new node is only created if it has a valid position, rotation, and joint angles."""
    new_pose = compute_new_pose(node, config["offset"], config["multiple"])

    if not is_valid_position(new_pose["pos"], new_pose["rot"]):
        return None
    if graph.get_node(new_pose["pos"], new_pose["rot"], new_pose["tilt"]):
        return None

    joint_angle = spot_state_creater(
        SPOT_LEG, new_pose["pos"], new_pose["rot"],
        BASE_TRANSLATION, new_pose["tilt"], config["leg_end_position"]
    )
    if not is_valid_joint_angle(joint_angle):
        return None

    new_node = SpotNode(
        new_pose["pos"], new_pose["rot"], new_pose["tilt"],
        joint_angle=joint_angle
    )
    node.connect(config["direction"], new_node)
    return new_node

def is_valid_position(position, rotation) -> bool:
    """
    check if the position and rotation of a Spot node are valid.
    A valid position has a height greater than 0.05 and rotation within specified limits.
    """
    if position[2] < 0.05:
        return False
    if any(abs(r) > max_r for r, max_r in zip(rotation, MAX_ROTATION)):
        return False
    return True

def is_valid_joint_angle(joint_angle: np.ndarray) -> bool:
    """
    Check if the joint angles of a Spot node are valid.
    Valid joint angles are within specified limits and follow a specific sequence.
    JOINT_LIMITS is a dictionary containing the limits for each joint.
    """
    if joint_angle is None or np.isnan(joint_angle).any():
        return False

    base_joint_indices = [0, 3, 6, 9]
    knee_joint_indices = [2, 5, 8, 11]
    angle_sequence_indices = [1, 4, 7, 10]

    if any(abs(joint_angle[i]) > JOINT_LIMITS["base"] for i in base_joint_indices):
        return False
    if any(abs(joint_angle[i]) > JOINT_LIMITS["knee"] for i in knee_joint_indices):
        return False
    if any(joint_angle[i] < joint_angle[i + 1] for i in angle_sequence_indices):
        return False

    return True

async def save_spot_graph_nodes(db, graph):
    """
    Save the Spot graph nodes to the database.
    """
    node_objs = []
    node_key_map = {}

    for (pos, rot, tilt), node in graph.node_map.items():
        obj = SpotNode(
                pos, rot, tilt, joint_angle=node.joint_angle,
            )
        node_objs.append(obj)
        node_key_map[(tuple(pos), tuple(rot))] = obj

    key_to_id = await db.bulk_add_nodes(node_objs)
    return key_to_id

async def save_spot_graph_links(db, graph, key_to_id):
    """
    Save the links between nodes in the Spot graph to the database.
    """
    links = []
    for (pos, rot, tilt), node in graph.node_map.items():
        from_id = key_to_id[tuple(pos + rot + tilt)]
        for direction, neighbor in node.neighbors.items():
            if neighbor:
                key = tuple(
                    neighbor.base_position +
                    neighbor.base_rotation +
                    neighbor.base_tilt
                )
                to_id = key_to_id[key]
                links.append((from_id, to_id, direction))

    await db.bulk_update_direction_links(links)
    print(f"Inserted {len(links)} links into DB.")
