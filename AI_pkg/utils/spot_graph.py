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
            base_tilt=None, is_visited=False, joint_angle=None,
            ):
        self.base_position = base_position
        self.base_rotation = base_rotation
        if base_tilt is None:
            base_tilt = [0, 0]

        self.base_tilt = base_tilt
        self.joint_angle = joint_angle or [0] * 12
        self.is_visited = is_visited
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
    joint_lengths = [0.0801, 0.1501, 0.1451]
    base_translation = [0.3740, 0.1670, 0]
    spot_leg = SpotLeg(joint_lengths, [0, 0, 0])

    root = SpotNode([0, 0, 0.2], [0, 0, 0])
    root.joint_angle = spot_state_creater(spot_leg,
                            root.base_position,
                            root.base_rotation,
                            base_translation,
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
            new_pos = [round(node.base_position[i] + offset[i] * multiple, 3) for i in range(3)]
            new_rot = [round(node.base_rotation[i] + offset[i+3] * multiple, 3) for i in range(3)]
            new_tilt = [round(node.base_tilt[i] + offset[i+6] * multiple, 3) for i in range(2)]

            if new_pos[2] < 0.05 or any(abs(r) > l for r, l in zip(new_rot, [40, 30, 15])):
                continue
            if graph.get_node(new_pos, new_rot, new_tilt):
                continue

            joint_angle = spot_state_creater(
                spot_leg, new_pos, new_rot, base_translation, new_tilt, leg_end_position)
            if any(abs(joint_angle[i]) > 30 for i in [0, 3, 6, 9]) or \
               any(abs(joint_angle[i]) > 160 for i in [2, 5, 8, 11]) or \
               np.isnan(joint_angle).any():
                continue
            if any(joint_angle[i] < joint_angle[i+1] for i in [1, 4, 7, 10]):
                continue

            new_node = SpotNode(new_pos, new_rot, new_tilt, joint_angle=joint_angle)
            node.connect(direction, new_node)
            graph.add_node(new_node)
            queue.append((new_node, level + 1))
            pbar.update(1)

    pbar.close()
    print(f"Graph generated in {time.time() - start_time:.2f} seconds")
    print(f"Total nodes: {len(graph.node_map)}")
    return graph

async def save_spot_graph_nodes(db, graph):
    """
    Save the Spot graph nodes to the database.
    """
    node_objs = []
    node_key_map = {}

    for (pos, rot, tilt), node in graph.node_map.items():
        obj = SpotNode(
                pos, rot, tilt, joint_angle=node.joint_angle,
                is_visited=int(node.is_visited))
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
