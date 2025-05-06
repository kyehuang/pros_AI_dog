import asyncio
import json
import time
from tqdm import tqdm
from collections import deque
import numpy as np
from IK.spot_state import spot_state_creater
from IK.DH import get_foot_position
from IK.spot_leg import SpotLeg

from Spot.spot_graph_db import AsyncSpotGraphDB

class SpotNode:
    """
    A class representing a node in the Spot robot's graph.
    Each node contains information about its position, rotation, joint angles,
    and connections to neighboring nodes.
    """
    def __init__(self, base_position, base_rotation, is_visited=False, joint_angle=None):
        self.base_position = base_position
        self.base_rotation = base_rotation
        self.joint_angle = joint_angle or [0] * 12
        self.is_visited = is_visited
        self.neighbors = {key: None for key in key_mapping}

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
        key = (tuple(node.base_position), tuple(node.base_rotation))
        if key not in self.node_map:
            self.node_map[key] = node

    def get_node(self, base_position, base_rotation):
        """
        Get a node from the graph based on its base position and rotation.
        """
        return self.node_map.get((tuple(base_position), tuple(base_rotation)))

key_mapping = {
    "up": [0, 0, 0.05, 0, 0, 0], "down": [0, 0, -0.05, 0, 0, 0],
    "right": [0, -0.05, 0, 0, 0, 0], "left": [0, 0.05, 0, 0, 0, 0],
    "front": [0.05, 0, 0, 0, 0, 0], "back": [-0.05, 0, 0, 0, 0, 0],
    "rx_plus": [0, 0, 0, 5, 0, 0], "rx_minus": [0, 0, 0, -5, 0, 0],
    "ry_plus": [0, 0, 0, 0, 5, 0], "ry_minus": [0, 0, 0, 0, -5, 0],
    "rz_plus": [0, 0, 0, 0, 0, 5], "rz_minus": [0, 0, 0, 0, 0, -5],
}

reversed_key_mapping = {k: k.replace("plus", "minus") if "plus" in k else k.replace("minus", "plus") for k in key_mapping}
reversed_key_mapping.update({"up": "down", "down": "up", "left": "right", "right": "left", "front": "back", "back": "front"})

async def generate_spot_graph():
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
                            base_translation)

    graph = SpotGraph()
    graph.add_node(root)

    queue = deque([root])
    pbar = tqdm(desc="生成節點", unit=" nodes")
    start_time = time.time()

    multiple = 1

    while queue:
        node = queue.popleft()
        for direction, offset in key_mapping.items():
            new_pos = [round(node.base_position[i] + offset[i] * multiple, 3) for i in range(3)]
            new_rot = [round(node.base_rotation[i] + offset[i+3] * multiple, 3) for i in range(3)]

            if new_pos[2] < 0.05 or any(abs(r) > l for r, l in zip(new_rot, [30, 30, 15])):
                continue
            if graph.get_node(new_pos, new_rot):
                continue

            joint_angle = spot_state_creater(spot_leg, new_pos, new_rot, base_translation)
            if any(abs(joint_angle[i]) > 30 for i in [0, 3, 6, 9]) or \
               any(abs(joint_angle[i]) > 165 for i in [1, 4, 7, 10]) or \
               np.isnan(joint_angle).any():
                continue
            if any(joint_angle[i] < joint_angle[i+1] for i in [1, 4, 7, 10]):
                continue

            new_node = SpotNode(new_pos, new_rot, joint_angle=joint_angle)
            node.connect(direction, new_node)
            graph.add_node(new_node)
            queue.append(new_node)
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

    for (pos, rot), node in graph.node_map.items():
        obj = SpotNode(pos, rot, joint_angle=node.joint_angle, is_visited=int(node.is_visited))
        node_objs.append(obj)
        node_key_map[(tuple(pos), tuple(rot))] = obj

    key_to_id = await db.bulk_add_nodes(node_objs)
    return key_to_id

async def save_spot_graph_links(db, graph, key_to_id):
    """
    Save the links between nodes in the Spot graph to the database.
    """
    links = []
    for (pos, rot), node in graph.node_map.items():
        from_id = key_to_id[tuple(pos + rot)]
        for direction, neighbor in node.neighbors.items():
            if neighbor:
                to_id = key_to_id[tuple(neighbor.base_position + neighbor.base_rotation)]
                links.append((from_id, to_id, direction))

    await db.bulk_update_direction_links(links)
    print(f"Inserted {len(links)} links into DB.")

async def main():
    """
    Main function to generate the Spot graph and save it to the database.
    """
    db = AsyncSpotGraphDB("postgresql+asyncpg://myuser:mypassword@localhost:5432/mydatabase")
    await db.create_tables()

    graph = await generate_spot_graph()
    key_to_id = await save_spot_graph_nodes(db, graph)
    await save_spot_graph_links(db, graph, key_to_id)

    await db.close()

if __name__ == "__main__":
    asyncio.run(main())
