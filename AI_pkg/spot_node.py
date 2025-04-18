import json
from collections import deque
import time
import numpy as np

from IK.spot_state import spot_state_creater
from IK.DH import get_foot_position
from IK.spot_leg import SpotLeg
from Spot.spot_graph_db import SpotGraphDB, SpotNode

key_mapping = {
    "up": [0, 0, 0.01, 0, 0, 0],
    "down": [0, 0, -0.01, 0, 0, 0], 
    "right": [-0.01, 0, 0, 0, 0, 0],
    "left": [0.01, 0, 0, 0, 0, 0],
    "front": [0.01, 0, 0, 0, 0, 0],
    "back": [-0.01, 0, 0, 0, 0, 0],
    "rx_plus": [0, 0, 0, 1, 0, 0],
    "rx_minus": [0, 0, 0, -1, 0, 0],
    "ry_plus": [0, 0, 0, 0, 1, 0],
    "ry_minus": [0, 0, 0, 0, -1, 0],
    "rz_plus": [0, 0, 0, 0, 0, 1],
    "rz_minus": [0, 0, 0, 0, 0, -1],
}

def pose_key(pos, rot):
    return (tuple(pos), tuple(rot))

def is_valid_pose(joint_angle):
    if np.isnan(joint_angle).any():
        return False
    limits = [
        (0, 30), (1, 160),
        (3, 30), (4, 160),
        (6, 30), (7, 160),
        (9, 30), (10, 160)
    ]
    for i, limit in limits:
        if abs(joint_angle[i]) > limit:
            return False
    return True

def spot():
    joint_lengths = [0.0801, 0.1501, 0.1451]
    base_translation = [0.3740, 0.1670, 0]
    spot_leg = SpotLeg(joint_lengths, [0, 0, 0])

    root_node = SpotNode([0, 0, 0.2], [0, 0, 0])
    root_node.joint_angle = spot_state_creater(spot_leg, root_node.base_position, root_node.base_rotation, base_translation)
    root_node.is_leaf = False

    graph = SpotGraphDB()
    root_id = graph.add_node(root_node)

    queue = deque([root_id])
    visited_nodes = set([pose_key(root_node.base_position, root_node.base_rotation)])

    start_time = time.time()
    processed_nodes = 0
    time_step = 0
    x = 2

    while queue:
        node_id = queue.popleft()
        if node_id is None:
            continue

        node = graph.get_node_position_by_id(node_id)
        for direction, offset in key_mapping.items():
            new_position = [round(node["position"][i] + x * offset[i], 3) for i in range(3)]
            new_rotation = [round(node["rotation"][i] + x * offset[i+3], 3) for i in range(3)]

            if new_position[2] < 0.05:
                continue
            if any(abs(angle) > limit for angle, limit in zip(new_rotation, [30, 30, 15])):
                continue

            key = pose_key(new_position, new_rotation)
            if key in visited_nodes:
                continue

            joint_angle = spot_state_creater(spot_leg, new_position, new_rotation, base_translation)
            if not is_valid_pose(joint_angle):
                continue

            new_node = SpotNode(new_position, new_rotation)
            new_node.joint_angle = joint_angle
            new_node_id = graph.add_node(new_node)
            graph.update_direction_link(node_id, new_node_id, direction)
            queue.append(new_node_id)
            visited_nodes.add(key)
            processed_nodes += 1

        if time_step % 10000 == 0:
            print("\ntime", time_step, node)
            print(get_foot_position(node["joint_angle"], joint_lengths, base_translation))

        if time_step % 1000 == 0:
            elapsed = time.time() - start_time
            print(f"Time step: {time_step}, Processed: {processed_nodes}, Speed: {processed_nodes / elapsed:.2f} nodes/sec")

        time_step += 1

    end_time = time.time()
    print("Execution time:", end_time - start_time)
    print("Total processed nodes:", processed_nodes)

if __name__ == "__main__":
    spot()
