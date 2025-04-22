
import json
from collections import deque
import time
import numpy as np

from IK.spot_state import spot_state_creater
from IK.DH import get_foot_position
from IK.spot_leg import SpotLeg
from Spot.spot_graph_db import SpotGraphDB, SpotNode
import time

key_mapping = {
    "up":       [    0,     0,  0.04,  0, 0, 0],
    "down":     [    0,     0, -0.04,  0, 0, 0], 
    "right":    [    0, -0.04,     0,  0, 0, 0],
    "left":     [    0,  0.04,     0,  0, 0, 0],
    "front":    [ 0.04,     0,     0,  0, 0, 0],
    "back":     [-0.04,     0,     0,  0, 0, 0],
    "rx_plus":  [    0,     0,     0,  4, 0, 0],
    "rx_minus": [    0,     0,     0, -4, 0, 0],
    "ry_plus":  [    0,     0,     0,  0, 4, 0],
    "ry_minus": [    0,     0,     0,  0, -4, 0],
    "rz_plus":  [    0,     0,     0,  0, 0, 4],
    "rz_minus": [    0,     0,     0,  0, 0, -4],
}

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
    root_node.joint_angle = spot_state_creater(
        spot_leg, root_node.base_position,
        root_node.base_rotation, base_translation)
    root_node.is_leaf = False


    graph = SpotGraphDB("test.db")
    root_node_id = graph.add_node(root_node)

    queue = deque([[root_node, root_node_id]])
    visited_node = set()

    max_nodes = 100000
    start_time = time.time()
    processed_nodes = 0
    time_step = 0
    mult = 1
    while queue:
        node, node_id = queue.pop()
        if node_id >= max_nodes:
            break
        if node_id in visited_node:
            continue
        visited_node.add(node_id)

        next_nodes = []
        for direction, offset in key_mapping.items():
            new_position = [round(node.base_position[i] + offset[i] * mult, 4) for i in range(3)]
            new_rotation = [round(node.base_rotation[i] + offset[i + 3] * mult, 4) for i in range(3)]
            new_node = SpotNode(new_position, new_rotation)

            if new_position[2] < 0.05:
                continue
            if any(abs(angle) > limit for angle, limit in zip(new_rotation, [30, 30, 15])):
                continue
            new_node.joint_angle = spot_state_creater(
                spot_leg, new_node.base_position,
                new_node.base_rotation, base_translation)
            if not is_valid_pose(new_node.joint_angle):
                continue
            next_nodes.append([new_node, direction])

        direction_map = {}
        for next_node, direction in next_nodes:
            if len(visited_node) >= max_nodes:
                break
            next_node_id = graph.add_node(next_node)
            direction_map[direction] = next_node_id
            # visited_node.add(next_node_id)
            processed_nodes += 1
            queue.append([next_node, next_node_id])

        graph.update_direction_links(node_id, direction_map)

        if time_step % 1000 == 0:
            elapsed = time.time() - start_time
            print(f"Time step: {time_step}, Processed: {processed_nodes}, Speed: {processed_nodes / elapsed:.2f} nodes/sec")
        time_step += 1
        
if __name__ == "__main__":
    iterations = 1
    times = []
    for _ in range(iterations):
        start_time = time.time()
        spot()
        end_time = time.time()
        times.append(end_time - start_time)
        print(f"Iteration took {end_time - start_time:.4f} seconds")
    avg_time = sum(times) / iterations
    print(f"Average time per iteration: {avg_time:.4f} seconds")