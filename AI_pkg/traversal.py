"""
This module provides functionality to find a route for the Spot robot using a graph database.
#         Find a route from the origin point to the target point.
"""
import json
import asyncio
import time
from collections import deque
from typing import List

import numpy as np

from Spot.spot_graph_db import AsyncSpotGraphDB
from Spot.spot_pose_nodes import AsyncSpotNameDB
from utils.gym_manger import GymManager
from utils.pose_query import query_pose_dbs_by_feet_positions

class SpotRouteFinder:
    """
    A class to find a route for the Spot robot using a graph database.
    """
    def __init__(self,
          db_url="postgresql+asyncpg://myuser:mypassword@localhost:5432/mydatabase",
          table_name="nodes_t"):
        self.db_url = db_url
        self.table_name = table_name
        self._config = {
            "unit": 0.05
        }
        self.db = None
        self.key_map = {}
        self.setup_done = False

    async def setup(self):
        """
        Setup the database connection and create tables if they do not exist.
        """
        start_time = time.time()
        print("Connecting to database...")
        self.db = AsyncSpotGraphDB(table_name=self.table_name, db_url=self.db_url)
        await self.db.create_tables()

        key_to_id = await self.db.get_all_node_keys()
        key_to_joint_angles = await self.db.get_all_node_joint_angles()
        self.key_map = {
            "forward": key_to_id,
            "reverse": {v: k for k, v in key_to_id.items()},
            "joint_angles": dict(key_to_joint_angles)
        }
        await self.db.preload_all_neighbors()
        print(f"Database connected in {time.time() - start_time:.2f} seconds")
        print(f"Total nodes: {len(self.key_map['forward'])}")
        print("Database setup complete.")

    def is_valid_pose(self, joint_angle):
        """
        Check if the joint angles are within valid limits.
        """
        if np.isnan(joint_angle).any():
            return False
        limits = [(0, 30), (1, 165), (3, 30), (4, 165), (6, 30), (7, 165), (9, 30), (10, 165)]
        return all(abs(joint_angle[i]) <= limit for i, limit in limits)

    def calculate_offset(self, origin, target):
        """
        Calculate the offset between two points in 3D space.
        """
        return {
            'pos': [round(target['pos'][i] - origin['pos'][i], 3) for i in range(3)],
            'rot': [round(target['rot'][i] - origin['rot'][i], 3) for i in range(3)]
        }

    def get_direction(self, diff):
        """
        Determine the direction of movement based on the difference in position and rotation.
        """
        direction = []
        dirs = ['front', 'back', 'left', 'right', 'up', 'down']
        for i in range(3):
            if diff['pos'][i] > 0:
                direction.append(dirs[i * 2])
            elif diff['pos'][i] < 0:
                direction.append(dirs[i * 2 + 1])

        rot_dirs = ['rx_plus', 'rx_minus', 'ry_plus', 'ry_minus', 'rz_plus', 'rz_minus']
        for i in range(3):
            if diff['rot'][i] > 0:
                direction.append(rot_dirs[i * 2])
            elif diff['rot'][i] < 0:
                direction.append(rot_dirs[i * 2 + 1])
        tilt_dirs = ['tilt_lf_rb_plus', 'tilt_lf_rb_minus', 'tilt_rf_lb_plus', 'tilt_rf_lb_minus']
        # return direction
        return rot_dirs + dirs + tilt_dirs

    def get_node_id(self, point):
        """
        Get the node ID from the database based on the position and rotation of the Spot robot.
        """
        print("get_node_id", point)
        unit = self._config["unit"]
        rounded_components = [
            round(round(x / unit) * unit, 3)
            for x in point["pos"] + point["rot"] + point["tilt"]
        ]

        key = tuple(rounded_components)
        return self.key_map["forward"].get(key)

    def calculate_offset_point(self, from_key, to_point):
        """
        Calculate the offset from a key to a target point.
        """
        return self.calculate_offset(
            {'pos': list(from_key[:3]), 'rot': list(from_key[3:])},
            to_point
        )

    async def traverse(self, origin_id, target_id, target_point):
        """
        Traverse the graph to find a path from origin_id to target_id.
        """
        reverse_map = self.key_map["reverse"]
        queue = deque([[[origin_id],
                    self.calculate_offset_point(reverse_map[origin_id],
                    target_point)]])
        visited = set()

        while queue:
            ids, offset = queue.popleft()
            if ids[-1] == target_id:
                return ids
            if len(ids) > 100 or ids[-1] in visited:
                continue
            visited.add(ids[-1])
            neighbors = await self.db.get_direction_neighbors(ids[-1])
            directions = self.get_direction(offset)

            for direc in directions:
                next_id = neighbors.get(direc)
                if next_id and next_id not in visited:
                    next_point = {'pos': reverse_map[next_id][:3], 'rot': reverse_map[next_id][3:]}
                    new_offset = self.calculate_offset(next_point, target_point)
                    queue.append([ids + [next_id], new_offset])

        return []

    async def find_route(self, origin_point, target_point):
        """
        Find a route from the origin point to the target point.
        """
        if not self.setup_done:
            await self.setup()
            self.setup_done = True
        origin_id = self.get_node_id(origin_point)
        target_id = self.get_node_id(target_point)
        print(f"Origin ID: {origin_id}, Target ID: {target_id}")

        if origin_id is None or target_id is None:
            print("Invalid origin or target point")
            await self.db.close()
            return []
        start_time = time.time()
        path = await self.traverse(origin_id, target_id, target_point)
        end_time = time.time()
        print(f"Traversal time: {end_time - start_time:.5f} seconds")
        await self.db.close()
        return path

def interpolate_angles(start, end, steps=10):
    """
    Interpolate between two sets of angles.
    """
    start = np.array(start)
    end = np.array(end)
      # exclude start & end
    return [((1 - t) * start + t * end).tolist() for t in np.linspace(0, 1, steps + 2)[1:-1]]

def ask_db_ip() -> str:
    """
    Ask the user for the database URL.
    """
    if input("Use cloud database? [y/N]: ").strip().lower() == "y":
        db_ip = input("SQL IP: ").strip()
        return db_ip
    return "localhost"

def play_spot_route(path_ids: List[int], finder,
                    steps_per_segment: int = 10, dt: float = 0.01) -> None:
    """
    Play the Spot route using the AI dog node.
    """
    rev_map = finder.key_map["joint_angles"]

    dog_node, ros_thread = GymManager().init_ai_dog_node()
    try:
        # 逐段處理
        for cur_id, nxt_id in zip(path_ids, path_ids[1:]):
            # 取出兩端點的（pos, rot）
            start_angles, end_angles = rev_map[cur_id], rev_map[nxt_id]

            # 發送插值
            for ang in interpolate_angles(start_angles, end_angles, steps_per_segment):
                dog_node.publish_spot_actions(ang)
                time.sleep(dt)

            # 再發送一次終點角度，確保到位
            dog_node.publish_spot_actions(end_angles)
            time.sleep(dt)
    finally:
        GymManager().shutdown_ai_dog_node(dog_node, ros_thread)
        print("[INFO] AI_dog_node stopped.")


async def traversal_at_stance(
            waypoints: List[dict] = None,
            table_name: str = "nodes_t",
            db_ip: str = "localhost") -> None:
    """
    Main function to run the Spot route finder.
    """
    # Initialize SpotRouteFinder
    db_url = f"postgresql+asyncpg://myuser:mypassword@{db_ip}:5432/mydatabase"
    finder = SpotRouteFinder(db_url, table_name=table_name["stance"])

    full_path: list[int] = []

    # (start, goal) 兩兩配對
    for start, goal in zip(waypoints, waypoints[1:]):
        print(f"Finding route from {start} to {goal}...")
        segment = await finder.find_route(start, goal)
        print(f"Segment from {start} to {goal}: {segment}")
        if not segment:
            raise RuntimeError(f"找不到 {start} → {goal} 的路徑")

        # 第一段保留起點，其餘段落去掉「段首」避免重複
        full_path.extend(segment if not full_path else segment[1:])

    print("Combined path:", full_path)
    print("Total nodes :", len(full_path))
    input("Press Enter to start the dog movement...")
    # Execute the dog movement
    if full_path:
        play_spot_route(full_path, finder)
    else:
        print("No path found.")



if __name__ == "__main__":
    # Ask for database URL
    ip = ask_db_ip()
    foot_positions = [
        [0.0, 0.0, 0.05],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0]
    ]
    name_db = AsyncSpotNameDB(
        table_name="spot_nodes_name",
        db_url=f"postgresql+asyncpg://myuser:mypassword@{ip}:5432/db_name"
    )
    stance_pose_db = asyncio.run(query_pose_dbs_by_feet_positions(
        name_db,
        foot_positions,
    ))
    print(f"Stance pose DB: {stance_pose_db}")

    traversal_points = [
        {"pos": [0.00, 0.00, 0.20], "rot": [0, 0, 0], "tilt": [0, 0]},       # 起點
        {"pos": [0.05, 0.03, 0.20], "rot": [0, 0, 0], "tilt": [0, 0]},      # 第一目標
        {"pos": [0.05, -0.03, 0.20], "rot": [0, 0, 0], "tilt": [0, 0]},     # 第二目標
        {"pos": [-0.05, -0.03, 0.20], "rot": [0, 0, 0], "tilt": [0, 0]},     # 第三目標
        {"pos": [-0.05, 0.03, 0.20], "rot": [0, 0, 0], "tilt": [0, 0]},     # 第四目標
        {"pos": [0.00, 0.00, 0.20], "rot": [0, 0, 0], "tilt": [0, 0]},     # 終點
    ]
    asyncio.run(traversal_at_stance(traversal_points,
                                     table_name=stance_pose_db,
                                     db_ip=ip))
