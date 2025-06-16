"""
This module provides functionality to find a route for the Spot robot using a graph database.
#         Find a route from the origin point to the target point.
"""
import asyncio
import time
import threading
from collections import deque
from typing import List
import json
from dataclasses import dataclass, field

import numpy as np

from Spot.spot_graph_db import AsyncSpotGraphDB
from Spot.spot_pose_nodes import AsyncSpotNameDB
from Spot.spot_lift_action import AsyncSpotLiftActionDB
from ros_receive_and_processing import ai_dog_node
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
            "unit": 0.01
        }
        self.db = None
        self.key_map = {
            "forward": None,
            "reverse": None,
            "joint_angles": None
        }
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


@dataclass
class SpotRouteConfig:
    """
    Configuration for the Spot route executor.
    This class holds the configuration parameters for the Spot route executor.
    """
    table_name: str = "nodes_t"
    db_ip: str = "localhost"
    waypoints: List[dict] = None
    step_per_time: float = 0.01
    steps_per_segment: int = 10
    dog_node: ai_dog_node = None
    ros_thread: threading.Thread = None

class SpotRouteExecutor:
    """
    A class to execute a route for the Spot robot using waypoints.
    This class computes the path based on provided waypoints and executes it.
    """
    def __init__(
            self,
            config: SpotRouteConfig = SpotRouteConfig()
        ):
        self.config = config
        self.db_url = f"postgresql+asyncpg://myuser:mypassword@{config.db_ip}:5432/mydatabase"
        self.finder = SpotRouteFinder(self.db_url, table_name=config.table_name)
        self.waypoints = config.waypoints if config.waypoints else []
        self.step_per_time = config.step_per_time
        self.steps_per_segment = config.steps_per_segment
        self.full_path = []

        if config.dog_node is None:
            config.dog_node, config.ros_thread = GymManager().init_ai_dog_node()

    async def compute_path(self) -> List[int]:
        """
        Compute the full path IDs from the waypoints.
        """
        if self.full_path:
            print("Path already computed, returning cached path.")
            return self.full_path

        # Check if waypoints are provided
        if not self.waypoints:
            raise ValueError("No waypoints provided for path computation.")
        if not self.finder.setup_done:
            await self.finder.setup()
            self.finder.setup_done = True
        if len(self.waypoints) < 2:
            raise ValueError("At least two waypoints are required to compute a path.")

        full_path: list[int] = []

        for start, goal in zip(self.waypoints, self.waypoints[1:]):
            # print(f"Finding route from {start} to {goal}...")
            segment = await self.finder.find_route(start, goal)
            # print(f"Segment from {start} to {goal}: {segment}")
            if not segment:
                raise RuntimeError(f"找不到 {start} → {goal} 的路徑")

            # 第一段保留起點，其餘段落去掉「段首」避免重複
            full_path.extend(segment if not full_path else segment[1:])

        self.full_path = full_path

        return full_path

    async def execute_route(self) -> None:
        """
        Execute the route by playing the Spot route.
        """
        if not self.full_path:
            await self.compute_path()

        if self.full_path:
            self.play_spot_route(
                path_ids=self.full_path,
                steps_per_segment=self.steps_per_segment,
                step_per_time=self.step_per_time
            )
        else:
            print("No path found.")

    async def run(self):
        """
        Run the Spot route executor.
        """
        await self.execute_route()
        return self.full_path

    async def clear_path(self) -> None:
        """
        Clear the computed path.
        """
        self.full_path = []
        print("Path cleared.")

    async def run_start_to_end(self) -> None:
        """
        Run the Spot route executor from the start to the end of the computed path.
        """
        if not self.full_path:
            await self.compute_path()

        # Example of running with the first two nodes
        start_id, end_id = self.full_path[0], self.full_path[-1]
        print(f"Running with nodes: {start_id} → {end_id}")
        self.play_spot_route([start_id, end_id],
                             steps_per_segment=self.steps_per_segment,
                             step_per_time=self.step_per_time)

    def play_spot_route(
            self,
            path_ids,
            steps_per_segment: int = 10,
            step_per_time: float = 0.01
        ) -> None:
        """
        Play the Spot route using the AI dog node.
        :param path_ids: List of node IDs representing the route.
        :param steps_per_segment: Number of steps per segment for interpolation.
        :param step_per_time: Delay between steps in seconds.
        """

        for cur_id, nxt_id in zip(path_ids, path_ids[1:]):
            # Get the joint angles for the current and next node
            start_angles, end_angles = self.finder.key_map["joint_angles"][cur_id], \
                                    self.finder.key_map["joint_angles"][nxt_id]

            self.config.dog_node.send_joint_angle_trajectory(
                start_action=start_angles,
                target_action=end_angles,
                step=steps_per_segment,
                delay=step_per_time
            )

@dataclass
class SpotLiftConfig:
    """
    Configuration for the Spot lift executor.
    This class holds the configuration parameters for the Spot lift executor.
    """
    column_name: str = "stance_pose_db_be55ce64_LF_[0.0, 0.0, 0.2]_distance_0.05"
    length: int = 4
    step_per_times: List[float] = field(default_factory=lambda: [0.03, 0.03, 0.03, 0.03])
    steps_per_segments: List[int] = field(default_factory=lambda: [10, 10, 10, 10])

class SpotLiftExecutor:
    """
    A class to execute the Spot lift route.
    This class is a placeholder for future implementation of lift route execution.
    """
    def __init__(self,
                 db_ip: str = "localhost",
                 table_name: str = "spot_lift_action",
                 dog_node: ai_dog_node = None,
                 ros_thread: threading.Thread = None
        ):
        self.db_ip = db_ip
        self.table_name = table_name
        self.spot_lift_action = AsyncSpotLiftActionDB(
            db_url=f"postgresql+asyncpg://myuser:mypassword@{self.db_ip}:5432/db_actions",
            table_name=self.table_name
        )
        self.points_mapping = {}

        if dog_node is None:
            dog_node, ros_thread = GymManager().init_ai_dog_node()
        self.dog_node = dog_node
        self.ros_thread = ros_thread

    async def get_joint_angles(self, lift_config: SpotLiftConfig) -> List[int]:
        """
        Get joint angles for a specific action from the Spot lift action database.
        """
        print(f"Running Spot lift executor with config: {lift_config}")
        print(f"points_mapping: {self.points_mapping}")


        print(f"length: {lift_config.length}, column_name: {lift_config.column_name}")
        points = []
        for i in range(lift_config.length):
            # Simulate fetching joint angles from the database
            print("Fetching joint angles from the database...")
            joint_angles = await self.spot_lift_action.get_action_joint_angles(
                lift_config.column_name, i+1)
            if not joint_angles:
                print(f"No joint angles found for {lift_config.column_name} at index {i+1}")
                continue
            points.append({
                "joint_angles": json.loads(joint_angles["joint_angle"]),
                "step_per_time": lift_config.step_per_times[i],
                "steps_per_segment": lift_config.steps_per_segments[i]
            })
        self.points_mapping[lift_config.column_name] = points

    async def run(self, lift_config: SpotLiftConfig) -> None:
        """
        Run the Spot lift executor.
        """
        await self.execute_route(lift_config)
        return self.points_mapping.get(lift_config.column_name, [])

    async def execute_route(self, lift_config: SpotLiftConfig) -> None:
        """
        Execute the route by playing the Spot route.
        """
        if lift_config.column_name not in self.points_mapping:
            await self.get_joint_angles(lift_config)

        if lift_config.column_name in self.points_mapping:
            data = self.points_mapping[lift_config.column_name]
            for start, end in zip(data, data[1:]):
                self.dog_node.send_joint_angle_trajectory(
                    start_action=start['joint_angles'],
                    target_action=end['joint_angles'],
                    step=start['steps_per_segment'],
                    delay=start['step_per_time']
                )
                print(f"Executed lift action: {lift_config.column_name}, "
                        f"step: {start['steps_per_segment']}, "
                        f"delay: {start['step_per_time']}")

    def close(self):
        """
        Close the Spot lift action database connection.
        """
        GymManager.shutdown_ai_dog_node(self.dog_node, self.ros_thread)
        print("Spot lift action database connection closed.")

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

    # Initialize the SpotRouteExecutor with the stance pose DB and traversal points
    route_config = SpotRouteConfig(
        table_name=stance_pose_db["stance"],
        db_ip=ip,
        waypoints=traversal_points,
        step_per_time=0.03,
        steps_per_segment=10
    )

    route_executor = SpotRouteExecutor(
        config=route_config
    )

    asyncio.run(route_executor.run())
