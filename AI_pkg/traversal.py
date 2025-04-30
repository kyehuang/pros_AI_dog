import json
import time
import numpy as np
import asyncio
from collections import deque

from IK.spot_state import spot_state_creater
from IK.DH import get_foot_position
from IK.spot_leg import SpotLeg
from Spot.spot_graph_db import AsyncSpotGraphDB
from ros_receive_and_processing import ai_dog_node
from utils.gym_manger import GymManager

class SpotRouteFinder:
    """
    A class to find a route for the Spot robot using a graph database.
    """
    def __init__(self, db_url="postgresql+asyncpg://myuser:mypassword@localhost:5432/mydatabase"):
        self.db_url = db_url
        self._config = {
            "joint_lengths": [0.0801, 0.1501, 0.1451],
            "base_translation": [0.3740, 0.1670, 0],
            "unit": 0.05
        }
        self.db = None
        self.spot_leg = None
        self.key_map = {}

    async def setup(self):
        """
        Setup the database connection and create tables if they do not exist.
        """
        self.db = AsyncSpotGraphDB(self.db_url)
        await self.db.create_tables()
        self.spot_leg = SpotLeg(self._config["joint_lengths"], [0, 0, 0])
        key_to_id = await self.db.get_all_node_keys()
        self.key_map = {
            "forward": key_to_id,
            "reverse": {v: k for k, v in key_to_id.items()}
        }
        await self.db.preload_all_neighbors()

    def is_valid_pose(self, joint_angle):
        """
        Check if the joint angles are within valid limits.
        """
        if np.isnan(joint_angle).any():
            return False
        limits = [(0, 30), (1, 150), (3, 30), (4, 150), (6, 30), (7, 150), (9, 30), (10, 150)]
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

        return direction

    def get_node_id(self, point):
        """
        Get the node ID from the database based on the position and rotation of the Spot robot.
        """
        joint_angle = spot_state_creater(self.spot_leg,
                        point['pos'], point['rot'],
                        self._config["base_translation"])
        if not self.is_valid_pose(joint_angle):
            print("Invalid joint angle")
            return None
        unit = self._config["unit"]
        key = tuple(round(round(x / unit) * unit, 3) for x in point['pos'] + point['rot'])
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
        await self.setup()
        origin_id = self.get_node_id(origin_point)
        target_id = self.get_node_id(target_point)

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
    return [((1 - t) * start + t * end).tolist() for t in np.linspace(0, 1, steps + 2)[1:-1]]  # exclude start & end

async def main():
    """
    Main function to run the Spot route finder.
    """
    origin_node = {'pos': [0.05, 0.05, 0.2], 'rot': [15, 5, 5]}
    target_node = {'pos': [0, 0, 0.2], 'rot': [0, 0, 0]}

    target_node = {'pos': [0.05, 0.05, 0.2], 'rot': [15, 5, 5]}
    origin_node = {'pos': [0, 0, 0.2], 'rot': [0, 0, 0]}

    mode = input("If you want to use the database in the cloud [y/N]: ")
    if mode == 'y' or mode == 'Y':
        sql_ip = input("Please input sql ip: ")
        db_url = "postgresql+asyncpg://myuser:mypassword@{}:5432/mydatabase".format(sql_ip)
        finder = SpotRouteFinder(db_url)
    else:
        db_url = "postgresql+asyncpg://myuser:mypassword@localhost:5432/mydatabase"
        finder = SpotRouteFinder(db_url)

    paths = await finder.find_route(origin_node, target_node)
    print("Traversal path:", paths)
    print("Path length:", len(paths))

    # Initialize AI_dog_node
    dog_node, ros_thread = GymManager().init_ai_dog_node()

    for i in range(len(paths) - 1):
        # 取得當前與下一個角度
        start_node = {"pos": finder.key_map["reverse"][paths[i]][:3],
                      "rot": finder.key_map["reverse"][paths[i]][3:]}
        end_node = {"pos": finder.key_map["reverse"][paths[i + 1]][:3],
                     "rot": finder.key_map["reverse"][paths[i + 1]][3:]}

        start_angles = spot_state_creater(finder.spot_leg,
                        start_node['pos'], start_node['rot'],
                        finder._config["base_translation"])
        end_angles = spot_state_creater(finder.spot_leg,
                        end_node['pos'], end_node['rot'],
                        finder._config["base_translation"])

        # 插值中間的角度
        interpolated_steps = interpolate_angles(start_angles, end_angles, steps=10)

        # 發送出每一段動作
        for angles in interpolated_steps:
            dog_node.publish_spot_actions(angles)
            time.sleep(0.05)

        # 最後也要執行一次 end_angles，確保到達終點
        dog_node.publish_spot_actions(end_angles)
        time.sleep(0.05)

    # Shutdown AI_dog_node
    GymManager().shutdown_ai_dog_node(dog_node, ros_thread)
    print("[INFO] AI_dog_node has stopped.")

if __name__ == "__main__":
    asyncio.run(main())
