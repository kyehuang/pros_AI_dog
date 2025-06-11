import asyncio
import hashlib
import json

from Spot.spot_pose_nodes import AsyncSpotNameDB, SpotNodePose
from Spot.spot_graph_db import AsyncSpotGraphDB
from utils.spot_graph import generate_spot_graph, save_spot_graph_nodes, save_spot_graph_links

def generate_pose_db_name(prefix: str, foot_positions: list[list[float]]) -> str:
    """
    Base on the provided foot positions, generate a unique pose database name.
    The name is created by hashing the foot positions and appending it to the prefix.
    :param prefix: the prefix for the database name.
    :param foot_positions: a list of foot positions, each position is a list of floats.
    :return: a unique pose database name.
    """
    pos_str = json.dumps(foot_positions, separators=(',', ':'))
    pos_hash = hashlib.md5(pos_str.encode()).hexdigest()[:8]
    return f"{prefix}_{pos_hash}"

def load_foot_positions_map(json_path: str = "foot_positions_map.json") -> dict:
    """
    Load the foot positions map from a JSON file.
    :param json_path: path to the JSON file containing foot positions map.
    :return: a dictionary mapping foot positions to their corresponding patterns.
    """
    with open(json_path, "r", encoding="utf-8") as f:
        return json.load(f)

async def link_foot_positions_to_db_name(
            db: AsyncSpotNameDB = None,
            map_path = "foot_positions_map.json"):
    """
    Link foot positions to database names in the SpotNodePose table.
    :param db: an instance of AsyncSpotNameDB to interact with the database.
    :param map_path: path to the JSON file containing foot positions map.
    :return: a list of SpotNodePose instances with linked database names.
    """
    foot_positions_map = load_foot_positions_map(map_path)

    node_poses = []
    for foot_positions, pattern in foot_positions_map.items():
        # Generate a unique pose database name
        stance_pose_db = generate_pose_db_name("stance_pose_db", json.loads(foot_positions))
        lift_pose_db = generate_pose_db_name(f"{pattern}_lift_pose_db", json.loads(foot_positions))

        # Create a SpotNodePose instance with the generated name
        node_pose = SpotNodePose(
            foot_positions=foot_positions,
            stance_pose_db=stance_pose_db,
            lf_lift_pose_db=lift_pose_db if pattern == "lf" else None,
            rf_lift_pose_db=lift_pose_db if pattern == "rf" else None,
            rb_lift_pose_db=lift_pose_db if pattern == "rb" else None,
            lb_lift_pose_db=lift_pose_db if pattern == "lb" else None
        )
        # Add the node to the database
        await db.add_node(node_pose)
        node_poses.append(node_pose)

    print("Foot positions linked to database names successfully.")
    return node_poses


async def collect_states(node_poses: list[SpotNodePose]):
    """
    Collect the state of the Spot robot.
    This function is a placeholder for future implementation.
    """
    # Placeholder for future implementation
    print("Collecting state of the Spot robot...")
    for node_pose in node_poses:
        await asyncio.sleep(1)  # Simulate some processing time

        # Calculate the stance pose based on the foot positions
        await calculate_stance_pose(
            foot_positions=json.loads(node_pose.foot_positions),
            table_name=node_pose.stance_pose_db
        )

async def calculate_stance_pose(
            foot_positions: list[list[float]],
            table_name) -> list[float]:
    """
    Calculate the stance pose for the given foot positions and create a database table.
    :param foot_positions: a list of foot positions, each position is a list of floats.
    :param table_name: the name of the database table to create.
    """
    stance_db = AsyncSpotGraphDB(
        table_name=table_name,
        db_url="postgresql+asyncpg://myuser:mypassword@localhost:5432/mydatabase"
    )
    await stance_db.create_tables()

    key_mapping_stance = {
        "up": [0, 0, 0.05, 0, 0, 0, 0, 0], "down": [0, 0, -0.05, 0, 0, 0, 0, 0],
        "right": [0, -0.05, 0, 0, 0, 0, 0, 0], "left": [0, 0.05, 0, 0, 0, 0, 0, 0],
        "front": [0.05, 0, 0, 0, 0, 0, 0, 0], "back": [-0.05, 0, 0, 0, 0, 0, 0, 0],
        "rx_plus": [0, 0, 0, 5, 0, 0, 0, 0], "rx_minus": [0, 0, 0, -5, 0, 0, 0, 0],
        "ry_plus": [0, 0, 0, 0, 5, 0, 0, 0], "ry_minus": [0, 0, 0, 0, -5, 0, 0, 0],
        "rz_plus": [0, 0, 0, 0, 0, 5, 0, 0], "rz_minus": [0, 0, 0, 0, 0, -5, 0, 0],
        "tilt_lf_rb_plus": [0, 0, 0, 0, 0, 0, 0.1, 0],
        "tilt_lf_rb_minus": [0, 0, 0, 0, 0, 0, -0.1, 0],
        "tilt_rf_lb_plus": [0, 0, 0, 0, 0, 0, 0, 0.1],
        "tilt_rf_lb_minus": [0, 0, 0, 0, 0, 0, 0, -0.1],
    }

    graph = await generate_spot_graph(max_level=5,
                                      multiple=1,
                                      key_mapping=key_mapping_stance,
                                      leg_end_position=foot_positions)
    key_to_id = await save_spot_graph_nodes(stance_db, graph)
    await save_spot_graph_links(stance_db, graph, key_to_id)

    print(f"Stance pose table '{table_name}' created successfully.")
    await stance_db.close()

async def main():
    """
    main function
    """
    foot_mapping_db = AsyncSpotNameDB(
        table_name="spot_nodes_name",
        db_url="postgresql+asyncpg://myuser:mypassword@localhost:5432/db_name"
    )
    await foot_mapping_db.create_tables()

    node_poses = await link_foot_positions_to_db_name(foot_mapping_db)
    await collect_states(node_poses)

if __name__ == "__main__":
    asyncio.run(main())
    print("Foot positions linked to database names successfully.")
