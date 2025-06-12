"""
Utility function to query pose databases by foot positions.
# This code is part of the AI_pkg project and is used to interact 
# with the Spot robot's pose database.
"""
import json
from Spot.spot_pose_nodes import AsyncSpotNameDB

async def query_pose_dbs_by_feet_positions(
    db: AsyncSpotNameDB,
    feet_positions: list[list[float]]
) -> dict | None:
    """
    Utility function to query pose database names by foot positions.

    Args:
        db (AsyncSpotNameDB): The database access object.
        feet_positions (list[list[float]]): The four foot positions in 3D (list of 4 lists).

    Returns:
        dict | None: Dictionary containing stance and lift pose DBs if found; otherwise None.
    """
    return await db.get_pose_dbs_by_foot_positions(json.dumps(feet_positions))
