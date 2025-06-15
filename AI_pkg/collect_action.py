"""
Collect foot positions and link them to database names for Spot robot.
This script generates unique database names based on foot positions,
calculates stance and lift poses, and saves the results in a PostgreSQL database.
"""
import asyncio
import hashlib
import json

from Spot.spot_lift_action import AsyncSpotLiftActionDB, SpotLiftAction

from IK.spot_leg import SpotLeg
from IK.spot_state import spot_state_creater

JOINT_LENGTHS = [0.0801, 0.1501, 0.1451]  # lengths of the Spot robot's joints in meters
BASE_TRANSLATION = [0.3740, 0.1670, 0]
SPOT_LEG = SpotLeg(JOINT_LENGTHS, [0, 0, 0])

def load_lift_action_map(json_path: str = "lift_action_map.json") -> dict:
    """
    Load the lift action map from a JSON file.
    :param json_path: path to the JSON file containing lift action map.
    :return: a dictionary mapping lift actions to their corresponding patterns.
    """
    with open(json_path, "r", encoding="utf-8") as f:
        return json.load(f)

def generate_lift_action_name(action: dict) -> str:
    """
    Generate a unique name for a lift action based on its properties.
    :param action: a dictionary containing the properties of the lift action.
    :return: a unique name for the lift action.
    """
    state = action["state"]
    start_pos = action["base_position"]
    distance = action["distance"]
    foot_positions = action["foot_positions"]

    pos_str = json.dumps(foot_positions, separators=(',', ':'))
    pos_hash = hashlib.md5(pos_str.encode()).hexdigest()[:8]
    return f"stance_pose_db_{pos_hash}_{state}_{start_pos}_distance_{distance}"

async def collect_lift_actions(
        db: AsyncSpotLiftActionDB,
        lift_actions: list[dict]
        ) -> None:
    """
    Collect lift actions and save them to the database.
    :param db: an instance of AsyncSpotLiftActionDB to interact with the database.
    :param lift_action_map: a dictionary mapping lift actions to their corresponding patterns.
    :return: a list of lift actions collected from the map.
    """
    for action_cfg in lift_actions:

        base_pos = action_cfg['base_position']
        foot_seq = action_cfg['foot_position_sequence']
        action_name = generate_lift_action_name(action_cfg)

        for step_idx, foot_positions in enumerate(foot_seq, start=1):
            joint_angle = spot_state_creater(
                SPOT_LEG,
                base_pos,
                [0, 0, 0],
                BASE_TRANSLATION,
                [0, 0],
                foot_positions
            )

            # Save the lift action to the database
            action_record = SpotLiftAction(
                name=action_name,
                step_index=step_idx,  # Step index starts from 1
                joint_angle=joint_angle  # Store joint angles as a JSON string
            )

            await db.add_action(action_record)
            print(f"Added lift action: {action_name}, step: {step_idx}, joint angles: {joint_angle}")

async def main():
    """
    main function
    """
    lift_action_db = AsyncSpotLiftActionDB(
        table_name="spot_lift_action",
        db_url="postgresql+asyncpg://myuser:mypassword@localhost/db_actions"
    )
    await lift_action_db.create_tables()

    lift_actions = load_lift_action_map("lift_action.json")

    await collect_lift_actions(lift_action_db, lift_actions)

if __name__ == "__main__":
    asyncio.run(main())
