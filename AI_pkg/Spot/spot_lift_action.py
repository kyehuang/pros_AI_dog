"""
Asynchronous Database Handler for Spot Robot Lift Actions
#     This class provides methods to interact with the SpotNodePose table in a PostgreSQL database.
#     - create_tables: Create the SpotNodePose table in the database.
#     - add_action: Add a new lift action to the table, skipping if the action already exists.
"""
import asyncio
import dataclasses
import json

from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy import Column, Integer, Text, UniqueConstraint, select
from sqlalchemy.orm import declarative_base, sessionmaker
from sqlalchemy.dialects.postgresql import insert

Base = declarative_base()

@dataclasses.dataclass
class SpotLiftAction(Base):
    """
    Spot robot lift action data model.
    This class represents the lift action data of a Spot robot's leg in a database table.
    It includes the name of the action, the step index, and the joint angle.
    Attributes:
    - id: Primary key, auto-incremented integer ID.
    - name: A string representing the name of the lift action.
    - step_index: An integer representing the index of the step in the lift action sequence.
    - joint_angle: A string representing the joint angles for the lift action
        (suggested format: JSON string).
    """
    __abstract__ = True

    id: int = Column(Integer, primary_key=True, autoincrement=True)
    name: str = Column(Text, nullable=False)
    step_index: int = Column(Integer, nullable=False)

    joint_angle: str = Column(Text)

def create_lift_action_table_class(table_name: str):
    """
    According to the provided table_name, dynamically create a concrete subclass of SpotLiftAction.
    This allows for creating multiple tables with the same structure but different names.
    Args:
        table_name (str): The name of the table to be created.
    Returns:
        type: A new class that inherits from SpotLiftAction with the specified table name.
    """
    return type(
        table_name,
        (SpotLiftAction,),
        {
            '__tablename__': table_name,
            '__table_args__': (
                UniqueConstraint(
                    'name', 'step_index',
                    name=f'unique_{table_name}'
                ),
            ),
        }
    )

class AsyncSpotLiftActionDB:
    """
    Asynchronous database handler for Spot robot lift actions.
    This class provides methods to interact with the SpotLiftAction table in a PostgreSQL database.
    It supports adding, updating, and querying lift actions.
    """
    def __init__(
        self,
        table_name: str = "spot_lift_action",
        db_url: str = "postgresql+asyncpg://user:password@localhost/db_actions"
        ):
        self.engine = create_async_engine(db_url, echo=False)
        self.asnc_session = sessionmaker(
            self.engine,
            expire_on_commit=False,
            class_=AsyncSession
        )
        self.table_class = create_lift_action_table_class(table_name)

    async def create_tables(self):
        """
        Create the SpotLiftAction table in the database.
        This method creates the table if it does not already exist.
        """
        async with self.engine.begin() as conn:
            await conn.run_sync(self.table_class.metadata.create_all)

    async def add_action(self, action: SpotLiftAction):
        """
        Add a new lift action to the table.
        If an action with the same name and step index already exists, it will be skipped.
        Args:
            action (SpotLiftAction): The lift action to be added.
        """
        async with self.asnc_session() as session:
            async with session.begin():
                stmt = insert(self.table_class).values(
                    name=action.name,
                    step_index=action.step_index,
                    joint_angle=json.dumps(action.joint_angle)
                ).on_conflict_do_nothing(
                    index_elements=['name', 'step_index']
                )
                await session.execute(stmt)

    async def update_action(self, action: SpotLiftAction):
        """
        Update an existing lift action in the table.
        If the action does not exist, it will be added.
        Args:
            action (SpotLiftAction): The lift action to be updated.
        """
        async with self.asnc_session() as session:
            async with session.begin():
                stmt = (
                    self.table_class.__table__.update()
                    .where(
                        self.table_class.name == action.name,
                        self.table_class.step_index == action.step_index
                    )
                    .values(
                        joint_angle=json.dumps(action.joint_angle)
                    )
                )
                reslult = await session.execute(stmt)
                return reslult.rowcount > 0

    async def get_action_joint_angles(
            self,
            name: str,
            step_index: int) -> SpotLiftAction | None:
        """
        Get the joint angles of a specific lift action by name and step index.
        Args:
            name (str): The name of the lift action.
            step_index (int): The index of the step in the lift action sequence.
        Returns:
            SpotLiftAction | None: The lift action with the specified name and step index,
                or None if it does not exist.
        """
        async with self.asnc_session() as session:
            stmt = select(
                self.table_class.joint_angle
            ).where(
                self.table_class.name == name,
                self.table_class.step_index == step_index
            )

            result = await session.execute(stmt)
            action = result.scalar_one_or_none()
            if action:
                return {
                    "name": name,
                    "step_index": step_index,
                    "joint_angle": action
                }

    async def close(self):
        """
        Close the database connection.
        This method should be called when the database operations are complete.
        """
        await self.engine.dispose()

async def main():
    """
    Main function to demonstrate the usage of AsyncSpotLiftActionDB.
    This function creates a database, adds a lift action, updates it,
    retrieves it, and then closes the database connection.
    """
    db = AsyncSpotLiftActionDB(
        table_name="spot_lift_action_test",
        db_url="postgresql+asyncpg://myuser:mypassword@localhost/db_actions")
    await db.create_tables()

    # Example of adding a lift action
    action = SpotLiftAction(
        name="lift_LF",
        step_index=1,
        joint_angle= json.dumps(
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
        )  # JSON string of joint angles
        # Example joint angles
    )
    await db.add_action(action)

    # Example of updating a lift action
    action.joint_angle = [0.2, 0.3, 0.4]
    await db.update_action(action)

    # Example of retrieving a lift action
    retrieved_action = await db.get_action_joint_angles("lift_LF", 1)
    if retrieved_action:
        print(f"Retrieved action: {retrieved_action}")
    else:
        print("Action not found.")

    await db.close()

if __name__ == "__main__":
    asyncio.run(main())
