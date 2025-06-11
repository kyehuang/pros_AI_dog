import asyncio
import dataclasses
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy import Column, Integer, Text, UniqueConstraint, select
from sqlalchemy.orm import declarative_base, sessionmaker
from sqlalchemy.dialects.postgresql import insert
import json

Base = declarative_base()


@dataclasses.dataclass
class SpotNodePose(Base):
    """
    Spot robot four-legged node pose data model.
    This class represents the pose data of a Spot robot's four legs in a database table.
    It includes the positions of the feet and the corresponding databases for different states.
    Attributes:
    - id: Primary key, auto-incremented integer ID.
    - foot_positions: A string representing the positions of the four feet in space
        (suggested format: JSON string).
    - stance_pose_db: Database name for the pose when all feet are on the ground.
    - *_pose_db: Database name for the pose when the left front foot is about to lift.
    """
    __abstract__ = True

    id: int = Column(Integer, primary_key=True, autoincrement=True)

    # the positions of the four feet in space (JSON string)
    foot_positions: str = Column(Text, nullable=False)

    # the database name for the pose when all feet are on the ground
    stance_pose_db: str = Column(Text)

    # the database names for the poses when each foot is about to lift
    lf_lift_pose_db: str = Column(Text)
    rf_lift_pose_db: str = Column(Text)
    rb_lift_pose_db: str = Column(Text)
    lb_lift_pose_db: str = Column(Text)

def create_node_table_class(table_name: str):
    """
    According to the provided table_name, dynamically create a concrete subclass of SpotNodePose.
    This allows for creating multiple tables with the same structure but different names.
    Args:
        table_name (str): The name of the table to be created.
    Returns:
        type: A new class that inherits from SpotNodePose with the specified table name.
    """
    return type(
        table_name,
        (SpotNodePose,),
        {
            '__tablename__': table_name,
            '__table_args__': (
                UniqueConstraint('foot_positions', name=f'unique_{table_name}'),
            )
        }
    )

class AsyncSpotNameDB:
    """
    Asynchronous database handler for Spot robot node poses.
    This class provides methods to create tables, add and update node poses,
    and retrieve node IDs based on their foot positions.
    It uses SQLAlchemy's asynchronous capabilities to interact with a PostgreSQL database.
    Attributes:
    - table_name: The name of the table to store Spot node poses.
    - db_url: The database connection URL.
    Methods:
    - create_tables: Create the table if it does not exist.
    - add_node: Add a new node pose to the table, skipping if the foot position already exists.
    - update_node: Update an existing node pose based on its foot position.
    - get_node_id_by_foot_positions: Retrieve the node ID based on its foot position.
    - close: Close the database connection.
    """
    def __init__(
        self,
        table_name: str = "spot_nodes_name",
        db_url: str = "postgresql+asyncpg://myuser:mypassword@localhost:5432/db_name"
    ):
        self.engine = create_async_engine(db_url, echo=False)
        self.async_session = sessionmaker(
            self.engine,
            expire_on_commit=False,
            class_=AsyncSession
        )
        self.table_class = create_node_table_class(table_name)

    async def create_tables(self):
        """
        Create the SpotNodePose table in the database if it does not already exist.
        """
        async with self.engine.begin() as conn:
            await conn.run_sync(self.table_class.metadata.create_all)

    async def add_node(self, node_pose: SpotNodePose):
        """
        Add a new node pose to the database.
        If a node with the same foot_positions already exists, it will skip adding it.
        Args:
            node_pose (SpotNodePose): The node pose to be added.
        Returns:
            None
        Raises:
            Exception: If there is an error during the database operation.
        """
        async with self.async_session() as session:
            async with session.begin():
                stmt = insert(self.table_class).values(
                    foot_positions=node_pose.foot_positions,
                    stance_pose_db=node_pose.stance_pose_db,
                    lf_lift_pose_db=node_pose.lf_lift_pose_db,
                    rf_lift_pose_db=node_pose.rf_lift_pose_db,
                    rb_lift_pose_db=node_pose.rb_lift_pose_db,
                    lb_lift_pose_db=node_pose.lb_lift_pose_db,
                ).on_conflict_do_nothing(
                    index_elements=['foot_positions']
                )
                await session.execute(stmt)

    async def update_node(self, node_pose: SpotNodePose) -> bool:
        """
        Based on the foot_positions, update the corresponding node's data_base field.
        This method updates the node's pose data in the database.
        Args:
            node_pose (SpotNodePose): The node pose to be updated.
        Returns:
            bool: True if the update was successful, False if no matching node was found.
        """
        async with self.async_session() as session:
            async with session.begin():
                stmt = (
                    self.table_class.__table__.update()
                    .where(self.table_class.foot_positions == node_pose.foot_positions)
                    .values(
                        stance_pose_db=node_pose.stance_pose_db,
                        lf_lift_pose_db=node_pose.lf_lift_pose_db,
                        rf_lift_pose_db=node_pose.rf_lift_pose_db,
                        rb_lift_pose_db=node_pose.rb_lift_pose_db,
                        lb_lift_pose_db=node_pose.lb_lift_pose_db
                    )
                )
                result = await session.execute(stmt)
                return result.rowcount > 0


    async def get_pose_dbs_by_foot_positions(self, foot_positions: str) -> dict | None:
        """
        Retrieve the pose databases for a given foot position string.
        Args:
            foot_positions (str): A string representing the positions of the four feet in space.
        Returns:
            dict | None: A dictionary containing the pose databases for the stance and lifted feet,
                          or None if no matching node is found.
        """
        async with self.async_session() as session:
            stmt = select(
                self.table_class.stance_pose_db,
                self.table_class.lf_lift_pose_db,
                self.table_class.rf_lift_pose_db,
                self.table_class.rb_lift_pose_db,
                self.table_class.lb_lift_pose_db,
            ).where(self.table_class.foot_positions == foot_positions)

            result = await session.execute(stmt)
            row = result.first()

            if row:
                return {
                    "stance": row.stance_pose_db,
                    "lf_lift": row.lf_lift_pose_db,
                    "rf_lift": row.rf_lift_pose_db,
                    "rb_lift": row.rb_lift_pose_db,
                    "lb_lift": row.lb_lift_pose_db
                }

            return None


    async def close(self):
        """
        Close the database connection.
        """
        await self.engine.dispose()


async def main():
    db = AsyncSpotNameDB(
        table_name="spot_nodes_name_test",
        db_url="postgresql+asyncpg://myuser:mypassword@localhost:5432/db_name"
    )
    await db.create_tables()

    # Example of adding a node
    node_pose_a = SpotNodePose(
        foot_positions=json.dumps([
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0]
        ]),
        stance_pose_db='stance_db_a',
        lf_lift_pose_db='lf_lift_db_a',
        rf_lift_pose_db='rf_lift_db_a',
        rb_lift_pose_db='rb_lift_db_a',
        lb_lift_pose_db='lb_lift_db_a'
    )
    node_pose_b = SpotNodePose(
        foot_positions=json.dumps([
            [0.0, 0.0, 0.05],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0]
        ]),
        stance_pose_db='stance_db_b',
        lf_lift_pose_db='lf_lift_db_b',
        rf_lift_pose_db='rf_lift_db_b',
        rb_lift_pose_db='rb_lift_db_b',
        lb_lift_pose_db='lb_lift_db_b'
    )

    await db.add_node(node_pose_a)
    await db.add_node(node_pose_b)
    print("Nodes added successfully.")

    # Example of updating a node
    node_pose_a.lf_lift_pose_db = 'updated_lf_lift_db_a'
    updated = await db.update_node(node_pose_a)
    if updated:
        print("Node updated successfully.")
    else:
        print("No matching node found to update.")
    # Example of retrieving pose databases by foot positions
    foot_positions = json.dumps([
        [0.0, 0.0, 0.05],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0]
    ])
    pose_dbs = await db.get_pose_dbs_by_foot_positions(foot_positions)
    if pose_dbs:
        print("Retrieved pose databases:", pose_dbs)
    else:
        print("No matching node found for the given foot positions.")
    await db.close()

if __name__ == "__main__":
    asyncio.run(main())
