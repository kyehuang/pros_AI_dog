"""
This file contains the AI_dog_node class, which is a ROS2 node that subscribes to the
following topics:
"""
import threading
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from trajectory_msgs.msg import JointTrajectoryPoint

class AIDogNode(Node):
    """
    AI_dog_node class to culculate the dog's state and publish the action to the dog.
    """
    def __init__(self):
        super().__init__("AI_dog_node")
        self.get_logger().info("AI_dog_node has started.")

        # Initialize subscriber node
        self.__subscriber_node()

        # Initialize publisher node
        self.__publisher_node()

        # Initialize variables
        self.__latest_data : dict = None

        # dog updated flag
        self._date_received_event = threading.Event()


    ## Initialize publisher
    def __publisher_node(self):
        """
        Publisher node for AI_dog_node
        """
        # Publisher for spot_actions
        self.__publisher_spot_actions = self.create_publisher(
            JointTrajectoryPoint,
            "spot_actions",
            10,
        )

        # Publisher for reset unity
        self.__publisher_dog_scene_reset = self.create_publisher(
            Bool,
            "reset_unity",
            10,
        )

    ## Initialize subscriber
    def __subscriber_node(self):
        """
        Subscriber node for AI_dog_node
        """
        __subscriber_spot_rotate_state = self.create_subscription(
            Float32MultiArray,
            "spot_states",
            self.__listener_callback_spot_states,
            10,
        )

    ## Publish function

    # Publish spot_actions
    def publish_spot_actions(self, spot_actions: list) -> None:
        """
        Publish spot_actions

        Args:
            spot_actions (list): List of spot actions
        """
        # Ensure all values in spot_actions are float
        if not all(isinstance(action, float) for action in spot_actions):
            raise ValueError("All spot actions must be floats.")

        # turn deg to rad
        spot_actions = [self.__deg_to_rad(action) for action in spot_actions]

        msg = JointTrajectoryPoint()
        msg.positions = spot_actions
        self.__publisher_spot_actions.publish(msg)

    ## Callback function for subscriber
    def __listener_callback_spot_states(self, msg: Float32MultiArray) -> None:
        # self.get_logger().info(f"I heard: {msg.data}")
        self.__latest_data = {
            "spot_states": msg.data,
        }

        self._date_received_event.set()

    ## Utility function

    def reset_unity(self):
        """
        Reset Unity environment
        """
        msg = Bool()
        msg.data = True
        self.__publisher_dog_scene_reset.publish(msg)

    # Get latest data
    def reset_latest_data(self):
        """
        Reset latest data
        """
        self._date_received_event.clear()
        self.__latest_data = None

    # Get latest data
    def get_latest_data(self, timeout: float =1.0) -> dict:
        """
        Get latest data

        Args:
            timeout (float): Timeout for waiting for data.
                             unit: second

        Returns:
            dict: Latest data.
        """
        if self.__latest_data is not None:
            return self.__latest_data

        while True:
            received = self._date_received_event.wait(timeout=timeout)
            if received and self.__latest_data is not None:
                break

            self.get_logger().warn("Timeout waiting for data")

        self._date_received_event.clear()
        return self.__latest_data

    # deg to rad
    @staticmethod
    def __deg_to_rad(deg: float) -> float:
        """
        Convert degrees to radians.

        Args:
            deg (float): Angle in degrees.

        Returns:
            float: Angle in radians.
        """
        return deg * 0.01745329252
