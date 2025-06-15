"""
This file contains the AI_dog_node class, which is a ROS2 node that subscribes to the
following topics:
"""
import threading
import copy
import time

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

        action = self.data_arrange(spot_actions)

        msg = JointTrajectoryPoint()
        msg.positions = action
        self.__publisher_spot_actions.publish(msg)

    def data_arrange(self, spot_actions: list) -> list:
        """
        Arrange the data for spot actions
        Args:
            spot_actions (list): List of spot actions
        Returns:
            list: List of arranged spot actions
        """
        actions_copy = copy.deepcopy(spot_actions)

        # LF, RB : first joint
        actions_copy[0] = 90 + actions_copy[0]  # LF
        actions_copy[6] = 90 - actions_copy[6]  # RB
        # LF, RB : second joint
        actions_copy[1] = actions_copy[1]       # LF
        actions_copy[7] = 180 - actions_copy[7] # RB
        # LF, RB : third joint
        actions_copy[2] = 180 - actions_copy[2] # LF
        actions_copy[8] = actions_copy[8]       # RB

        # RF, LB : first joint
        actions_copy[3] = 90 + actions_copy[3]
        actions_copy[9] = 90 - actions_copy[9]
        # RF, LB : second joint
        actions_copy[4]  = 180 -actions_copy[4]
        actions_copy[10] = actions_copy[10]
        # RF, LB : third joint
        actions_copy[5]  = actions_copy[5]
        actions_copy[11] = 180 - actions_copy[11]

        # turn deg to rad
        actions_copy = [self.__deg_to_rad(action) for action in actions_copy]

        return actions_copy

    def send_joint_angle_trajectory(
            self,
            start_action: list,
            target_action: list,
            step: int = 10,
            delay: float = 0.03) -> None:
        """
        Send joint angle trajectory to the dog.
        Args:
            start_action (list): Start action for the dog.
            target_action (list): Target action for the dog.
            step (int): Number of steps to reach the target action.
            delay (float): Delay between each step in seconds.
        """
        # Ensure all values in start_action and target_action are float
        if not (all(isinstance(action, float) for action in start_action) and
                all(isinstance(action, float) for action in target_action)):
            raise ValueError("All actions must be floats.")

        # Arrange the data
        for i in range(1, step + 1):
            t = i / (step + 1)
            interpolated = [
                (1 - t) * start_action[j] + t * target_action[j]
                for j in range(len(target_action))
            ]
            self.publish_spot_actions(interpolated)

            time.sleep(delay)

        self.publish_spot_actions(target_action)  # Ensure the final action is sent

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
