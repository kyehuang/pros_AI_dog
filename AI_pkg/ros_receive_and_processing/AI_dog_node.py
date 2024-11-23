from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Bool
import rclpy
import threading

class AI_dog_node(Node):
    def __init__(self):
        super().__init__("AI_dog_node")
        self.get_logger().info("AI_dog_node has started.")

        # Initialize subscriber node
        self.__subscriber_node()

        # Initialize publisher node
        self.__publisher_node()

        # Initialize variables
        self.__dog_data : dict = {} # dog data
        self.__latest_data : dict = None # latest data
        self.__dog_updated_flag : dict[str, bool] = { # dog updated flag
            "motor_states": False,
            "spot_states": False,
            "target_pos": False,
        }

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
        self.__publisher_spot_actions

        # Publisher for reset unity
        self.__publisher_dog_scene_reset = self.create_publisher(
            Bool,
            "reset_unity",
            10,
        )
        self.__publisher_dog_scene_reset

    ## Initialize subscriber
    def __subscriber_node(self):
        """
        Subscriber node for AI_dog_node
        """

        # Subscriber for four_feet_state
        self.__subscriber_four_feet_state = self.create_subscription(
            Float32MultiArray,
            "four_feet_state",
            self.__listener_callback_four_feet_state,
            10,
        )
        self.__subscriber_four_feet_state

        self.__subscriber_spot_rotate_state = self.create_subscription(
            Float32MultiArray,
            "spot_rotate_state",
            self.__listener_callback_spot_rotate_state,
            10,
        )
        self.__subscriber_spot_rotate_state

        self.__subscriber_targe_pos = self.create_subscription(
            Float32MultiArray,
            "target_state",
            self.__listener_callback_target_pos,
            10,
        )
        self.__subscriber_targe_pos

        self.__subscriber_motor_states = self.create_subscription(
            Float32MultiArray,
            "spot_motor_state",
            self.__listener_callback_motor_states,
            10,
        )
        self.__subscriber_motor_states

    ## Publish function

    # Publish spot_actions
    def publish_spot_actions(self, spot_actions):
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

    # Callback function for four_feet_state
    def __listener_callback_four_feet_state(self, msg):
        pass
        # self.get_logger().info("I heard: %s" % msg.data)

    # Callback function for spot_rotate_state
    def __listener_callback_spot_rotate_state(self, msg):
        # self.get_logger().info("I heard: %s" % msg.data)
        self.__dog_data["spot_states"] = msg.data
        self.__dog_updated_flag["spot_states"] = True
        self.__check_all_data_updated()

    # Callback function for target_pos
    def __listener_callback_target_pos(self, msg):
        # self.get_logger().info("I heard: %s" % msg.data)
        self.__dog_data["target_pos"] = msg.data
        self.__dog_updated_flag["target_pos"] = True
        self.__check_all_data_updated()

    # Callback function for motor_states
    def __listener_callback_motor_states(self, msg):
        # self.get_logger().info("I heard: %s" % msg.data)
        self.__dog_data["motor_states"] = msg.data
        self.__dog_updated_flag["motor_states"] = True
        self.__check_all_data_updated()

    ## Utility function

    def reset_unity(self):
        """
        Reset Unity environment
        """
        msg = Bool()
        msg.data = True #  Reset Unity environment
        self.__publisher_dog_scene_reset.publish(msg)

    # Get latest data
    def reset_latest_data(self):
        """
        Reset latest data
        """
        self.__latest_data = None

    # Get latest data
    def get_latest_data(self):
        """
        Get latest data

        Returns:
            dict: Dictionary of dog data
        """
        dog_data = self.__latest_data

        i = 0 # counter
        while dog_data is None:
            if (i % 90000000 == 0) and i != 0:
                self.get_logger().info("Waiting for data...")
            dog_data = self.__latest_data
            i += 1
        return dog_data

    # deg to rad
    def __deg_to_rad(self, deg):
        return deg * 0.01745329252

    # check if all data is updated
    def __check_all_data_updated(self):
        """
        Check if all data is updated

        Returns:
            dict: Dictionary of dog data
        """
        if all(self.__dog_updated_flag.values()):
            for key in self.__dog_updated_flag:
                self.__dog_updated_flag[key] = False
            # latest data is updated
            self.__latest_data = self.__prepose_data()

    # prepose data
    def __prepose_data(self):
        """
        Prepose data

        Returns:
            dict: Dictionary of dog data
                the latest data in AI_dog_node. Ready to be used by other nodes.
        """
        state_dict = {
            "motor_states": self.__dog_data["motor_states"],
            "spot_states": self.__dog_data["spot_states"],
            "target_pos": self.__dog_data["target_pos"],
        }
        return state_dict
