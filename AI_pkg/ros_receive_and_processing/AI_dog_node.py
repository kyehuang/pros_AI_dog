from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint


class AI_dog_node(Node):
    def __init__(self):
        super().__init__("AI_dog_node")
        self.get_logger().info("AI_dog_node has started.")
        
        # Initialize subscriber node
        self.__subscriber_node()

        # Initialize publisher node
        self.__publisher_node()

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


    ## Utility function

    # deg to rad
    def __deg_to_rad(self, deg):
        return deg * 0.01745329252

        