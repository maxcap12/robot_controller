from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

class JointNode(Node):
    def __init__(self):
        super().__init__('joint_node')

        self.joint_positions = None
        self.joint_velocities = None

        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        self.pause_play_pub = self.create_publisher(Empty, '/pause_play', 10)


    def joint_state_callback(self, msg):
        self.joint_positions = list(msg.position)
        self.joint_velocities = list(msg.velocity)

    def get_joint_attributes(self):
        return self.joint_positions, self.joint_velocities
    
    def play_pause(self):
        self.pause_play_pub.publish(Empty())