import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateBridge(Node):
    """
    Subscribes to sm_shi (JointState from IK node).
    Continuously republishes the last received state to /joint_states at 50Hz.
    Holds last position when IK stops publishing.
    """

    def __init__(self):
        super().__init__('joint_state_bridge')

        # Start with all joints at zero
        self.last_msg = JointState()
        self.last_msg.name = [
            'leg1_coxa_joint', 'leg1_femur_joint', 'leg1_tibia_joint',
            'leg2_coxa_joint', 'leg2_femur_joint', 'leg2_tibia_joint',
            'leg3_coxa_joint', 'leg3_femur_joint', 'leg3_tibia_joint',
            'leg4_coxa_joint', 'leg4_femur_joint', 'leg4_tibia_joint',
            'leg5_coxa_joint', 'leg5_femur_joint', 'leg5_tibia_joint',
            'leg6_coxa_joint', 'leg6_femur_joint', 'leg6_tibia_joint',
        ]
        self.last_msg.position = [0.0] * 18
        self.last_msg.velocity = []
        self.last_msg.effort = []

        # Publisher → robot_state_publisher reads this
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscriber — IK node publishes JointState here
        self.sub = self.create_subscription(
            JointState,
            'sm_shi',
            self.ik_callback,
            10
        )

        # Timer: publish at 50Hz regardless of IK activity
        self.timer = self.create_timer(0.02, self.publish_joints)

        self.get_logger().info('JointStateBridge started. Listening on sm_shi')

    def ik_callback(self, msg: JointState):
        # Store latest JointState from IK node as-is
        # IK already converts to radians — no modification needed
        self.last_msg = msg

    def publish_joints(self):
        # Always publish last known state — holds position when IK is idle
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.last_msg.name
        msg.position = self.last_msg.position
        msg.velocity = []
        msg.effort = []
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()