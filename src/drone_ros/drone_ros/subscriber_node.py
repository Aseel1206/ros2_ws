import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RelayToMavros(Node):
    def __init__(self):
        super().__init__('relay_to_mavros')
        self.subscription = self.create_subscription(
            Twist,
            '/demo_topic',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Received velocity: x={msg.linear.x}")
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RelayToMavros()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
