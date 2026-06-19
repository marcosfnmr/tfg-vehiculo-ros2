import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2   # avanzar
        msg.angular.z = 0.3  # girar

        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: {msg.linear.x}, {msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()