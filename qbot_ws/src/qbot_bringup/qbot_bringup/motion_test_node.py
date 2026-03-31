import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotionTestNode(Node):
    def __init__(self):
        super().__init__('motion_test_node')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.start_time = self.get_clock().now()
        self.get_logger().info('Motion test node started.')

    def timer_callback(self):
        msg = Twist()
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if 0.0 <= elapsed < 3.0:
            msg.linear.x = 0.2
            msg.angular.z = 0.0

        elif 3.0 <= elapsed < 5.0:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        elif 5.0 <= elapsed < 8.0:
            msg.linear.x = 0.0
            msg.angular.z = 0.5

        elif 8.0 <= elapsed < 10.0:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info('Motion test complete.')
            self.destroy_timer(self.timer)
            rclpy.shutdown()
            return

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotionTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()