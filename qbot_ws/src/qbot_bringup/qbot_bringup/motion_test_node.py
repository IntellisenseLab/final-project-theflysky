import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotionTestNode(Node):
    def __init__(self):
        super().__init__('motion_test_node')

        self.publisher = self.create_publisher(Twist, '/model/qbot/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.update_motion)  # 20 Hz

        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.state = 0
        self.state_start_time = self.start_time

        self.current_linear = 0.0
        self.current_angular = 0.0

        self.get_logger().info("Selective smoothing motion test started")

    def get_time(self):
        return self.get_clock().now().seconds_nanoseconds()[0]

    def smooth(self, current, target, step=0.01):
        if current < target:
            return min(current + step, target)
        elif current > target:
            return max(current - step, target)
        return current

    def update_motion(self):
        t = self.get_time()
        elapsed = t - self.state_start_time

        target_linear = 0.0
        target_angular = 0.0

        # ---- SMOOTH PART (UNCHANGED) ----

        if self.state == 0:  # Forward
            target_linear = 0.3
            if elapsed > 5:
                self.next_state()

        elif self.state == 1:  # Turn CW
            target_angular = -0.6
            if elapsed > 2.6:
                self.next_state()

        elif self.state == 2:  # Forward again
            target_linear = 0.3
            if elapsed > 5:
                self.next_state()

        # ---- FIXED PART (ADD STOP STATES HERE ONLY) ----

        elif self.state == 3:  # STOP before CCW turn
            # 🔧 Added stop to stabilize before problematic turn
            if elapsed > 0.8:
                self.next_state()

        elif self.state == 4:  # Turn CCW
            target_angular = 0.5   # slightly reduced for stability
            if elapsed > 3.0:
                self.next_state()

        elif self.state == 5:  # STOP before reverse
            # 🔧 Added stop before direction flip (critical)
            if elapsed > 0.8:
                self.next_state()

        elif self.state == 6:  # Backward
            target_linear = -0.25  # reduced speed to avoid instability
            if elapsed > 5:
                self.next_state()

        else:
            target_linear = 0.0
            target_angular = 0.0

        # ---- SMOOTH TRANSITION ----
        self.current_linear = self.smooth(self.current_linear, target_linear)
        self.current_angular = self.smooth(self.current_angular, target_angular)

        # ---- PUBLISH ----
        msg = Twist()
        msg.linear.x = self.current_linear
        msg.angular.z = self.current_angular

        self.publisher.publish(msg)

    def next_state(self):
        self.state += 1
        self.state_start_time = self.get_time()


def main(args=None):
    rclpy.init(args=args)
    node = MotionTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()