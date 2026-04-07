import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from kobukidriver import Kobuki


class KobukiControlNode(Node):
    def __init__(self):
        super().__init__('kobuki_control_node')

        self.robot = None
        self.connected = False

        # Try to connect to real Kobuki hardware
        try:
            self.robot = Kobuki()
            self.connected = True
            self.get_logger().info('Kobuki connected successfully.')
        except Exception as e:
            self.connected = False
            self.get_logger().warning(f'Kobuki not connected: {e}')

        # Subscribe to velocity commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info('Kobuki control node started. Listening on /cmd_vel')

    def cmd_vel_callback(self, msg: Twist) -> None:
        linear = msg.linear.x
        angular = msg.angular.z

        speed, radius = self.twist_to_speed_radius(linear, angular)

        if self.connected and self.robot is not None:
            try:
                self.robot.base_control(speed, radius)
                self.get_logger().info(
                    f'Sent command -> speed: {speed}, radius: {radius}'
                )
            except Exception as e:
                self.get_logger().error(f'Failed to send Kobuki command: {e}')
        else:
            self.get_logger().warning(
                f'No Kobuki connected. Computed command -> speed: {speed}, radius: {radius}'
            )

    def twist_to_speed_radius(self, linear: float, angular: float) -> tuple[int, int]:
        """
        Convert ROS2 Twist (linear.x, angular.z) into Kobuki base_control(speed, radius).

        Assumption:
        - speed is sent as an integer proportional to linear velocity
        - radius is derived from linear/angular relationship
        - radius = 0 is used for straight motion, matching the driver behavior
        """

        # Scale linear velocity into integer command space.
        # This is an initial assumption and may need tuning with real hardware.
        speed = int(linear * 1000)

        # Straight motion
        if abs(angular) < 1e-6:
            radius = 0
            return speed, radius

        # Pure rotation in place or near-zero linear speed
        if abs(linear) < 1e-6:
            # Large difference from straight motion; using a small signed radius proxy.
            # This may need adjustment once real hardware is available.
            radius = 1 if angular > 0 else -1
            return 0, radius

        # General curved motion
        radius_m = linear / angular
        radius = int(radius_m * 1000)

        return speed, radius

    def stop_robot(self) -> None:
        if self.connected and self.robot is not None:
            try:
                self.robot.base_control(0, 0)
                self.get_logger().info('Stop command sent to Kobuki.')
            except Exception as e:
                self.get_logger().error(f'Failed to stop Kobuki: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = KobukiControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()