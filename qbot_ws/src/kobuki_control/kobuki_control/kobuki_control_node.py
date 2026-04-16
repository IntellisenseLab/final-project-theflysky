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

        # Try to connect to Kobuki
        try:
            self.robot = Kobuki()
            self.connected = True
            self.get_logger().info('Kobuki connected successfully.')
        except Exception as e:
            self.connected = False
            self.get_logger().warning(f'Kobuki not connected: {e}')

        # Subscribe to /cmd_vel topic for velocity commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info('Kobuki control node started. Listening on /cmd_vel')

    def cmd_vel_callback(self, msg):
        # Only parameters needed for kobuki 2D motion
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



    def twist_to_speed_radius(self, linear, angular):
        """
        Handle 2 major conflicts between ROS2 (/cmd_vel Twist:msg) and Kobuki API (base_control(speed, radius)):
            A) Unit conversions
            B) Special encoded parameters expected by the API for special cases of motions

        ROS2 Twist:
        - linear.x: (m/s)
        - angular.z: (rad/s)

        Kobuki Driver base_control:
        - speed: (mm/s)
        - radius: (mm)

        AS PER DOCUMENTATION
        """

        speed = int(linear * 1000)

        # Pure Translation
        if abs(angular) < 1e-6:
            # Specific encoding as per documentation
            radius = 0

        # Pure rotation
        elif abs(linear) < 1e-6:
            b = 230 # Kobuki wheelbase
            # Specific encoding (otherwise speed = 0)
            speed = int((angular * b) / 2) 
            # Specific encoding (Flag for spin_in one place)
            radius = 1 

        # General (Translation + Rotation)
        else:
            radius = int((linear / angular) * 1000)

        return speed, radius

    def stop_robot(self):
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