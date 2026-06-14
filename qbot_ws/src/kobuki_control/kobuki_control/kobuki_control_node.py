import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial
from serial.tools import list_ports

# The Kobuki ships with an FTDI USB-serial adapter (USB id 0403:6001) whose
# descriptor contains "Kobuki". We detect the port ourselves by matching on
# either, so the node no longer depends on the (Windows-only, buggy) port
# auto-detection in the third-party kobukidriver package.
KOBUKI_USB_ID = '0403:6001'
KOBUKI_DESC_HINT = 'kobuki'


class KobukiControlNode(Node):
    def __init__(self):
        super().__init__('kobuki_control_node')

        self.serial = None
        self.connected = False

        # Fail-safe command limits for hardware testing
        self.max_linear = 0.20   # m/s
        self.max_angular = 1.00  # rad/s

        # Find and open the Kobuki serial port ourselves.
        port = self._find_kobuki_port()
        if port is not None:
            try:
                self.serial = serial.Serial(port=port, baudrate=115200, timeout=0.1)
                self.connected = True
                self.get_logger().info(f'Kobuki connected on {port}.')
            except Exception as e:
                self.get_logger().warning(f'Found Kobuki at {port} but could not open it: {e}')
        else:
            self.get_logger().warning(
                'No Kobuki serial port found (looked for USB '
                f'{KOBUKI_USB_ID} or "{KOBUKI_DESC_HINT}" in the port description).'
            )

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
        linear = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        angular = max(-self.max_angular, min(self.max_angular, msg.angular.z))

        speed, radius = self.twist_to_speed_radius(linear, angular)

        if self.connected and self.serial is not None:
            try:
                self._send_base_control(speed, radius)

                self.get_logger().info(
                    f'[ROS] linear: {linear:.3f} m/s, angular: {angular:.3f} rad/s'
                )
                self.get_logger().info(
                    f'[KOBUKI] speed: {speed} mm/s, radius: {radius} mm'
                )

            except Exception as e:
                self.get_logger().error(f'Failed to send Kobuki command: {e}')
        else:
            self.get_logger().warning(
                f'No Kobuki connected. Computed [Kobuki] command -> speed: {speed}, radius: {radius}'
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

    @staticmethod
    def _find_kobuki_port():
        """Return the /dev path of the Kobuki's USB-serial adapter, or None."""
        for p in list_ports.comports():
            hwid = (p.hwid or '').upper()
            desc = (p.description or '').lower()
            if KOBUKI_USB_ID.upper() in hwid or KOBUKI_DESC_HINT in desc:
                return p.device
        return None

    def _send_base_control(self, speed, radius):
        """Build and write a Kobuki Base Control serial packet.

        Wire format: header 0xAA 0x55, length 0x06, then the sub-payload
        [cmd=0x01, len=0x04, speed(int16 LE), radius(int16 LE)], followed by a
        one-byte XOR checksum over the length byte and the whole sub-payload.
        """
        packet = bytearray([0xAA, 0x55, 0x06, 0x01, 0x04])
        packet += int(speed).to_bytes(2, byteorder='little', signed=True)
        packet += int(radius).to_bytes(2, byteorder='little', signed=True)
        checksum = 0
        for byte in packet[2:]:
            checksum ^= byte
        packet.append(checksum)
        self.serial.write(packet)

    def stop_robot(self):
        if self.connected and self.serial is not None:
            try:
                self._send_base_control(0, 0)
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