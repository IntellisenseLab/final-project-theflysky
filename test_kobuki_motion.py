#!/usr/bin/env python3
"""Small, safe Kobuki motion test.

Publishes short, gentle velocity bursts on ``/cmd_vel`` so you can confirm the
robot moves the right way. Each move is deliberately small and always ends
with a stop command.

Requires the Kobuki bridge to be running in another terminal:
    ros2 run kobuki_control kobuki_control_node

Then run one move at a time:
    python3 test_kobuki_motion.py forward
    python3 test_kobuki_motion.py backward
    python3 test_kobuki_motion.py spin
    python3 test_kobuki_motion.py oscillate

NOTE: do NOT run this while the full pipeline (run_qbot.sh) is active — the
behaviour node also drives /cmd_vel and the two would fight for control.
"""

import sys
import time

import rclpy
from geometry_msgs.msg import Twist

# Gentle limits (the bridge clamps to 0.20 m/s and 1.0 rad/s anyway).
LIN = 0.08          # m/s  -> ~8 cm over 1 s
ANG = 0.6           # rad/s -> ~34 deg/s
PUB_HZ = 20.0


def main() -> None:
    motion = sys.argv[1].lower() if len(sys.argv) > 1 else ""
    moves = {"forward", "backward", "spin", "oscillate"}
    if motion not in moves:
        print(f"usage: python3 test_kobuki_motion.py [{'|'.join(sorted(moves))}]")
        sys.exit(1)

    rclpy.init()
    node = rclpy.create_node("kobuki_motion_test")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    time.sleep(0.6)  # let discovery match the bridge

    def drive(linear: float, angular: float, duration: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        end = time.time() + duration
        while time.time() < end and rclpy.ok():
            pub.publish(msg)
            time.sleep(1.0 / PUB_HZ)

    def stop(duration: float = 0.6) -> None:
        drive(0.0, 0.0, duration)

    node.get_logger().info(f"Running '{motion}' (small movement)...")
    if motion == "forward":
        drive(LIN, 0.0, 1.0)
    elif motion == "backward":
        drive(-LIN, 0.0, 1.0)
    elif motion == "spin":
        drive(0.0, ANG, 1.2)
    elif motion == "oscillate":
        for _ in range(3):
            drive(0.0, ANG, 0.35)
            drive(0.0, -ANG, 0.35)

    stop()
    node.get_logger().info("Done. Robot stopped.")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
