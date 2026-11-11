#!/usr/bin/python3
"""cmd_vel watchdog: zero the robot if the commanding node goes silent.

Monitors /cmd_vel. If the last received command was non-zero and no new
message arrives within `timeout` seconds, publishes a zero Twist once.
Also re-publishes zero if the zero command itself is lost. It never
forwards normal traffic, so it does not add latency or duplicate messages.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def nonzero(t: Twist) -> bool:
    return any(abs(v) > 1e-6 for v in (
        t.linear.x, t.linear.y, t.linear.z,
        t.angular.x, t.angular.y, t.angular.z))


class CmdVelWatchdog(Node):
    def __init__(self):
        super().__init__('cmd_vel_watchdog')
        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('rate', 20.0)
        self.timeout = float(self.get_parameter('timeout').value)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cb, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.last_time = None
        self.last_cmd = Twist()
        self.armed = False  # True once we published a zero for a stale cmd
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter('rate').value), self.check)

    def cb(self, msg):
        self.last_time = self.get_clock().now()
        self.last_cmd = msg
        self.armed = False

    def check(self):
        if self.armed or self.last_time is None:
            return
        dt = (self.get_clock().now() - self.last_time).nanoseconds * 1e-9
        if dt > self.timeout and nonzero(self.last_cmd):
            self.get_logger().warn(
                f'cmd_vel silent for {dt:.1f}s with non-zero command -> zeroing')
            self.pub.publish(Twist())
            self.last_cmd = Twist()
            self.armed = True


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
