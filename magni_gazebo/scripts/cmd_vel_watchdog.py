import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration

class CmdVelWatchdog(Node):
    def __init__(self):
        super().__init__('cmd_vel_watchdog')

        self.timeout = 0.2  # seconds
        self.last_cmd_time = self.get_clock().now()
        self.active = False  # Start inactive

        self.cmd_sub = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info('CmdVel Watchdog initialized, waiting for first cmd_vel (TwistStamped)...')

    def cmd_callback(self, msg: TwistStamped):
        self.last_cmd_time = self.get_clock().now()
        if not self.active:
            self.active = True
            self.get_logger().info('First cmd_vel received. Watchdog activated.')

    def watchdog_callback(self):
        if not self.active:
            return  # Do nothing until first message is received

        time_since_last_cmd = self.get_clock().now() - self.last_cmd_time

        if time_since_last_cmd > Duration(seconds=self.timeout):
            # Publish zero TwistStamped
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = self.get_clock().now().to_msg()
            twist_stamped.header.frame_id = 'base_link'
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = 0.0
            self.cmd_pub.publish(twist_stamped)
            self.get_logger().info('Timeout! Zero velocity sent.')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
