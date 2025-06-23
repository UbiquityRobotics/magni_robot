import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration

class CmdVelWatchdog(Node):
    def __init__(self):
        super().__init__('cmd_vel_watchdog')

        self.timeout = 0.5  # seconds
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

        self.get_logger().info('CmdVel Watchdog initialized, waiting for first cmd_vel...')

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
            stamped_twist = TwistStamped()
            stamped_twist.header.stamp = self.get_clock().now().to_msg()
            stamped_twist.header.frame_id = 'base_link'
            stamped_twist.twist.linear.x = 0.0
            stamped_twist.twist.angular.z = 0.0
            self.cmd_pub.publish(stamped_twist)
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
