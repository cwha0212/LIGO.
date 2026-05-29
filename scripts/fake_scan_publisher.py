#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FakeScanPublisher(Node):
    def __init__(self):
        super().__init__('fake_scan_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)
        self.get_logger().info('Fake LaserScan publisher started on /scan at 10Hz')

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'aft_mapped'
        msg.angle_min = -3.14159
        msg.angle_max = 3.14159
        msg.angle_increment = 0.01745
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 30.0
        msg.ranges = []
        msg.intensities = []
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeScanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
