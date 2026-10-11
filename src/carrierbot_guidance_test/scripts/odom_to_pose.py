#!/usr/bin/env python3
"""Chuyển nav_msgs/Odometry (EKF) sang PoseWithCovarianceStamped cho guidance_test_node.

Dùng khi thử Guidance trên xe thật mà không chạy AMCL: guidance chỉ nhận
PoseWithCovarianceStamped, còn EKF phát nav_msgs/Odometry trên /odometry/filtered.

Tham số:
  input_topic  (str)   topic Odometry đầu vào, mặc định /odometry/filtered
  output_topic (str)   topic tư thế cho guidance, mặc định /guidance_pose
  rate_hz      (float) giới hạn tần số phát. 0 = chuyển mọi bản tin (khoảng 30 Hz).
                       Đặt 2.0 để giả lập tần số cập nhật của AMCL.
"""

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomToPose(Node):

    def __init__(self):
        super().__init__('odom_to_pose')
        input_topic = self.declare_parameter('input_topic', '/odometry/filtered').value
        output_topic = self.declare_parameter('output_topic', '/guidance_pose').value
        rate_hz = float(self.declare_parameter('rate_hz', 0.0).value)
        self.min_period_ns = int(1e9 / rate_hz) if rate_hz > 0.0 else 0
        self.last_publish_ns = None

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, output_topic, 10)
        self.create_subscription(Odometry, input_topic, self.on_odom, 10)
        self.get_logger().info(
            f'odom_to_pose: {input_topic} -> {output_topic} '
            f'({"moi ban tin" if rate_hz <= 0.0 else f"toi da {rate_hz:.1f} Hz"})')

    def on_odom(self, msg: Odometry):
        now_ns = self.get_clock().now().nanoseconds
        if (self.min_period_ns and self.last_publish_ns is not None and
                now_ns - self.last_publish_ns < self.min_period_ns):
            return
        self.last_publish_ns = now_ns

        pose = PoseWithCovarianceStamped()
        pose.header = msg.header
        pose.pose = msg.pose
        self.publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
