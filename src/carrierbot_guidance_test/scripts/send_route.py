#!/usr/bin/env python3
"""Gửi một tuyến waypoint cho guidance_test_node, toạ độ tính theo vị trí xuất phát của xe.

Mỗi điểm viết dạng TOI,TRAI (mét) trong hệ toạ độ gắn với xe lúc bắt đầu:
TOI = phía trước mũi xe, TRAI = bên trái xe (âm = bên phải).
Script đọc tư thế hiện tại một lần, đổi các điểm sang hệ odom rồi phát lên
/fablab_waypoints. Điểm xuất phát do guidance tự thêm, không cần ghi.

Ví dụ:
  # Đi thẳng 6 m
  ros2 run carrierbot_guidance_test send_route.py 6,0
  # Đi thẳng 4 m rồi rẽ trái 90 độ, đi tiếp 4 m
  ros2 run carrierbot_guidance_test send_route.py 4,0 4,4
  # Hình vuông 3 m, quay về điểm đầu
  ros2 run carrierbot_guidance_test send_route.py 3,0 3,3 0,3 0,0
  # Chỉ in toạ độ, không gửi
  ros2 run carrierbot_guidance_test send_route.py 4,0 4,4 --dry-run
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


def parse_point(text):
    try:
        forward, left = (float(v) for v in text.split(','))
    except ValueError:
        raise argparse.ArgumentTypeError(f'"{text}" khong dung dang TOI,TRAI (vi du 4,0)')
    return forward, left


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RouteSender(Node):

    def __init__(self, pose_topic, waypoint_topic):
        super().__init__('send_route')
        self.pose = None
        self.create_subscription(PoseWithCovarianceStamped, pose_topic, self.on_pose, 10)
        self.publisher = self.create_publisher(PoseStamped, waypoint_topic, 100)

    def on_pose(self, msg):
        self.pose = msg

    def wait_for(self, condition, timeout_s):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not condition() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return condition()


def main():
    parser = argparse.ArgumentParser(
        description='Gui tuyen waypoint (toa do theo vi tri xuat phat cua xe) cho guidance.')
    parser.add_argument('points', nargs='+', type=parse_point, help='Cac diem TOI,TRAI tinh bang met')
    parser.add_argument('--pose-topic', default='/guidance_pose')
    parser.add_argument('--waypoint-topic', default='/fablab_waypoints')
    parser.add_argument('--frame', default='odom')
    parser.add_argument('--dry-run', action='store_true', help='Chi in toa do, khong gui')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = RouteSender(args.pose_topic, args.waypoint_topic)
    try:
        if not node.wait_for(lambda: node.pose is not None, 5.0):
            node.get_logger().error(f'Khong nhan duoc tu the tren {args.pose_topic} sau 5 s.')
            return 1

        p = node.pose.pose.pose
        x0, y0, yaw0 = p.position.x, p.position.y, yaw_from_quaternion(p.orientation)
        c, s = math.cos(yaw0), math.sin(yaw0)
        node.get_logger().info(
            f'Vi tri xuat phat: x={x0:.3f} y={y0:.3f} yaw={math.degrees(yaw0):.1f} do')

        waypoints = [(x0 + c * f - s * l, y0 + s * f + c * l) for f, l in args.points]
        for i, ((f, l), (x, y)) in enumerate(zip(args.points, waypoints), start=1):
            node.get_logger().info(f'  WP{i}: toi={f:+.2f} trai={l:+.2f}  ->  {args.frame} x={x:.3f} y={y:.3f}')
        if args.dry_run:
            return 0

        if not node.wait_for(lambda: node.publisher.get_subscription_count() > 0, 5.0):
            node.get_logger().error(f'Khong co node nao nghe {args.waypoint_topic}. Guidance da chay chua?')
            return 1

        for x, y in waypoints:
            msg = PoseStamped()
            msg.header.frame_id = args.frame
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.orientation.w = 1.0
            node.publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.2)
        node.get_logger().info(f'Da gui {len(waypoints)} waypoint.')
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
