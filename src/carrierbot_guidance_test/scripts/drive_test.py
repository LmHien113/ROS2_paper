#!/usr/bin/env python3
"""Cho xe đi thẳng một quãng rồi xoay tại chỗ một góc, dừng theo odometry.

Dùng để hiệu chỉnh odometry (bán kính bánh, khoảng cách hai bánh) và kiểm tra IMU:
xe tự dừng khi odometry báo đủ quãng đường / đủ góc, sau đó so với số đo thật.

Script phát lệnh thẳng lên /cmd_vel. Phải chạy launch với cmd_vel_topic:=/cmd_vel_auto
để guidance không phát lệnh 0 đè lên:
  ros2 launch carrierbot_guidance_test guidance_odom_test.launch.py cmd_vel_topic:=/cmd_vel_auto

Ví dụ:
  drive_test.py                         # thẳng 2 m, rồi xoay trái 360 độ
  drive_test.py --distance 5 --turn 0   # chỉ đi thẳng 5 m
  drive_test.py --distance 0 --turn -360  # chỉ xoay phải 360 độ
  drive_test.py --yaw-source imu        # dừng xoay theo IMU thay vì EKF

Bấm Ctrl+C bất cứ lúc nào để dừng xe.
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


class YawTracker:
    """Cộng dồn góc quay, không bị nhảy khi vượt qua +-180 độ."""

    def __init__(self):
        self.raw = None
        self.total = 0.0
        self.count = 0

    def update(self, yaw):
        if self.raw is not None:
            self.total += wrap(yaw - self.raw)
        self.raw = yaw
        self.count += 1


class DriveTest(Node):

    def __init__(self, cmd_topic):
        super().__init__('drive_test')
        self.cmd_topic = cmd_topic
        self.cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.position = None
        self.last_ekf = None
        self.yaw = {'ekf': YawTracker(), 'odom': YawTracker(), 'imu': YawTracker()}
        self.create_subscription(Odometry, '/odometry/filtered', self.on_ekf, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/odom',
            lambda m: self.yaw['odom'].update(yaw_of(m.pose.pose.orientation)),
            qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/imu/data',
            lambda m: self.yaw['imu'].update(yaw_of(m.orientation)),
            qos_profile_sensor_data)

    def on_ekf(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y)
        self.last_ekf = time.monotonic()
        self.yaw['ekf'].update(yaw_of(msg.pose.pose.orientation))

    def send(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd.publish(msg)

    def stop(self):
        for _ in range(10):
            self.send(0.0, 0.0)
            time.sleep(0.03)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def ekf_fresh(self):
        return self.last_ekf is not None and time.monotonic() - self.last_ekf < 0.5

    def yaw_snapshot(self):
        return {k: t.total for k, t in self.yaw.items()}


def drive_straight(node, distance, speed, min_speed):
    x0, y0 = node.position
    deadline = time.monotonic() + distance / min_speed + 10.0
    period = 0.05
    next_cmd = time.monotonic()
    while True:
        rclpy.spin_once(node, timeout_sec=0.005)
        if not node.ekf_fresh():
            raise RuntimeError('Mat du lieu /odometry/filtered qua 0.5 s')
        if time.monotonic() > deadline:
            raise RuntimeError('Qua thoi gian cho phep khi di thang (xe co bi ket?)')
        travelled = math.hypot(node.position[0] - x0, node.position[1] - y0)
        remaining = distance - travelled
        if remaining <= 0.005:
            return
        if time.monotonic() >= next_cmd:
            next_cmd += period
            # Giảm tốc dần trong đoạn cuối để không trôi quá đích
            node.send(clamp(remaining * 1.0, min_speed, speed), 0.0)


def rotate(node, angle, rate, min_rate, source):
    tracker = node.yaw[source]
    start = tracker.total
    target = abs(angle)
    sign = 1.0 if angle > 0 else -1.0
    t_start = time.monotonic()
    deadline = t_start + target / min_rate + 10.0
    period = 0.05
    next_cmd = t_start
    count0 = tracker.count
    while True:
        rclpy.spin_once(node, timeout_sec=0.005)
        if not node.ekf_fresh():
            raise RuntimeError('Mat du lieu /odometry/filtered qua 0.5 s')
        if time.monotonic() > deadline:
            raise RuntimeError('Qua thoi gian cho phep khi xoay (xe co bi ket?)')
        if time.monotonic() - t_start > 2.0 and tracker.count == count0:
            raise RuntimeError(f'Khong nhan duoc goc tu nguon "{source}"')
        turned = abs(tracker.total - start)
        remaining = target - turned
        if remaining <= math.radians(0.5):
            return
        if time.monotonic() >= next_cmd:
            next_cmd += period
            node.send(0.0, sign * clamp(remaining * 1.0, min_rate, rate))


def main():
    parser = argparse.ArgumentParser(
        description='Di thang mot quang roi xoay tai cho, dung theo odometry.')
    parser.add_argument('--distance', type=float, default=2.0, help='Quang duong di thang (m), 0 = bo qua')
    parser.add_argument('--turn', type=float, default=360.0,
                        help='Goc xoay (do), duong = sang trai, am = sang phai, 0 = bo qua')
    parser.add_argument('--speed', type=float, default=0.2, help='Van toc dai toi da (m/s)')
    parser.add_argument('--min-speed', type=float, default=0.05, help='Van toc dai nho nhat khi giam toc (m/s)')
    parser.add_argument('--rate', type=float, default=0.3, help='Van toc goc toi da (rad/s)')
    parser.add_argument('--min-rate', type=float, default=0.08, help='Van toc goc nho nhat khi giam toc (rad/s)')
    parser.add_argument('--yaw-source', choices=['ekf', 'imu', 'odom'], default='ekf',
                        help='Nguon goc dung de quyet dinh khi nao dung xoay')
    parser.add_argument('--cmd-topic', default='/cmd_vel')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    if args.distance < 0 or args.speed <= 0 or args.min_speed <= 0 or args.rate <= 0 or args.min_rate <= 0:
        parser.error('Quang duong khong duoc am, cac van toc phai > 0')

    rclpy.init()
    node = DriveTest(args.cmd_topic)
    code = 0
    try:
        node.spin_for(1.0)
        if node.position is None:
            node.get_logger().error('Khong nhan duoc /odometry/filtered. Launch da chay chua?')
            return 1
        publishers = node.count_publishers(args.cmd_topic)
        if publishers > 1:
            node.get_logger().error(
                f'Co {publishers - 1} node khac dang phat len {args.cmd_topic} (thuong la guidance). '
                'Hay chay launch voi cmd_vel_topic:=/cmd_vel_auto.')
            return 1
        if node.count_subscribers(args.cmd_topic) == 0:
            node.get_logger().error(f'Khong co bo dieu khien nao nghe {args.cmd_topic}.')
            return 1
        if args.turn != 0 and node.yaw[args.yaw_source].count == 0:
            node.get_logger().error(f'Khong nhan duoc goc tu nguon "{args.yaw_source}", khong the xoay.')
            return 1

        report = []
        if args.distance > 0:
            node.get_logger().info(f'Di thang {args.distance:.2f} m...')
            yaw0 = node.yaw_snapshot()
            x0, y0 = node.position
            drive_straight(node, args.distance, args.speed, args.min_speed)
            node.stop()
            node.spin_for(1.5)   # chờ xe dừng hẳn
            d = math.hypot(node.position[0] - x0, node.position[1] - y0)
            yaw1 = node.yaw_snapshot()
            report.append(f'Di thang: odometry bao {d:.3f} m (muc tieu {args.distance:.2f} m)')
            report.append('  Goc lech trong luc di thang (do): ' + ', '.join(
                f'{k} {math.degrees(yaw1[k] - yaw0[k]):+.2f}' for k in ('ekf', 'imu', 'odom')
                if node.yaw[k].count))

        if args.turn != 0:
            node.get_logger().info(f'Xoay {args.turn:+.0f} do theo {args.yaw_source}...')
            yaw0 = node.yaw_snapshot()
            rotate(node, math.radians(args.turn), args.rate, args.min_rate, args.yaw_source)
            node.stop()
            node.spin_for(1.5)
            yaw1 = node.yaw_snapshot()
            report.append(f'Xoay tai cho (muc tieu {args.turn:+.0f} do, dung theo {args.yaw_source}):')
            for k, label in (('ekf', 'EKF /odometry/filtered'), ('imu', 'IMU /imu/data'),
                             ('odom', 'Banh xe /odom')):
                if node.yaw[k].count:
                    report.append(f'  {label:<24} {math.degrees(yaw1[k] - yaw0[k]):+8.2f} do')
                else:
                    report.append(f'  {label:<24} khong co du lieu')

        print('\n'.join(['', '===== KET QUA ====='] + report + [
            '',
            'So voi thuc te: do quang duong bang thuoc va goc xoay that.',
            '  wheel_radius moi     = wheel_radius cu     x (quang duong that / quang duong odom)',
            '  wheel_separation moi = wheel_separation cu x (goc banh xe /odom / goc xoay that)',
        ]))
    except KeyboardInterrupt:
        print('\nDung theo yeu cau (Ctrl+C).')
        code = 130
    except RuntimeError as e:
        node.get_logger().error(str(e))
        code = 1
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
