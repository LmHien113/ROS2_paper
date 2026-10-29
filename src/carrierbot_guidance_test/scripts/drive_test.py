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
  drive_test.py --distance 2 --turn 0 --csv straight_2m.csv

CSV ghi từng mẫu EKF, odom, IMU nhận được và từng lệnh, cả khi chờ và khi dừng.
Mặc định tạo drive_test_<timestamp>.csv trong thư mục chạy lệnh; --csv chọn tên khác.
File đã có sẽ không bị ghi đè. Dữ liệu được flush mỗi giây và đóng khi kết thúc.

Các cột chính:
  source: ekf / odom / imu / cmd; một dòng là một mẫu, chưa đồng bộ các nguồn.
  stamp_ns: timestamp header của mẫu; received_ros_ns: thời điểm node nhận mẫu.
  elapsed_s / segment_elapsed_s: thời gian monotonic của node / đoạn chạy.
  phase: waiting / straight / settling_straight / turn / settling_turn / stopping.
  yaw_deg / delta_yaw_deg: yaw gốc / góc cộng dồn so với đầu đoạn (độ).
  x_m, y_m, vx_m_s, wz_rad_s: pose và vận tốc của đúng nguồn.
  forward_m / left_m: dịch chuyển theo hướng đầu đoạn của từng nguồn odom/EKF.
  yaw_variance_rad2: phương sai yaw do nguồn báo, không phải sai số thực tế.
  cmd_v_m_s / cmd_w_rad_s: lệnh gần nhất do script phát, không phải feedback.

Yaw IMU là trong frame cảm biến; script không biến đổi TF. Kiểm tra frame và
đồng bộ timestamp trước khi so sánh. Nguồn đến muộn lấy mẫu đầu làm mốc riêng.
Ô thiếu dữ liệu để trống; IMU không cung cấp vị trí. Cần đo pose thực tế độc lập.
Chỉ đi thẳng phải truyền --turn 0 (mặc định cũ vẫn xoay 360 độ sau khi đi thẳng).

Bấm Ctrl+C bất cứ lúc nào để dừng xe.
"""

import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
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

    def __init__(self, cmd_topic, csv_path=None):
        super().__init__('drive_test')
        self.cmd_topic = cmd_topic
        self.cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.position = None
        self.last_ekf = None
        self.yaw = {'ekf': YawTracker(), 'odom': YawTracker(), 'imu': YawTracker()}
        self.phase = 'waiting'
        self.segment = ''
        self.segment_start = None
        self.segment_position = None
        self.segment_heading = None
        self.segment_yaw = {}
        self.latest_pose = {}
        self.segment_pose = {}
        self.command = (0.0, 0.0)
        self.log_start = time.monotonic()
        self.last_flush = self.log_start
        self.log_file = None
        self.csv_writer = None
        if csv_path is not None:
            path = Path(csv_path).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            # Khong ghi de mot lan do cu.
            self.log_file = path.open('x', newline='', encoding='utf-8')
            self.csv_writer = csv.DictWriter(self.log_file, fieldnames=[
                'elapsed_s', 'segment_elapsed_s', 'phase', 'segment', 'source',
                'stamp_ns', 'received_ros_ns', 'frame_id', 'child_frame_id',
                'sample_count', 'x_m', 'y_m', 'yaw_rad', 'yaw_deg',
                'delta_yaw_deg', 'yaw_variance_rad2', 'vx_m_s', 'wz_rad_s',
                'forward_ekf_m', 'left_ekf_m', 'cmd_v_m_s', 'cmd_w_rad_s',
                'forward_m', 'left_m', 'displacement_m',
            ])
            self.csv_writer.writeheader()
            self.log_file.flush()
            self.get_logger().info(f'CSV: {path.resolve()}')
        self.create_subscription(Odometry, '/odometry/filtered', self.on_ekf, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/odom',
            self.on_odom,
            qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/imu/data',
            self.on_imu,
            qos_profile_sensor_data)

    def on_ekf(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y)
        self.last_ekf = time.monotonic()
        self.yaw['ekf'].update(yaw_of(msg.pose.pose.orientation))
        self.latest_pose['ekf'] = (p.x, p.y, self.yaw['ekf'].raw)
        self.log_sample('ekf', msg)

    def on_odom(self, msg):
        self.yaw['odom'].update(yaw_of(msg.pose.pose.orientation))
        p = msg.pose.pose.position
        self.latest_pose['odom'] = (p.x, p.y, self.yaw['odom'].raw)
        self.log_sample('odom', msg)

    def on_imu(self, msg):
        # sensor_msgs/Imu: -1 nghia la khong co uoc luong orientation.
        if msg.orientation_covariance[0] != -1:
            self.yaw['imu'].update(yaw_of(msg.orientation))
        self.log_sample('imu', msg)

    def begin_segment(self, name):
        self.phase = name
        self.segment = name
        self.segment_start = time.monotonic()
        self.segment_position = self.position
        self.segment_heading = self.yaw['ekf'].raw
        self.segment_yaw = {
            k: t.total if t.count else None for k, t in self.yaw.items()
        }
        self.segment_pose = self.latest_pose.copy()

    def log_sample(self, source, msg=None):
        if self.csv_writer is None:
            return
        now = time.monotonic()
        row = {
            'elapsed_s': now - self.log_start,
            'segment_elapsed_s': '' if self.segment_start is None else now - self.segment_start,
            'phase': self.phase, 'segment': self.segment, 'source': source,
            'received_ros_ns': self.get_clock().now().nanoseconds,
            'cmd_v_m_s': self.command[0], 'cmd_w_rad_s': self.command[1],
        }
        if msg is not None:
            row['stamp_ns'] = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            row['frame_id'] = msg.header.frame_id
            tracker = self.yaw[source]
            valid_yaw = source != 'imu' or msg.orientation_covariance[0] != -1
            row['sample_count'] = tracker.count
            if valid_yaw:
                row['yaw_rad'] = tracker.raw
                row['yaw_deg'] = math.degrees(tracker.raw)
                baseline = self.segment_yaw.get(source)
                if self.segment and baseline is None:
                    # Nguon den muon: moc la mau dau cua nguon do, khong phai 0 tuyet doi.
                    baseline = tracker.total
                    self.segment_yaw[source] = baseline
                if baseline is not None:
                    row['delta_yaw_deg'] = math.degrees(tracker.total - baseline)
                row['yaw_variance_rad2'] = (
                    msg.orientation_covariance[8] if source == 'imu' else msg.pose.covariance[35])
            if source in ('ekf', 'odom'):
                p = msg.pose.pose.position
                row.update(x_m=p.x, y_m=p.y, child_frame_id=msg.child_frame_id,
                           vx_m_s=msg.twist.twist.linear.x, wz_rad_s=msg.twist.twist.angular.z)
                if self.segment:
                    if source not in self.segment_pose:
                        self.segment_pose[source] = self.latest_pose[source]
                    x0, y0, heading0 = self.segment_pose[source]
                    dx, dy = p.x - x0, p.y - y0
                    c, s = math.cos(heading0), math.sin(heading0)
                    row['forward_m'] = c * dx + s * dy
                    row['left_m'] = -s * dx + c * dy
                    row['displacement_m'] = math.hypot(dx, dy)
                if source == 'ekf' and self.segment_position is not None:
                    dx = p.x - self.segment_position[0]
                    dy = p.y - self.segment_position[1]
                    c, s = math.cos(self.segment_heading), math.sin(self.segment_heading)
                    row['forward_ekf_m'] = c * dx + s * dy
                    row['left_ekf_m'] = -s * dx + c * dy
            elif msg.angular_velocity_covariance[0] != -1:
                row['wz_rad_s'] = msg.angular_velocity.z
        try:
            self.csv_writer.writerow(row)
            if now - self.last_flush >= 1.0:
                self.log_file.flush()
                self.last_flush = now
        except OSError as error:
            # Cho phep finally phat lenh dung ngay ca khi o dia ghi log bi loi.
            self.csv_writer = None
            raise RuntimeError(f'Khong ghi duoc CSV: {error}') from error

    def close_log(self):
        if self.log_file is not None:
            try:
                self.log_file.close()
            finally:
                self.log_file = None
                self.csv_writer = None

    def send(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd.publish(msg)
        self.command = (float(v), float(w))
        self.log_sample('cmd')

    def stop(self):
        for _ in range(10):
            if not rclpy.ok():
                break
            self.send(0.0, 0.0)
            self.spin_for(0.03)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def ekf_fresh(self):
        return self.last_ekf is not None and time.monotonic() - self.last_ekf < 0.5

    def yaw_snapshot(self):
        return {k: t.total for k, t in self.yaw.items()}


def yaw_changes(node, start):
    """Góc đã quay (độ) theo từng nguồn kể từ snapshot `start`."""
    return {k: math.degrees(node.yaw[k].total - start[k]) for k in node.yaw if node.yaw[k].count}


def describe(changes):
    names = {'ekf': 'EKF', 'imu': 'IMU', 'odom': 'banh xe'}
    return ', '.join(f'{names[k]} {v:+.0f} do' for k, v in changes.items())


def drive_straight(node, distance, speed, min_speed, max_yaw_deg):
    x0, y0 = node.position
    yaw0 = node.yaw_snapshot()
    deadline = time.monotonic() + 2.0 * distance / speed + 8.0
    period = 0.05
    next_cmd = time.monotonic()
    while True:
        rclpy.spin_once(node, timeout_sec=0.005)
        if not node.ekf_fresh():
            raise RuntimeError('Mat du lieu /odometry/filtered qua 0.5 s')
        if time.monotonic() > deadline:
            raise RuntimeError(
                'Qua thoi gian cho phep khi di thang: odometry khong ghi nhan xe tien len. '
                f'Goc da quay: {describe(yaw_changes(node, yaw0))}')
        # An toàn: lệnh là đi thẳng mà xe lại quay -> dừng ngay
        changes = yaw_changes(node, yaw0)
        if any(abs(v) > max_yaw_deg for v in changes.values()):
            raise RuntimeError(
                f'Lenh la DI THANG nhung xe dang QUAY ({describe(changes)}). '
                'Kiem tra chieu quay banh xe va dau cua IMU.')
        travelled = math.hypot(node.position[0] - x0, node.position[1] - y0)
        remaining = distance - travelled
        if remaining <= 0.005:
            return
        if time.monotonic() >= next_cmd:
            next_cmd += period
            # Giảm tốc dần trong đoạn cuối để không trôi quá đích
            node.send(clamp(remaining * 1.0, min_speed, speed), 0.0)


def rotate(node, angle, rate, min_rate, source, max_shift_m):
    tracker = node.yaw[source]
    yaw0 = node.yaw_snapshot()
    start = tracker.total
    x0, y0 = node.position
    target = abs(angle)
    sign = 1.0 if angle > 0 else -1.0
    t_start = time.monotonic()
    deadline = t_start + 2.0 * target / rate + 8.0
    period = 0.05
    next_cmd = t_start
    count0 = tracker.count
    while True:
        rclpy.spin_once(node, timeout_sec=0.005)
        if not node.ekf_fresh():
            raise RuntimeError('Mat du lieu /odometry/filtered qua 0.5 s')
        changes = yaw_changes(node, yaw0)
        if time.monotonic() > deadline:
            raise RuntimeError(f'Qua thoi gian cho phep khi xoay. Goc da quay: {describe(changes)}')
        if time.monotonic() - t_start > 2.0 and tracker.count == count0:
            raise RuntimeError(f'Khong nhan duoc goc tu nguon "{source}"')
        # An toàn: các nguồn góc lệch nhau quá 45 độ, hoặc một nguồn báo đã quay quá đích nhiều
        spread = max(changes.values()) - min(changes.values()) if changes else 0.0
        if spread > 45.0 or any(abs(v) > math.degrees(target) + 45.0 for v in changes.values()):
            raise RuntimeError(
                f'Cac nguon goc mau thuan nhau ({describe(changes)}). '
                'Kiem tra dau cua IMU va odometry.')
        # An toàn: lệnh là xoay tại chỗ mà xe lại chạy đi -> dừng ngay
        shift = math.hypot(node.position[0] - x0, node.position[1] - y0)
        if shift > max_shift_m:
            raise RuntimeError(
                f'Lenh la XOAY TAI CHO nhung odometry bao xe da di {shift:.2f} m. '
                'Kiem tra chieu quay banh xe.')
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
    parser.add_argument('--max-yaw-straight', type=float, default=20.0,
                        help='Dung khan cap neu xe quay qua goc nay (do) trong luc di thang')
    parser.add_argument('--cmd-topic', default='/cmd_vel')
    parser.add_argument('--csv', default=None,
                        help='File CSV moi; mac dinh drive_test_<timestamp>.csv trong thu muc hien tai')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    if args.distance < 0 or args.speed <= 0 or args.min_speed <= 0 or args.rate <= 0 or args.min_rate <= 0:
        parser.error('Quang duong khong duoc am, cac van toc phai > 0')

    rclpy.init()
    csv_path = args.csv or datetime.now().strftime('drive_test_%Y%m%d_%H%M%S_%f.csv')
    try:
        node = DriveTest(args.cmd_topic, csv_path)
    except OSError as error:
        print(f'Khong tao duoc CSV "{csv_path}": {error}', file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return 1
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
            node.begin_segment('straight')
            node.get_logger().info(f'Di thang {args.distance:.2f} m...')
            yaw0 = node.yaw_snapshot()
            x0, y0 = node.position
            drive_straight(node, args.distance, args.speed, args.min_speed, args.max_yaw_straight)
            node.phase = 'settling_straight'
            node.stop()
            node.spin_for(1.5)   # chờ xe dừng hẳn
            d = math.hypot(node.position[0] - x0, node.position[1] - y0)
            yaw1 = node.yaw_snapshot()
            report.append(f'Di thang: odometry bao {d:.3f} m (muc tieu {args.distance:.2f} m)')
            report.append('  Goc lech trong luc di thang (do): ' + ', '.join(
                f'{k} {math.degrees(yaw1[k] - yaw0[k]):+.2f}' for k in ('ekf', 'imu', 'odom')
                if node.yaw[k].count))

        if args.turn != 0:
            node.begin_segment('turn')
            node.get_logger().info(f'Xoay {args.turn:+.0f} do theo {args.yaw_source}...')
            yaw0 = node.yaw_snapshot()
            rotate(node, math.radians(args.turn), args.rate, args.min_rate, args.yaw_source, 0.3)
            node.phase = 'settling_turn'
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
            f'CSV: {Path(csv_path).expanduser().resolve()}',
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
        node.phase = 'stopping'
        try:
            node.stop()
        finally:
            try:
                node.close_log()
            except OSError as error:
                print(f'Khong dong duoc CSV: {error}', file=sys.stderr)
                code = 1
            finally:
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())

