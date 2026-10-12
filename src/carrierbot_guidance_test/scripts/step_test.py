#!/usr/bin/env python3
"""Thử đáp ứng bậc kênh quay để nhận dạng K, L, tau và đề xuất KP, KI, KD.

Xe chạy thẳng với vận tốc không đổi, script đổi lệnh vận tốc góc theo từng bậc
(0 -> +A -> -A -> 0 cho mỗi biên độ A), ghi lệnh và tốc độ quay đo được
(gyro IMU, odometry bánh xe) ra CSV, rồi fit mô hình bậc nhất có trễ:

    omega_do(s) / omega_lenh(s) = K * exp(-L*s) / (tau*s + 1)

Script phát lệnh thẳng lên /cmd_vel. Phải chạy launch với cmd_vel_topic:=/cmd_vel_auto:
  ros2 launch carrierbot_guidance_test guidance_odom_test.launch.py cmd_vel_topic:=/cmd_vel_auto

Ví dụ:
  step_test.py                          # mặc định: 0.2 m/s, biên độ 0.15 và 0.25 rad/s
  step_test.py --amplitudes 0.1 0.2 0.3 # thêm biên độ
  step_test.py --speed 0                # đổi bậc khi xoay tại chỗ
  step_test.py --analyze step_test_XXXX.csv   # chỉ phân tích lại một file đã ghi

Bấm Ctrl+C bất cứ lúc nào để dừng xe.
"""

import argparse
import csv
import math
import os
import sys
import time

FILTER_ALPHA = 0.2      # bộ lọc lệnh omega trong guidance: w = 0.2*u + 0.8*w_cu
GUIDANCE_PERIOD = 0.1   # chu kỳ guidance (s)


# ---------------------------------------------------------------- phân tích

def fit_fopdt(samples, du):
    """Fit y(t) = K*du*(1 - exp(-(t-L)/tau)) cho t > L. samples = [(t, y)] tính từ lúc đổi bậc."""
    if len(samples) < 8 or abs(du) < 1e-6:
        return None
    t_end = samples[-1][0]
    best = None
    L = 0.0
    while L <= min(1.5, t_end - 0.3):
        tau = 0.02
        while tau <= 1.5:
            sbb = sby = 0.0
            basis = []
            for t, y in samples:
                b = du * (1.0 - math.exp(-(t - L) / tau)) if t > L else 0.0
                basis.append(b)
                sbb += b * b
                sby += b * y
            if sbb > 1e-12:
                k = sby / sbb
                sse = sum((y - k * b) ** 2 for (_, y), b in zip(samples, basis))
                if best is None or sse < best[0]:
                    best = (sse, k, L, tau)
            tau += 0.02
        L += 0.02
    if best is None:
        return None
    sse, k, L, tau = best
    mean = sum(y for _, y in samples) / len(samples)
    sst = sum((y - mean) ** 2 for _, y in samples)
    r2 = 1.0 - sse / sst if sst > 1e-12 else 0.0
    return k, L, tau, r2


def median(values):
    v = sorted(values)
    n = len(v)
    return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])


def recommend_pid(K, L, tau, extra_delay=0.0):
    """SIMC cho vòng hướng: K*e^{-Ls}/(s(tau*s+1)) nối tiếp bộ lọc lệnh của guidance."""
    tau_f = GUIDANCE_PERIOD * (1.0 - FILTER_ALPHA) / FILTER_ALPHA   # Backward Euler: 0.4 s
    lag_big, lag_small = max(tau, tau_f), min(tau, tau_f)
    tau1 = lag_big + lag_small / 2.0                    # quy tắc "half rule" của Skogestad
    # + nửa chu kỳ lấy mẫu + độ trễ của nguồn tư thế (vd. AMCL)
    theta = L + lag_small / 2.0 + GUIDANCE_PERIOD / 2.0 + extra_delay
    out = []
    for name, tau_c in (('Can bang (tau_c = theta)', theta), ('Than trong (tau_c = 2*theta)', 2 * theta)):
        kc = 1.0 / (K * (tau_c + theta))
        ti = 4.0 * (tau_c + theta)
        td = tau1
        out.append((name, kc * (1.0 + td / ti), kc / ti, kc * td))
    return tau_f, tau1, theta, out


def analyze(path, extra_delay=0.0):
    events = {'cmd_w': [], 'imu_wz': [], 'odom_wz': []}
    steps = []
    with open(path) as f:
        for row in csv.reader(f):
            if not row or row[0].startswith('#') or row[0] == 't':
                continue
            t, kind, value = float(row[0]), row[1], float(row[2])
            if kind == 'step':
                steps.append((t, float(row[3]), value))   # (t0, truoc, sau)
            elif kind in events:
                events[kind].append((t, value))

    if not steps:
        print('Khong tim thay buoc nhay nao trong file.')
        return 1
    print(f'File: {path}')
    print(f'{len(steps)} buoc nhay | mau IMU: {len(events["imu_wz"])} | mau odom: {len(events["odom_wz"])}')
    results = {'imu_wz': [], 'odom_wz': []}
    print(f'\n{"Buoc":<6}{"Lenh (rad/s)":<18}{"Nguon":<9}{"K":>7}{"L (s)":>8}{"tau (s)":>9}{"R2":>7}')
    for i, (t0, before, after) in enumerate(steps):
        t_next = steps[i + 1][0] if i + 1 < len(steps) else t0 + 3.0
        for src in ('imu_wz', 'odom_wz'):
            pre = [y for t, y in events[src] if t0 - 0.5 <= t < t0]
            post = [(t - t0, y) for t, y in events[src] if t0 <= t < t_next]
            if len(pre) < 2 or len(post) < 8:
                continue
            y0 = sum(pre) / len(pre)
            fit = fit_fopdt([(t, y - y0) for t, y in post], after - before)
            if fit is None:
                continue
            k, L, tau, r2 = fit
            label = 'IMU' if src == 'imu_wz' else 'Banh xe'
            print(f'{i + 1:<6}{before:+.2f} -> {after:+.2f}{"":<4}{label:<9}{k:>7.2f}{L:>8.2f}{tau:>9.2f}{r2:>7.2f}')
            if r2 >= 0.6:
                results[src].append((k, L, tau))

    print('\nTrung vi (chi lay cac buoc co R2 >= 0.6):')
    summary = {}
    for src, label in (('imu_wz', 'IMU (than xe quay that)'), ('odom_wz', 'Banh xe (/odom)')):
        r = results[src]
        if r:
            summary[src] = tuple(median([x[j] for x in r]) for j in range(3))
            k, L, tau = summary[src]
            print(f'  {label:<26} K = {k:.2f}   L = {L:.2f} s   tau = {tau:.2f} s   ({len(r)} buoc)')
        else:
            print(f'  {label:<26} khong du du lieu tot')

    src = 'imu_wz' if 'imu_wz' in summary else ('odom_wz' if 'odom_wz' in summary else None)
    if src is None:
        print('\nKhong du du lieu de de xuat PID.')
        return 1
    K, L, tau = summary[src]
    tau_f, tau1, theta, pids = recommend_pid(K, L, tau, extra_delay)
    print(f'\nDe xuat PID cho guidance (dung so lieu {"IMU" if src == "imu_wz" else "banh xe"}):')
    print(f'  Mo hinh vong huong: K*e^(-{L:.2f}s) / (s*({tau:.2f}s+1)) noi tiep bo loc lenh tau_f = {tau_f:.2f} s')
    print(f'  Quy doi: tau1 = {tau1:.2f} s, tre hieu dung theta = {theta:.2f} s'
          + (f' (da cong {extra_delay:.2f} s tre cua nguon tu the)' if extra_delay else ''))
    for name, kp, ki, kd in pids:
        print(f'  {name:<30} kp:={kp:.3f} ki:={ki:.4f} kd:={kd:.3f}')
    print('  Bat dau voi bo "Than trong", roi chinh tiep bang bai thu A, B, C.')
    return 0


# ---------------------------------------------------------------- chạy thử

def run(args):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    class StepTest(Node):

        def __init__(self, cmd_topic):
            super().__init__('step_test')
            self.cmd = self.create_publisher(Twist, cmd_topic, 10)
            self.t0 = time.monotonic()
            self.rows = []
            self.got = {'imu': 0, 'odom': 0, 'ekf': 0}
            self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
            self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
            self.create_subscription(Odometry, '/odometry/filtered', self.on_ekf, qos_profile_sensor_data)

        def now(self):
            return time.monotonic() - self.t0

        def on_imu(self, m):
            self.got['imu'] += 1
            self.rows.append((self.now(), 'imu_wz', m.angular_velocity.z))

        def on_odom(self, m):
            self.got['odom'] += 1
            self.rows.append((self.now(), 'odom_wz', m.twist.twist.angular.z))

        def on_ekf(self, m):
            self.got['ekf'] += 1
            self.last_ekf = time.monotonic()

        def send(self, v, w):
            msg = Twist()
            msg.linear.x = float(v)
            msg.angular.z = float(w)
            self.cmd.publish(msg)
            self.rows.append((self.now(), 'cmd_w', float(w)))

        def stop(self):
            for _ in range(10):
                self.send(0.0, 0.0)
                time.sleep(0.03)

    # Kế hoạch: (thời gian giữ, lệnh omega)
    plan = [(args.lead, 0.0)]
    for a in args.amplitudes:
        plan += [(args.hold, a), (args.hold, -a), (args.hold, 0.0)]
    total = sum(h for h, _ in plan)
    print(f'Ke hoach: v = {args.speed:.2f} m/s, bien do {args.amplitudes} rad/s, giu moi muc {args.hold:.1f} s')
    print(f'Tong thoi gian {total:.0f} s, quang duong khoang {args.speed * total:.1f} m '
          f'(lech ngang co the toi vai chuc cm). Can khoang trong phia truoc it nhat {args.speed * total + 1:.1f} m.')
    if not args.yes:
        input('Giu san E-stop. Nhan Enter de bat dau, Ctrl+C de huy... ')

    rclpy.init()
    node = StepTest(args.cmd_topic)
    node.last_ekf = None
    code = 0
    try:
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
        if node.count_publishers(args.cmd_topic) > 1:
            node.get_logger().error(
                f'Co node khac dang phat len {args.cmd_topic} (thuong la guidance). '
                'Hay chay launch voi cmd_vel_topic:=/cmd_vel_auto.')
            return 1
        if node.count_subscribers(args.cmd_topic) == 0:
            node.get_logger().error(f'Khong co bo dieu khien nao nghe {args.cmd_topic}.')
            return 1
        if node.got['imu'] == 0 and node.got['odom'] == 0:
            node.get_logger().error('Khong nhan duoc /imu/data va /odom. Launch da chay chua?')
            return 1

        period = 1.0 / args.rate_hz
        current_w = 0.0
        for hold, w in plan:
            if w != current_w:
                node.rows.append((node.now(), 'step', w, current_w))
                node.get_logger().info(f'omega: {current_w:+.2f} -> {w:+.2f} rad/s')
                current_w = w
            seg_end = time.monotonic() + hold
            next_cmd = time.monotonic()
            while time.monotonic() < seg_end:
                rclpy.spin_once(node, timeout_sec=0.002)
                if time.monotonic() >= next_cmd:
                    next_cmd += period
                    node.send(args.speed, w)
                if node.last_ekf is not None and time.monotonic() - node.last_ekf > 0.5:
                    raise RuntimeError('Mat du lieu /odometry/filtered qua 0.5 s')
        node.stop()
    except KeyboardInterrupt:
        print('\nDung theo yeu cau (Ctrl+C).')
        code = 130
    except RuntimeError as e:
        node.get_logger().error(str(e))
        code = 1
    finally:
        node.stop()
        rows = node.rows
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    name = os.path.join(args.out_dir, time.strftime('step_test_%Y%m%d_%H%M%S.csv'))
    with open(name, 'w', newline='') as f:
        f.write(f'# speed={args.speed} amplitudes={args.amplitudes} hold={args.hold} rate_hz={args.rate_hz}\n')
        w = csv.writer(f)
        w.writerow(['t', 'kind', 'value', 'prev'])
        for r in rows:
            w.writerow([f'{r[0]:.4f}', r[1], f'{r[2]:.5f}'] + ([f'{r[3]:.5f}'] if len(r) > 3 else []))
    print(f'\nDa luu du lieu vao {name}\n')
    if code == 0:
        return analyze(name, args.extra_delay)
    return code


def default_data_dir():
    """Thu muc data/ cua package trong workspace (giong guidance_test_node); khong co thi thu muc hien tai."""
    prefix = os.environ.get('COLCON_PREFIX_PATH', '').split(':')[0].rstrip('/')
    if prefix.endswith('/install'):
        path = os.path.join(prefix[:-len('/install')], 'src', 'carrierbot_guidance_test', 'data')
        if os.path.isdir(path):
            return path
    return '.'


def main():
    parser = argparse.ArgumentParser(description='Thu dap ung bac kenh quay, nhan dang K, L, tau.')
    parser.add_argument('--analyze', metavar='CSV', help='Chi phan tich file da ghi, khong chay xe')
    parser.add_argument('--speed', type=float, default=0.2, help='Van toc dai khi thu (m/s), 0 = xoay tai cho')
    parser.add_argument('--amplitudes', type=float, nargs='+', default=[0.15, 0.25],
                        help='Cac bien do omega (rad/s)')
    parser.add_argument('--hold', type=float, default=2.5, help='Thoi gian giu moi muc lenh (s)')
    parser.add_argument('--lead', type=float, default=3.0, help='Thoi gian chay thang truoc buoc dau tien (s)')
    parser.add_argument('--rate-hz', type=float, default=50.0, help='Tan so gui lenh (Hz)')
    parser.add_argument('--extra-delay', type=float, default=0.0,
                        help='Tre them cua nguon tu the khi tinh PID (s), vd. 0.3 cho AMCL')
    parser.add_argument('--cmd-topic', default='/cmd_vel')
    parser.add_argument('--out-dir', default=default_data_dir(),
                        help='Thu muc luu CSV (mac dinh: src/carrierbot_guidance_test/data)')
    parser.add_argument('--yes', action='store_true', help='Khong hoi xac nhan truoc khi chay')
    argv = sys.argv[1:]
    try:
        from rclpy.utilities import remove_ros_args
        argv = remove_ros_args(sys.argv)[1:]
    except ImportError:
        pass
    args = parser.parse_args(argv)
    if args.analyze:
        return analyze(args.analyze, args.extra_delay)
    if args.speed < 0 or args.hold <= 0 or args.lead < 0 or not args.amplitudes or \
            any(a <= 0 or a > 0.6 for a in args.amplitudes):
        parser.error('Toc do khong am, hold > 0, bien do trong khoang (0, 0.6] rad/s')
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
