"""Thử Guidance (LOS + PID) trên xe thật, KHÔNG dùng MQTT, AMCL, lidar, SLAM.

Chạy:  phần cứng + controller + IMU + EKF + odom_to_pose + guidance_test_node.
Tư thế cho guidance lấy từ EKF (/odometry/filtered, khung odom) qua odom_to_pose.
Waypoint gửi bằng tay:  ros2 run carrierbot_guidance_test send_route.py 4,0 4,4

CẢNH BÁO: không có lidar nên KHÔNG có dừng tránh vật cản. Bắt buộc có người cầm
nút dừng khẩn cấp (E-stop) và khu vực thử phải trống.

Ví dụ:
  ros2 launch carrierbot_guidance_test guidance_odom_test.launch.py
  ros2 launch carrierbot_guidance_test guidance_odom_test.launch.py kp:=0.44 ki:=0.043 kd:=0.19
  # Giả lập tần số cập nhật của AMCL (2 Hz) để so sánh:
  ros2 launch carrierbot_guidance_test guidance_odom_test.launch.py pose_rate_hz:=2.0
  # Khi đã có node deadman cho tay cầm:
  ros2 launch carrierbot_guidance_test guidance_odom_test.launch.py cmd_vel_topic:=/cmd_vel_auto
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    def arg(name, default, description):
        return DeclareLaunchArgument(name, default_value=default, description=description)

    def number(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    args = [
        arg('use_imu', 'true', 'Chay driver IMU BNO055 (EKF dung yaw tu IMU)'),
        # STM32: 0x020 byte dau = 2 la che do Auto (nhan van toc tu ROS qua 0x030). Gia tri khac se lai sai.
        arg('drive_mode', '2', 'Che do lai gui qua CAN 0x020: 2 = Auto (dieu khien tu ROS)'),
        arg('cmd_vel_topic', '/cmd_vel',
            'Topic lenh van toc cua guidance. Doi thanh /cmd_vel_auto khi co node deadman'),
        arg('pose_rate_hz', '0.0', 'Tan so tu the cho guidance. 0 = ~30 Hz cua EKF, 2.0 = gia lap AMCL'),
        arg('kp', '0.5', 'He so P'),
        arg('ki', '0.0', 'He so I'),
        arg('kd', '0.7', 'He so D'),
        arg('integral_zone', '0.15', 'Chi tich phan khi |sai so huong| nho hon nguong nay (rad)'),
        arg('integral_max_output', '0.3', 'Gioi han dong gop cua khau I (rad/s)'),
        arg('speed', '0.3', 'Van toc dai toi da khi thu (m/s)'),
        arg('min_speed', '0.2', 'Van toc dai toi thieu (m/s), phai <= speed'),
        arg('angular_speed_max', '0.5', 'Gioi han van toc goc (rad/s)'),
        arg('delta_min', '0.5', 'Khoang nhin truoc nho nhat (m)'),
        arg('delta_max', '0.8', 'Khoang nhin truoc lon nhat (m)'),
        arg('can_port', next(iter(sorted(glob.glob('/dev/serial/by-id/usb-1a86_USB_Serial-*'))), '/dev/usbcan'),
            'Cong serial cua bo chuyen USB-CAN'),
        # Dau tung banh (+1/-1) cua hardware interface thu nghiem. Kiem tra bang drive_test khi ke banh.
        # STM32 hien tai: ca hai banh duong = tien (thu ngay 12/10/2026), nen lenh de +1, +1.
        arg('left_command_sign', '1', 'Dau lenh banh trai gui xuong STM32 (0x030)'),
        arg('right_command_sign', '1', 'Dau lenh banh phai gui xuong STM32 (0x030)'),
        arg('left_feedback_sign', '-1', 'Dau encoder banh trai doc ve (0x011)'),
        arg('right_feedback_sign', '1', 'Dau encoder banh phai doc ve (0x011)'),
        arg('imu_mode', '12', 'Che do BNO055: 12 = NDOF (co la ban), 8 = IMU (khong la ban)'),
    ]

    # Phan cung: giong carrierbot_firmware/launch/hardware_interface.launch.py, nhung dung
    # mo ta robot va plugin thu nghiem cua package nay (dau tung banh cau hinh duoc).
    robot_description = ParameterValue(Command([
        'xacro ',
        os.path.join(get_package_share_directory('carrierbot_guidance_test'), 'urdf', 'carrierbot_test.urdf.xacro'),
        ' drive_mode:=', LaunchConfiguration('drive_mode'),
        ' can_port:=', LaunchConfiguration('can_port'),
        ' left_command_sign:=', LaunchConfiguration('left_command_sign'),
        ' right_command_sign:=', LaunchConfiguration('right_command_sign'),
        ' left_feedback_sign:=', LaunchConfiguration('left_feedback_sign'),
        ' right_feedback_sign:=', LaunchConfiguration('right_feedback_sign'),
    ]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': False},
            os.path.join(get_package_share_directory('carrierbot_controller'), 'config', 'controller.yaml'),
        ],
        remappings=[('/carrierbot_controller/cmd_vel_unstamped', '/cmd_vel')],
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('carrierbot_controller'), 'launch', 'controller.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    # Giống real_robot.launch.py
    imu = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        parameters=[
            {'connection_type': 'i2c'},
            {'i2c_bus': 1},
            {'i2c_address': 40},
            {'frame_id': 'imu'},
            {'operation_mode': ParameterValue(LaunchConfiguration('imu_mode'), value_type=int)},
        ],
        remappings=[('/bno055/imu', '/imu/data')],
        condition=IfCondition(LaunchConfiguration('use_imu')),
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('carrierbot_slam'), 'config', 'ekf.yaml')],
    )

    odom_to_pose = Node(
        package='carrierbot_guidance_test',
        executable='odom_to_pose.py',
        name='odom_to_pose',
        output='screen',
        parameters=[{
            'input_topic': '/odometry/filtered',
            'output_topic': '/guidance_pose',
            'rate_hz': number('pose_rate_hz'),
        }],
    )

    guidance = Node(
        package='carrierbot_guidance_test',
        executable='guidance_test_node',
        name='guidance_test_node',
        output='screen',
        parameters=[{
            'pose_topic': '/guidance_pose',
            # Không nhận /initialpose từ RViz (khung map) để khỏi làm lệch tư thế khung odom.
            'initial_pose_topic': '/guidance_initialpose_unused',
            # Không có lidar.
            'safety_enabled': False,
            'KP': number('kp'),
            'KI': number('ki'),
            'KD': number('kd'),
            'integral_zone': number('integral_zone'),
            'integral_max_output': number('integral_max_output'),
            'linear_speed': number('speed'),
            'linear_speed_max': number('speed'),
            'min_speed_linear': number('min_speed'),
            'corner_max_speed': number('min_speed'),
            'angular_speed_max': number('angular_speed_max'),
            'delta_min': number('delta_min'),
            'delta_max': number('delta_max'),
        }],
        remappings=[('/cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
    )

    return LaunchDescription(args + [
        LogInfo(msg='guidance_odom_test: KHONG co lidar, KHONG dung tranh vat can. Giu san nut E-stop.'),
        LogInfo(msg=['Hardware TEST: drive_mode=', LaunchConfiguration('drive_mode'),
                     ' sign cmd L/R=', LaunchConfiguration('left_command_sign'), '/',
                     LaunchConfiguration('right_command_sign'),
                     ' feedback L/R=', LaunchConfiguration('left_feedback_sign'), '/',
                     LaunchConfiguration('right_feedback_sign')]),
        robot_state_publisher,
        controller_manager,
        imu,
        ekf,
        TimerAction(period=2.0, actions=[controller]),
        TimerAction(period=6.0, actions=[odom_to_pose, guidance]),
    ])
