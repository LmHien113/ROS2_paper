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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
    ]

    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('carrierbot_firmware'), 'launch', 'hardware_interface.launch.py')),
        launch_arguments={'use_sim_time': 'false', 'drive_mode': LaunchConfiguration('drive_mode')}.items(),
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
        hardware_interface,
        imu,
        ekf,
        TimerAction(period=2.0, actions=[controller]),
        TimerAction(period=6.0, actions=[odom_to_pose, guidance]),
    ])
