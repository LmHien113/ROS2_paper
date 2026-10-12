# carrierbot_guidance_test

Khu thử nghiệm Guidance (LOS + PID), tách biệt hoàn toàn với các package gốc.
Mọi sửa đổi ở đây **không** ảnh hưởng `carrierbot_navigation` hay `carrierbot_bringup`.

## Cấu trúc

```
carrierbot_guidance_test/
├── CMakeLists.txt
├── package.xml
├── README.md
├── src/guidance_test_node.cpp        # bản sao fablab_guidance_node, đã có PID
├── scripts/odom_to_pose.py           # /odometry/filtered -> /guidance_pose
├── scripts/send_route.py             # gửi waypoint theo vị trí xuất phát của xe
├── launch/guidance_odom_test.launch.py
├── config/ekf_test.yaml              # EKF: bánh xe góp vận tốc, IMU góp góc hướng
└── data/                             # log guidance_test_*.csv
```

## Khác bản gốc thế nào

| | Bản gốc | Bản thử nghiệm |
| --- | --- | --- |
| Package | `carrierbot_navigation` | `carrierbot_guidance_test` |
| Executable / tên node | `fablab_guidance_node` | `guidance_test_node` |
| Bộ điều khiển hướng | PD | PID (KP, KI, KD, tích phân có điều kiện, anti-windup) |
| Nguồn tư thế | `/amcl_pose` | `/guidance_pose` (từ EKF, qua `odom_to_pose`) |
| Log | `carrierbot_datalog/data/guidance_*.csv` | `carrierbot_guidance_test/data/guidance_test_*.csv` |

Các package gốc (`carrierbot_firmware`, `carrierbot_controller`, `carrierbot_slam`)
chỉ được **dùng lại**, không bị sửa.

## Hardware interface thử nghiệm (dấu từng bánh)

Package có bản sao riêng của hardware interface: `carrierbot_guidance_test::CarrierbotInterfaceTest`
(`src/carrierbot_interface_test.cpp`, mô tả robot `urdf/carrierbot_test.urdf.xacro`).
Khác bản gốc duy nhất ở chỗ dấu của từng bánh là tham số:

| Tham số launch | Mặc định (test) | Bản gốc (cố định trong code) |
| --- | --- | --- |
| `left_command_sign` | `1` | `-1` |
| `right_command_sign` | `1` | `1` |
| `left_feedback_sign` | `1` | `-1` |
| `right_feedback_sign` | `1` | `1` |

Đã kiểm chứng ngày 12/10/2026 (STM32 ở `drive_mode` 2): cả lệnh gửi xuống lẫn encoder đọc về,
hai bánh đều dương là tiến. So với firmware gốc, bánh trái không còn cần đảo dấu ở cả hai chiều.

Log khởi động in ra dấu đang dùng: `[TEST] Configured hardware: ... sign cmd L=+1 R=+1, feedback L=+1 R=+1`.

## EKF thử nghiệm

`config/ekf_test.yaml` (tham số launch `ekf_config`) khác `carrierbot_slam/config/ekf.yaml`:
bánh xe chỉ góp vận tốc, góc hướng lấy từ IMU tính tương đối so với lúc khởi động. Bản gốc gộp
hai góc tuyệt đối không cùng gốc (bánh xe từ 0°, IMU theo la bàn), làm góc EKF tự trôi khi xe chạy:
đi thẳng 2 m thì IMU −0.2°, bánh xe −0.1°, nhưng EKF −5.2°.

## Build

```bash
cd ~/Documents/carrierbot_ws
colcon build --packages-select carrierbot_guidance_test
source install/setup.bash
```

Các package gốc phải được build ít nhất một lần trước đó.

## Chạy trên xe thật (không MQTT, AMCL, lidar, SLAM)

> CẢNH BÁO: không có lidar nên **không có dừng tránh vật cản**.
> Luôn có người giữ nút E-stop, khu vực thử phải trống.

```bash
# Terminal 1
ros2 launch carrierbot_guidance_test guidance_odom_test.launch.py kp:=0.44 ki:=0.0 kd:=0.19

# Terminal 2: xem trước toạ độ, rồi gửi thật (TOI,TRAI tính bằng mét)
ros2 run carrierbot_guidance_test send_route.py 4,0 4,4 --dry-run
ros2 run carrierbot_guidance_test send_route.py 4,0 4,4
```

Tham số launch hay dùng: `kp`, `ki`, `kd`, `integral_zone`, `integral_max_output`,
`speed`, `min_speed`, `angular_speed_max`, `delta_min`, `delta_max`,
`pose_rate_hz` (`2.0` = giả lập tần số AMCL), `cmd_vel_topic`.
Xem đầy đủ: `ros2 launch carrierbot_guidance_test guidance_odom_test.launch.py --show-args`.

**Không chạy cùng lúc** với `real_robot.launch.py`: cả hai cùng phát `/cmd_vel`
và cùng điều khiển phần cứng.

## Tạm tắt package này

Tạo file rỗng `COLCON_IGNORE` trong thư mục này, colcon sẽ bỏ qua khi build.
Xoá file đó để bật lại.

## Đưa thay đổi về bản gốc

Khi bản thử nghiệm đã ổn trên xe, chép phần thuật toán từ
`src/guidance_test_node.cpp` sang `carrierbot_navigation/src/fablab_guidance_node.cpp`.
Giữ nguyên tên lớp, tên node và đường dẫn log của bản gốc.
