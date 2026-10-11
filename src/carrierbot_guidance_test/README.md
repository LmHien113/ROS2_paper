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

## Build

```bash
cd ~/ros2_paper
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
