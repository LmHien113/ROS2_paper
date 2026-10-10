# CAN Frame Message
# Định nghĩa cấu trúc message để gửi dữ liệu CAN

uint32 can_id          # CAN ID (ví dụ: 0x123)
uint8[] data           # Dữ liệu CAN (tối đa 8 bytes)
