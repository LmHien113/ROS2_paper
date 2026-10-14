#ifndef ROS_LINK_H
#define ROS_LINK_H

#include "app_config.h"

/*
 * Giao tiep ROS2 <-> STM32 va may trang thai mode PS2/AUTO.
 *
 *   0x050  ROS2 -> STM32  01 00 .. = bat gui 0x011/0x012, 00 00 .. = tat
 *   0x020  ROS2 -> STM32  data[0] = 1 (PS2) / 2 (AUTO)
 *   0x030  ROS2 -> STM32  int32 LE trai | phai, pulse/s, chi o AUTO
 *   0x040  ROS2 -> STM32  uint32 > 0 -> xoay tai cho, chi o AUTO
 *   0x011  STM32 -> ROS2  van toc thuc 0x606C, int32 LE trai | phai
 *   0x012  STM32 -> ROS2  vi tri thuc 0x6064, int32 LE trai | phai
 *   0x013  STM32 -> ROS2  trang thai (ROS_SEND_STATUS)
 *
 * ROS2 dung quy uoc "duong = banh quay TIEN"; module tu doi dau (AUTO_SIGN_*).
 * Vao AUTO: ROS2 phai gui 0x030 = (0, 0) truoc de ARM.
 */

#define ROS_FLAG_ARMED      0x01U
#define ROS_FLAG_LINK_LOST  0x02U
#define ROS_FLAG_NMT_BAD    0x04U
#define ROS_FLAG_DRV_FAULT  0x08U
#define ROS_FLAG_WD_EXPIRED 0x10U
#define ROS_FLAG_PS2_OK     0x20U

void    RosLink_Init(void);

/* Frame khong thuoc CANopen. Tra ve 1 = da xu ly. */
uint8_t RosLink_HandleCanFrame(uint16_t std_id, const uint8_t *data, uint8_t dlc);

/*
 * Ap dung yeu cau doi mode. Goi MOT lan moi vong, o diem co dinh.
 * Tra ve 1 = vua doi mode -> main phai ep v = 0 va khoa driver.
 */
uint8_t RosLink_Service(uint32_t now_ms);
void    RosLink_RequestMode(uint8_t mode, const char *src);
void    RosLink_SelectPressed(uint32_t now_ms);
uint8_t RosLink_GetMode(void);

/*
 * Van toc cho mode AUTO (da xu ly watchdog, clamp, slew, ARM, xoay 0x040).
 * *need_enable = 1 mot lan khi du dieu kien ARM: main Enable 2 driver roi
 * goi RosLink_AutoArmDone(). Tra ve 1 = duoc chay.
 */
uint8_t RosLink_Update(uint32_t now_ms, int32_t *vel_l, int32_t *vel_r,
                       uint8_t *need_enable);
void    RosLink_AutoArmDone(void);
uint8_t RosLink_AutoIsArmed(void);
uint8_t RosLink_AutoWatchdogExpired(uint32_t now_ms);
void    RosLink_AutoDisarm(const char *why);

void    RosLink_SendFeedback(void);          /* 0x011 + 0x012 */
void    RosLink_SendStatus(uint32_t now_ms, uint8_t flags);
void    RosLink_PrintStats(void);
void    RosLink_PrintTelemetry(uint32_t now_ms);
#endif
