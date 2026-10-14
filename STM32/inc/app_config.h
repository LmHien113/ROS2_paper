#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "stm32f4xx.h"
#include <stdint.h>

/* ======================================================================== */
/*                              He thong                                    */
/* ======================================================================== */

#define UART_BAUDRATE           115200UL
#define UART_TX_BUF_SIZE        1024U   /* luy thua cua 2; day thi vut log */

#define LOOP_PERIOD_MS          20U     /* vong lap chinh 50 Hz            */
#define UART_DEBUG_PERIOD_MS    1000U
#define ROS_UART_TELEM_PERIOD_MS 1000U

/* ======================================================================== */
/*                                  PS2                                     */
/* ======================================================================== */

/*
 * SPI1 mode 2 (CPOL High, CPHA 1Edge). Do tren board nay: mode 3 lech 1 bit.
 * Debounce 2 = tre 1 chu ky 20 ms.
 */
#define PS2_DEBOUNCE_COUNT      2U
#define PS2_LEADIN_US           30U
#define PS2_INTERBYTE_US        25U
#define PS2_TAIL_US             60U
#define PS2_SPI_TIMEOUT         100000UL
#define PS2_TIMEOUT_MS          300U    /* mat frame analog -> dung        */
#define PS2_ANALOG_ARM_COUNT    3U      /* so frame neutral lien tiep de ARM */

/* ---- Bit nut PS2 (sau khi dao: 1 = dang nhan) ---- */
#define BTN_SELECT              0x0001U
#define BTN_START               0x0008U
#define BTN_UP                  0x0010U
#define BTN_RIGHT               0x0020U
#define BTN_DOWN                0x0040U
#define BTN_LEFT                0x0080U
#define BTN_TRIANGLE            0x1000U
#define BTN_CIRCLE              0x2000U
#define BTN_CROSS               0x4000U
#define BTN_SQUARE              0x8000U

#define BTN_USED_MASK   (BTN_UP | BTN_RIGHT | BTN_DOWN | BTN_LEFT | \
                         BTN_TRIANGLE | BTN_CIRCLE | BTN_CROSS | BTN_SQUARE)

/* Nut hong tren tay cam (luon bao "dang nhan") -> bo qua. 0 = khong co. */
#define PS2_STUCK_MASK          0x0000U

/* ======================================================================== */
/*                              CAN / driver                                */
/* ======================================================================== */

/*
 * CAN2: RX = PB5, TX = PB13 (co dinh trong can_bus.c).
 * Chon sai chan RX -> "txOK ket o 3, rx = 0" (Bit Error, retry vo han).
 */
#define CAN_BITRATE_KBPS        1000U

/*
 * Thoi gian cho phat 1 frame. 1 frame 8 byte o 1 Mbit/s ~130 us.
 * Ha bitrate thi tang CAN_TX_TIMEOUT_US = 8 x thoi gian 1 frame.
 */
#define CAN_TX_MAILBOX_WAIT_US  200UL
#define CAN_TX_TIMEOUT_US       800UL

/* Node-ID: chi duoc la 1 va 2 (cac mang trang thai danh chi so bang ID). */
#define NODE_L                  1U
#define NODE_R                  2U

#define MODE_PROFILE_VELOCITY   3       /* 0x6060 = 3 */

/* Toc do PS2, pulses/s (0x60FF). rpm = 60 x v x 0x2A93:01 / (0x2A90 x 0x2A93:02) */
#define JOG_VEL                 110000L

/* Ramp PS2 (0x6083/0x6084): tang 2.2 s, giam 0.55 s tu JOG_VEL. */
#define ACCEL_VAL               50000UL
#define DECEL_VAL               200000UL

/* Doc gear + 0x607F luc khoi dong (chi DOC, an toan). Can cho clamp AUTO. */
#define DRIVE_DUMP_GEAR         1U

#define RPDO_KEEPALIVE_MS       100U    /* gui lai RPDO dinh ky            */
#define HEARTBEAT_PERIOD_MS     100U    /* driver phat heartbeat           */
#define NODE_TIMEOUT_MS         400U    /* mat 3 nhip -> coi nhu chet      */
#define DRIVE_RECOVER_PERIOD_MS 200U    /* chu ky Drive_CheckAndRecover    */

/*
 * TPDO1 = 0x606C + 0x6041, driver tu day moi TPDO1_EVENT_MS.
 * Neu driver khong ho tro, firmware tu lui ve hoi bang SDO.
 */
#define TPDO1_EVENT_MS          20U
#define ACTUAL_VEL_STALE_MS     200U
#define ROS_POS_READ_PERIOD_MS  20U     /* nhip hoi SDO 0x6064/0x6041      */

/* ======================================================================== */
/*                  MODE DIEU KHIEN: PS2 (tay cam) / AUTO (ROS2)            */
/* ======================================================================== */

#define CTRL_MODE_PS2           1U
#define CTRL_MODE_AUTO          2U
#define CTRL_MODE_DEFAULT       CTRL_MODE_PS2   /* KHONG BAO GIO boot vao AUTO */

/* SELECT x3 trong 1.5 s -> doi mode. START khi dang AUTO -> ve PS2 ngay. */
#define MODE_SELECT_PRESS_COUNT 3U
#define MODE_SELECT_WINDOW_MS   1500U
#define AUTO_ESTOP_BTN          BTN_START

/* ---- CAN ID giao tiep voi ROS2 ---- */
#define CANID_ROS_MODE_CMD      0x020U  /* ROS2 -> STM32 : chon mode        */
#define CANID_ROS_VEL_CMD       0x030U  /* ROS2 -> STM32 : van toc 2 banh   */
#define CANID_ROS_ROTATE_CMD    0x040U  /* ROS2 -> STM32 : xoay trai 90 do  */
#define CANID_ROS_FB_START      0x050U  /* ROS2 -> STM32 : bat/tat phan hoi */
#define CANID_ROS_VEL_FB        0x011U  /* STM32 -> ROS2 : van toc 0x606C   */
#define CANID_ROS_POS_FB        0x012U  /* STM32 -> ROS2 : vi tri 0x6064    */
#define CANID_ROS_STATUS        0x013U  /* STM32 -> ROS2 : trang thai       */

/*
 * Gioi han AUTO, pulse/s. 0.9 m/s ~ 286479 pulse/s.
 * Frame vuot REJECT bi VUT (du lieu hong), vuot MAX bi cat.
 * Firmware con lay MIN voi 0x607F doc tu driver.
 */
#define AUTO_VEL_MAX_PULSE       100000L
#define AUTO_VEL_REJECT_PULSE   (AUTO_VEL_MAX_PULSE * 4L)

/* Thay doi toi da moi chu ky 20 ms. 0 = tat. */
#define AUTO_SLEW_MAX_PULSE     30000L

/*
 * Watchdog 0x030: qua TIMEOUT -> v = 0; qua DISARM -> huy ARM, ROS2 phai
 * gui lai frame 0x030 = (0, 0) moi chay tiep. ROS2 nen gui 20..50 Hz.
 */
#define AUTO_CMD_TIMEOUT_MS     200U
#define AUTO_CMD_DISARM_MS      1000U

/*
 * ROS2 gui "duong = banh quay TIEN". Driver nay: banh trai AM khi tien.
 * Robot chay lui khi ROS2 bao tien -> doi dau CA HAI; quay tai cho -> doi MOT.
 */
#define AUTO_SIGN_L             (-1L)
#define AUTO_SIGN_R             (+1L)

/* Ramp rieng cho AUTO (vong kin ROS2 can phan ung nhanh hon PS2). */
#define AUTO_ACCEL_VAL          500000UL
#define AUTO_DECEL_VAL          1000000UL

///* ---- Phan hoi ve ROS2 (chi gui sau khi nhan 0x050 = 01 00 ..) ---- */
//#define ROS_FB_PERIOD_MS        20U     /* 0x011 + 0x012, 50 Hz            */
//#define ROS_FB_START_BYTE       0x01U
//#define ROS_FB_STOP_BYTE        0x00U

/* 0x013: mode, co ARM/loi, statusword. 1 = gui. */
#define ROS_SEND_STATUS         0U
#define ROS_STATUS_PERIOD_MS    100U

/* ======================================================================== */
/*               XOAY TAI CHO BANG LENH CAN 0x040 (chi o AUTO)              */
/* ======================================================================== */

/*
 * 040#0100000000000000 -> xoay. DLC = 8, byte 0..3 uint32 > 0.
 * ROTATE_ANGLE_FROM_CAN = 1: byte 0..3 la so do (cat ve ROTATE_MAX_DEG).
 * Dung khi MOT banh di du pulse; huy khi mat 0x6064 hoac qua thoi gian.
 */
#define ROTATE_ANGLE_DEG        90U
#define ROTATE_ANGLE_FROM_CAN   0U
#define ROTATE_MAX_DEG          360U
#define ROTATE_PULSE_PER_360    550000L /* 27500 x 20 */
#define ROTATE_VEL_PULSE        20000L
#define ROTATE_DIRECTION        (+1)    /* +1 = trai, -1 = phai */
#define ROTATE_POS_STALE_MS     200U
#define ROTATE_TIMEOUT_MARGIN_MS 2000U

#if (ROTATE_VEL_PULSE <= 0) || (ROTATE_VEL_PULSE > AUTO_VEL_MAX_PULSE)
  #error "ROTATE_VEL_PULSE phai > 0 va <= AUTO_VEL_MAX_PULSE"
#endif
#if (ROTATE_ANGLE_DEG == 0U) || (ROTATE_ANGLE_DEG > ROTATE_MAX_DEG)
  #error "ROTATE_ANGLE_DEG phai trong khoang 1..ROTATE_MAX_DEG"
#endif
#if ((ROTATE_DIRECTION) != 1) && ((ROTATE_DIRECTION) != -1)
  #error "ROTATE_DIRECTION chi duoc la +1 hoac -1"
#endif

#endif
