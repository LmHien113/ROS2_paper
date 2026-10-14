#ifndef CANOPEN_H
#define CANOPEN_H

#include "app_config.h"

/* Cac mang trang thai [3] danh chi so truc tiep bang Node-ID. */
#if (NODE_L == 0U) || (NODE_L > 2U) || (NODE_R == 0U) || (NODE_R > 2U) || (NODE_L == NODE_R)
  #error "NODE_L / NODE_R phai la 1 va 2, khong trung nhau."
#endif

/* Statusword CiA 402 */
#define SW_FAULT(s)         (((s) & 0x004FU) == 0x0008U)
#define SW_OP_ENABLED(s)    (((s) & 0x006FU) == 0x0027U)

/* So lieu tu driver. Cap nhat trong CAN_Poll() (khong co ISR). */
extern int32_t  g_actual_vel_l, g_actual_vel_r;     /* 0x606C */
extern int32_t  g_actual_pos_l, g_actual_pos_r;     /* 0x6064 */
extern uint32_t g_actual_vel_ms[3];                 /* 0 = chua co */
extern uint32_t g_actual_pos_ms[3];
extern uint16_t g_status[3];                        /* 0x6041 */
extern uint32_t g_hb_ms[3];
extern uint8_t  g_nmt_state[3];
extern uint8_t  g_node_booted[3];                   /* 1 = vua boot lai */
extern uint32_t g_sdo_write_fail;

/* Electronic gear + 0x607F doc tu driver (manual mau thuan ve default). */
typedef struct {
    uint32_t ppr;        /* 0x2A90    */
    uint32_t gear_num;   /* 0x2A93:01 */
    uint32_t gear_den;   /* 0x2A93:02 */
    uint32_t vmax;       /* 0x607F, pulses/s */
    uint8_t  valid;
} DriveGear;

extern DriveGear g_gear[3];

void    CAN_Poll(void);

/* SDO doc khong cho: ket qua ve qua CAN_Poll(). Toi da 1 lan/node/tick. */
uint8_t SDO_Read_Request(uint8_t node, uint16_t index, uint8_t sub);

/* Ghi SDO target=0 + Shutdown, roi RPDO Shutdown. Dung luc khoi dong. */
void    Drive_ForceSafe(uint8_t node);

void    Drive_DumpGearParams(void);              /* chi goi luc khoi dong  */
uint8_t Drive_Init(uint8_t node);                /* chan ~270 ms           */
uint8_t Drive_SetRamp(uint8_t node, uint32_t accel, uint32_t decel);
uint8_t Drive_EnableOperation(uint8_t node);     /* chan ~60 ms            */

uint8_t Drive_SetVelocity(uint8_t node, int32_t velocity);  /* CW 0x000F */
uint8_t Drive_Lock(uint8_t node);                           /* CW 0x0006, v = 0 */

/* 1 = node loi / chua Enabled -> khong cho chay. Goi dinh ky tu main. */
uint8_t Drive_CheckAndRecover(uint8_t node);
uint8_t Drive_CheckNmtState(uint8_t node);
uint8_t Drive_ActualVelFresh(uint8_t node);
void    Drive_LinkInit(void);
uint8_t Drive_LinkLost(uint8_t node);

#endif
