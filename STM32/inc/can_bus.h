#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "app_config.h"

extern uint32_t g_can_tx_ok;
extern uint32_t g_can_tx_fail;
extern uint32_t g_can_tx_cancel;
extern uint32_t g_can_rx_cnt;

void    CAN_Motor_Init(void);

/* 1 = frame da phat va co node ACK. Huy mailbox neu qua han. */
uint8_t CAN_SendFrame(uint16_t std_id, const uint8_t *data, uint8_t dlc);

void    CAN_PrintErrorStatus(void);

#endif
