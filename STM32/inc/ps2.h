#ifndef PS2_H
#define PS2_H

#include "app_config.h"

/* Header 0x79 (analog + pressure) dai 21 byte -> moi dem rx phai >= 21. */
#define PS2_FRAME_MAX           21U

void     PS2_SPI_Init(void);   /* doi PCLK2 -> goi TRUOC Delay_UsInit() */
void     PS2_EnterAnalog(void);

/* Doc 1 frame poll. Chi tra ve 1 voi frame analog (0x73 / 0x79). */
uint8_t  PS2_ReadPoll(uint8_t *rx);

uint8_t  PS2_IsDigitalFrame(const uint8_t *rx);

/* Toan bo 16 bit nut (co SELECT / START), da bo nut hong. */
uint16_t PS2_GetAllPressed(const uint8_t *rx);

/* Chi cac nut lai (BTN_USED_MASK). */
uint16_t PS2_GetPressedButtons(const uint8_t *rx);

#endif
