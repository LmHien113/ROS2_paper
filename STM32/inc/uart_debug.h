#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include "app_config.h"

/*
 * UART2 phat bang ngat TXE + vong dem: khong bao gio chan vong lap.
 * Dem day thi byte moi bi vut va dem vao UART2_GetDropped().
 */
void     UART2_Init(void);
void     UART2_SendByte(uint8_t value);
void     UART2_SendString(const char *text);
void     UART2_SendHexByte(uint8_t value);
void     UART2_SendHex16(uint16_t value);
void     UART2_SendInt32(int32_t value);
void     UART2_SendUInt32(uint32_t value);
void     UART2_SendFrame(const char *name, const uint8_t *data, uint8_t length);
void     UART2_SendButtonList(const char *prefix, uint16_t pressed);
uint32_t UART2_GetDropped(void);
void     UART2_Flush(void);   /* cho phat het, dung truoc vong treo */

#endif
