/* bsp_hc05.h - HC-05 Bluetooth on USART3 (PB10/PB11), 115200 8N1
 *
 * CubeMX enabled the USART3 global interrupt (priority 6). This driver uses
 * HAL_UART_Receive_IT() one byte at a time and buffers bytes in a ring.
 * AT mode is 38400 baud; for transparent serial mode set the module once with
 *   AT+UART=115200,0,0
 * STATE is not wired to an MCU pin in this schematic.
 */
#ifndef __BSP_HC05_H
#define __BSP_HC05_H

#include <stdint.h>
#include "main.h"

#define HC05_RX_BUF_SIZE      128U
#define HC05_TX_TIMEOUT_MS    200U

void     HC05_Init(void);
uint16_t HC05_Available(void);
int16_t  HC05_ReadByte(void);                 /* -1 when the buffer is empty */
uint16_t HC05_ReadLine(char *buf, uint16_t cap);  /* non-blocking, '\0' ended */
void     HC05_FlushRx(void);
uint8_t  HC05_SendBytes(const uint8_t *data, uint16_t len);
uint8_t  HC05_SendString(const char *str);
uint8_t  HC05_SendATCommand(const char *cmd);     /* appends "\r\n" */

#endif /* __BSP_HC05_H */
