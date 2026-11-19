/* bsp_oled.h - SSD1306 128x64 OLED on hardware I2C1 (PB8/PB9 remap)
 *
 * I2C1 is configured by CubeMX at 400 kHz. This driver keeps a 128x8
 * framebuffer in RAM and only pushes it to the panel on OLED_Refresh().
 */
#ifndef __BSP_OLED_H
#define __BSP_OLED_H

#include <stdint.h>
#include "main.h"

#define OLED_I2C_ADDR        0x78U   /* 7-bit 0x3C << 1; use 0x7A if SA0 is high */
#define OLED_WIDTH           128U
#define OLED_HEIGHT          64U
#define OLED_PAGES           (OLED_HEIGHT / 8U)
#define OLED_I2C_TIMEOUT_MS  100U

/* Returns 0 on success, 1 if the panel does not answer on I2C. */
uint8_t OLED_Init(void);          /* 0 = OK, 1 = I2C error */
void    OLED_Clear(void);
uint8_t OLED_Refresh(void);       /* 0 = OK, 1 = I2C error */
void    OLED_SetPixel(uint8_t x, uint8_t y, uint8_t on);
void    OLED_ShowChar(uint8_t x, uint8_t page, char ch);
void    OLED_ShowString(uint8_t x, uint8_t page, const char *str);
void    OLED_Printf(uint8_t x, uint8_t page, const char *fmt, ...);
void    OLED_SetContrast(uint8_t value);
void    OLED_DisplayOn(uint8_t on);

#endif /* __BSP_OLED_H */
