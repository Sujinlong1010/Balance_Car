/* bsp_oled.c - SSD1306 driver over HAL I2C, 6x8 ASCII font */
#include "bsp_oled.h"
#include "bsp_oled_font.h"
#include "i2c.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

static uint8_t s_gram[OLED_PAGES][OLED_WIDTH];

static uint8_t OLED_WriteCmd(uint8_t cmd)
{
  uint8_t data = cmd;
  if (HAL_I2C_Mem_Write(&hi2c1, OLED_I2C_ADDR, 0x00U,
                        I2C_MEMADD_SIZE_8BIT, &data, 1U,
                        OLED_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return 1U;
  }
  return 0U;
}

static uint8_t OLED_WriteData(uint8_t *buf, uint16_t len)
{
  if (HAL_I2C_Mem_Write(&hi2c1, OLED_I2C_ADDR, 0x40U,
                        I2C_MEMADD_SIZE_8BIT, buf, len,
                        OLED_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return 1U;
  }
  return 0U;
}

uint8_t OLED_Refresh(void)
{
  uint8_t page;

  for (page = 0U; page < OLED_PAGES; page++)
  {
    if (OLED_WriteCmd((uint8_t)(0xB0U + page)) != 0U) { return 1U; }
    if (OLED_WriteCmd(0x00U) != 0U) { return 1U; }  /* column low  nibble */
    if (OLED_WriteCmd(0x10U) != 0U) { return 1U; }  /* column high nibble */
    if (OLED_WriteData(s_gram[page], OLED_WIDTH) != 0U) { return 1U; }
  }
  return 0U;
}

uint8_t OLED_Init(void)
{
  static const uint8_t init_seq[] =
  {
    0xAEU,             /* display off */
    0xD5U, 0x80U,      /* clock divide / oscillator */
    0xA8U, 0x3FU,      /* multiplex ratio = 64 */
    0xD3U, 0x00U,      /* display offset = 0 */
    0x40U,             /* start line = 0 */
    0x8DU, 0x14U,      /* charge pump on */
    0x20U, 0x00U,      /* horizontal addressing mode */
    0xA1U,             /* segment remap */
    0xC8U,             /* COM scan direction remapped */
    0xDAU, 0x12U,      /* COM pins hardware config */
    0x81U, 0xCFU,      /* contrast */
    0xD9U, 0xF1U,      /* pre-charge period */
    0xDBU, 0x30U,      /* VCOMH deselect level */
    0xA4U,             /* output follows RAM */
    0xA6U,             /* normal (not inverted) display */
    0xAFU              /* display on */
  };
  uint16_t i;

  HAL_Delay(100U);     /* let the panel power supply settle */

  if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_I2C_ADDR, 2U,
                            OLED_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return 1U;
  }

  for (i = 0U; i < (uint16_t)sizeof(init_seq); i++)
  {
    if (OLED_WriteCmd(init_seq[i]) != 0U)
    {
      return 1U;
    }
  }

  OLED_Clear();
  return OLED_Refresh();
}

void OLED_Clear(void)
{
  memset(s_gram, 0, sizeof(s_gram));
}

void OLED_SetPixel(uint8_t x, uint8_t y, uint8_t on)
{
  uint8_t mask;

  if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
  {
    return;
  }
  mask = (uint8_t)(1U << (y & 7U));
  if (on != 0U)
  {
    s_gram[y >> 3][x] |= mask;
  }
  else
  {
    s_gram[y >> 3][x] &= (uint8_t)~mask;
  }
}

void OLED_ShowChar(uint8_t x, uint8_t page, char ch)
{
  uint8_t i;
  uint8_t index;

  if (page >= OLED_PAGES)
  {
    return;
  }
  if ((ch < (char)OLED_FONT_FIRST_CHAR) || (ch > (char)OLED_FONT_LAST_CHAR))
  {
    ch = ' ';
  }
  index = (uint8_t)(ch - (char)OLED_FONT_FIRST_CHAR);

  for (i = 0U; i < OLED_FONT_WIDTH; i++)
  {
    if ((uint16_t)x + i < OLED_WIDTH)
    {
      s_gram[page][x + i] = OLED_FONT6X8[index][i];
    }
  }
}

void OLED_ShowString(uint8_t x, uint8_t page, const char *str)
{
  while ((*str != '\0') && (x + OLED_FONT_WIDTH <= OLED_WIDTH))
  {
    OLED_ShowChar(x, page, *str);
    x = (uint8_t)(x + OLED_FONT_WIDTH);
    str++;
  }
}

void OLED_Printf(uint8_t x, uint8_t page, const char *fmt, ...)
{
  char line[32];       /* 128 px / 6 px = 21 chars max */
  va_list args;

  va_start(args, fmt);
  (void)vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);

  line[sizeof(line) - 1U] = '\0';
  OLED_ShowString(x, page, line);
}

void OLED_SetContrast(uint8_t value)
{
  (void)OLED_WriteCmd(0x81U);
  (void)OLED_WriteCmd(value);
}

void OLED_DisplayOn(uint8_t on)
{
  (void)OLED_WriteCmd(on != 0U ? 0xAFU : 0xAEU);
}
