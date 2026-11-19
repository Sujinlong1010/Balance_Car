/* bsp_hc05.c - USART3 ring buffer driver for HC-05 */
#include "bsp_hc05.h"
#include "usart.h"
#include <string.h>

extern UART_HandleTypeDef huart3;

static volatile uint8_t  s_rxbuf[HC05_RX_BUF_SIZE];
static volatile uint16_t s_head = 0U;
static volatile uint16_t s_tail = 0U;
static uint8_t           s_rx_byte = 0U;

void HC05_Init(void)
{
  s_head = 0U;
  s_tail = 0U;
  /* Start the first single-byte interrupt receive. The callback re-arms it. */
  (void)HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1U);
}

/* Overrides the __weak HAL callback. Runs in USART3_IRQHandler context. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint16_t next;

  if (huart->Instance != USART3)
  {
    return;
  }

  next = (uint16_t)((s_head + 1U) % HC05_RX_BUF_SIZE);
  if (next == s_tail)               /* buffer full: discard the oldest byte */
  {
    s_tail = (uint16_t)((s_tail + 1U) % HC05_RX_BUF_SIZE);
  }
  s_rxbuf[s_head] = s_rx_byte;
  s_head = next;

  (void)HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1U);
}

uint16_t HC05_Available(void)
{
  return (uint16_t)((s_head + HC05_RX_BUF_SIZE - s_tail) % HC05_RX_BUF_SIZE);
}

int16_t HC05_ReadByte(void)
{
  uint8_t byte;

  if (s_head == s_tail)
  {
    return -1;
  }
  byte = s_rxbuf[s_tail];
  s_tail = (uint16_t)((s_tail + 1U) % HC05_RX_BUF_SIZE);
  return (int16_t)byte;
}

void HC05_FlushRx(void)
{
  s_tail = s_head;
}

uint16_t HC05_ReadLine(char *buf, uint16_t cap)
{
  uint16_t n = 0U;
  int16_t  c;

  if ((buf == 0) || (cap == 0U))
  {
    return 0U;
  }

  while (n < (uint16_t)(cap - 1U))
  {
    c = HC05_ReadByte();
    if (c < 0)
    {
      break;
    }
    if ((c == '\n') || (c == '\r'))
    {
      if (n > 0U)
      {
        break;                     /* end of line */
      }
      continue;                    /* skip leading CR/LF */
    }
    buf[n] = (char)c;
    n++;
  }
  buf[n] = '\0';
  return n;
}

uint8_t HC05_SendBytes(const uint8_t *data, uint16_t len)
{
  if ((data == 0) || (len == 0U))
  {
    return 0U;
  }
  if (HAL_UART_Transmit(&huart3, (uint8_t *)data, len,
                        HC05_TX_TIMEOUT_MS) != HAL_OK)
  {
    return 1U;
  }
  return 0U;
}

uint8_t HC05_SendString(const char *str)
{
  if (str == 0)
  {
    return 0U;
  }
  return HC05_SendBytes((const uint8_t *)str, (uint16_t)strlen(str));
}

uint8_t HC05_SendATCommand(const char *cmd)
{
  uint8_t rc = HC05_SendString(cmd);
  if (rc != 0U)
  {
    return rc;
  }
  return HC05_SendString("\r\n");
}
