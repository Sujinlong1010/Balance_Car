/* bsp_flash_params.c - F1 flash page used as a gains storage slot
 *
 * Register-level programming (no HAL flash module dependency):
 *   unlock (KEYR x2) -> erase page (PER + AR + STRT) -> program halfwords
 *   (PG + write) -> lock. All BSY waits have a timeout so a hardware fault
 *   cannot hang the 5 ms task forever.
 *
 * Record layout on 0x0800FC00 (halfword units, little-endian int32 = 2 HW):
 *   [0..1]  magic 0xA55A,0xC33C
 *   [2]     version 0x0001
 *   [3..16] kp,ki,kd (x100), kvp,kvi (x10000), ktp,kti (x100)
 *   [17]    CRC16-CCITT over halfwords [0..16]
 *   rest    0xFFFF (erased)
 */
#include "bsp_flash_params.h"
#include "main.h"

/* Last 1 KB page of the 64 KB flash. The firmware image currently ends
 * around 0x0800A000 (~40 KB), so this page stays clear of the program. */
#define PP_PAGE_ADDR      0x0800FC00UL

/* F1 flash register bits / keys, named PP_* to avoid clashing with HAL. */
#define PP_KEY1           0x45670123UL
#define PP_KEY2           0xCDEF89ABUL
#define PP_SR_BSY         0x00000001UL
#define PP_SR_PGERR       0x00000004UL
#define PP_SR_WRPRTERR    0x00000010UL
#define PP_SR_EOP         0x00000020UL
#define PP_SR_CLR_ALL     (PP_SR_EOP | PP_SR_PGERR | PP_SR_WRPRTERR)
#define PP_CR_PG          0x00000001UL
#define PP_CR_PER         0x00000002UL
#define PP_CR_STRT        0x00000040UL
#define PP_CR_LOCK        0x00000080UL

#define PP_MAGIC0         0xA55AU
#define PP_MAGIC1         0xC33CU
#define PP_VERSION        0x0001U

#define PP_REC_HW         18U    /* halfwords in the record */
#define PP_BS_TIMEOUT     4000000UL

static uint16_t CRC16(const uint16_t *data, uint32_t n)
{
  uint16_t crc = 0xFFFFU;
  uint32_t i;
  int b;

  for (i = 0U; i < n; i++)
  {
    crc ^= data[i];
    for (b = 0; b < 8; b++)
    {
      crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0xA001U)
                       : (uint16_t)(crc >> 1);
    }
  }
  return crc;
}

static uint8_t FlashWaitDone(void)
{
  uint32_t guard = PP_BS_TIMEOUT;

  while ((FLASH->SR & PP_SR_BSY) != 0U)
  {
    if (--guard == 0UL) { return 0U; }
  }
  return 1U;
}

static uint8_t FlashUnlock(void)
{
  if ((FLASH->CR & PP_CR_LOCK) != 0U)
  {
    FLASH->KEYR = PP_KEY1;
    FLASH->KEYR = PP_KEY2;
  }
  if ((FLASH->CR & PP_CR_LOCK) != 0U) { return 0U; }  /* wrong sequence */
  return FlashWaitDone();
}

static void FlashLock(void)
{
  FLASH->CR |= PP_CR_LOCK;
}

static uint16_t PackHalf(const ParamsStore_Gains_t *g, uint32_t idx)
{
  const int32_t *vals = (const int32_t *)g;
  int32_t v = vals[(idx - 3U) / 2U];

  /* Odd record index = low halfword of the gain, even = high halfword. */
  return (uint16_t)(((idx & 1U) != 0U) ? (v & 0xFFFFU)
                                       : ((v >> 16) & 0xFFFFU));
}

uint8_t ParamsStore_Load(ParamsStore_Gains_t *out)
{
  const volatile uint16_t *p = (const volatile uint16_t *)PP_PAGE_ADDR;
  uint16_t rec[PP_REC_HW];
  uint32_t i;
  int32_t *vals;

  for (i = 0U; i < PP_REC_HW; i++) { rec[i] = p[i]; }
  if ((rec[0] != PP_MAGIC0) || (rec[1] != PP_MAGIC1) ||
      (rec[2] != PP_VERSION))
  {
    return 0U;                                    /* empty or foreign data */
  }
  if (rec[PP_REC_HW - 1U] != CRC16(rec, PP_REC_HW - 1U))
  {
    return 0U;                                    /* corrupt */
  }

  vals = (int32_t *)out;
  for (i = 0U; i < 7U; i++)
  {
    vals[i] = (int32_t)rec[3U + (i * 2U)] |
              ((int32_t)rec[4U + (i * 2U)] << 16);
  }
  return 1U;
}

uint8_t ParamsStore_Save(const ParamsStore_Gains_t *in)
{
  volatile uint16_t *p = (volatile uint16_t *)PP_PAGE_ADDR;
  uint16_t rec[PP_REC_HW];
  uint32_t i;

  if (in == 0) { return 0U; }

  rec[0] = PP_MAGIC0;
  rec[1] = PP_MAGIC1;
  rec[2] = PP_VERSION;
  for (i = 3U; i < 17U; i++) { rec[i] = PackHalf(in, i); }
  rec[17] = CRC16(rec, 17U);

  if (FlashUnlock() == 0U) { return 0U; }

  /* Erase the whole page. */
  FLASH->SR = PP_SR_CLR_ALL;
  FLASH->CR |= PP_CR_PER;
  FLASH->AR  = PP_PAGE_ADDR;
  FLASH->CR |= PP_CR_STRT;
  if (FlashWaitDone() == 0U)
  {
    FLASH->CR &= ~PP_CR_PER;
    FlashLock();
    return 0U;
  }
  FLASH->SR = PP_SR_CLR_ALL;
  FLASH->CR &= ~PP_CR_PER;

  /* Program the record as halfwords. */
  FLASH->CR |= PP_CR_PG;
  for (i = 0U; i < PP_REC_HW; i++)
  {
    p[i] = rec[i];
    if (FlashWaitDone() == 0U) { break; }
    if ((FLASH->SR & (PP_SR_PGERR | PP_SR_WRPRTERR)) != 0U) { break; }
  }
  FLASH->CR &= ~PP_CR_PG;
  FlashLock();
  if (i != PP_REC_HW) { return 0U; }

  /* Read back and verify before reporting success. */
  for (i = 0U; i < PP_REC_HW; i++)
  {
    if (p[i] != rec[i]) { return 0U; }
  }
  return 1U;
}
