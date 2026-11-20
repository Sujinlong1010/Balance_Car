/* bsp_flash_params.h - store the tuned PID gains in on-chip flash
 *
 * The F103C8 has no EEPROM, so the last 1 KB page of the 64 KB flash
 * (0x0800FC00) is used as an emulated-EEPROM slot: erased once, then written
 * as halfwords with a magic + version + CRC16 trailer. A valid record is
 * restored automatically at boot; SAVE overwrites it after one page erase.
 *
 * Erase endurance is ~10 k cycles per page - fine for occasional saves,
 * NOT for auto-saving on every slider move (the firmware only saves on the
 * explicit SAVE command, and refuses while the balance loop is running
 * because a page erase stalls the CPU for tens of milliseconds).
 */
#ifndef __BSP_FLASH_PARAMS_H
#define __BSP_FLASH_PARAMS_H

#include <stdint.h>

/* The seven gains, in the same integer scales as the wire protocol. */
typedef struct
{
  int32_t kp100;      /* angle loop, x100 */
  int32_t ki100;      /* angle loop, x100 */
  int32_t kd100;      /* angle loop, x100 */
  int32_t kvp10000;   /* speed loop, x10000 */
  int32_t kvi10000;   /* speed loop, x10000 */
  int32_t ktp100;     /* turn loop, x100 */
  int32_t kti100;     /* turn loop, x100 */
} ParamsStore_Gains_t;

/* Write the record (erase + program + read-back verify). Returns 1 on
 * success, 0 on any flash error or verify mismatch. */
uint8_t ParamsStore_Save(const ParamsStore_Gains_t *in);

/* Read and validate the stored record. Returns 1 and fills *out when a
 * valid record exists, 0 when the slot is empty (erased) or corrupt. */
uint8_t ParamsStore_Load(ParamsStore_Gains_t *out);

#endif /* __BSP_FLASH_PARAMS_H */
