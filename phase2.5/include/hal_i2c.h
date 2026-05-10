#ifndef HAL_I2C_H
#define HAL_I2C_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Configure the I2C peripheral that talks to the two MPU-6050s.
 * Idempotent: safe to call more than once. */
void hal_i2c_init(void);

/* Read n bytes from a given register on a given 7-bit slave address.
 *   slave_addr_7bit : MPU-6050 is 0x68 (AD0=GND) or 0x69 (AD0=VCC)
 *   reg             : starting register address
 *   buf             : destination buffer (must be at least n bytes)
 *   n               : number of bytes to read
 * Returns true on success, false on bus error / NACK. */
bool hal_i2c_read_bytes(uint8_t slave_addr_7bit,
                        uint8_t reg,
                        uint8_t *buf,
                        size_t n);

/* Write n bytes to a given register on a given 7-bit slave address.
 * Same address conventions as hal_i2c_read_bytes(). */
bool hal_i2c_write_bytes(uint8_t slave_addr_7bit,
                         uint8_t reg,
                         const uint8_t *buf,
                         size_t n);

#endif /* HAL_I2C_H */
