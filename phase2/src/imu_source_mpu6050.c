#include <string.h>
#include "imu_source.h"
#include "hal_i2c.h"

/* MPU-6050 driver. Implements the abstract imu_source.h interface so
 * the rest of the firmware doesn't know whether it's getting fake or
 * real data. All bus I/O goes through hal_i2c.h — no DriverLib leaks
 * into this file.
 *
 * Hardware notes:
 *   - Two MPU-6050s share the I2C bus. The lower (hip) IMU has AD0
 *     tied low, giving slave address 0x68. The upper (back) IMU has
 *     AD0 tied high, giving 0x69.
 *   - Default sensitivity after power-on:
 *       Accel : +/- 2g  -> 16384 LSB/g
 *       Gyro  : +/- 250 dps -> 131 LSB/(deg/s)
 *   - The 14-byte read starting at ACCEL_XOUT_H gives all six axes
 *     plus temperature in one transaction. */

#define MPU_ADDR_LOWER          0x68u
#define MPU_ADDR_UPPER          0x69u

#define MPU_REG_PWR_MGMT_1      0x6Bu
#define MPU_REG_WHO_AM_I        0x75u
#define MPU_REG_ACCEL_XOUT_H    0x3Bu

#define MPU_PWR_WAKEUP_VALUE    0x00u    /* clear sleep bit, internal 8 MHz */
#define MPU_WHO_AM_I_EXPECTED   0x68u

#define MPU_BURST_READ_LEN      14u

#define ACCEL_LSB_PER_G         16384.0f
#define GYRO_LSB_PER_DPS          131.0f
#define G_MS2                       9.81f

/* --- helpers ------------------------------------------------------------- */

static int16_t pack_be_i16(uint8_t hi, uint8_t lo) {
    return (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
}

/* Wake one MPU-6050 from sleep. WHO_AM_I check is best-effort: a
 * mismatch is logged via return value but not fatal — Phase 2 device
 * bring-up will surface it through a status LED if needed. */
static bool mpu_init_one(uint8_t addr) {
    uint8_t v = MPU_PWR_WAKEUP_VALUE;
    if (!hal_i2c_write_bytes(addr, MPU_REG_PWR_MGMT_1, &v, 1u)) {
        return false;
    }
    uint8_t who = 0u;
    if (!hal_i2c_read_bytes(addr, MPU_REG_WHO_AM_I, &who, 1u)) {
        return false;
    }
    return who == MPU_WHO_AM_I_EXPECTED;
}

/* Read all 14 bytes and convert to ImuRaw. */
static bool mpu_read_one(uint8_t addr, ImuRaw *out) {
    uint8_t buf[MPU_BURST_READ_LEN];
    if (!hal_i2c_read_bytes(addr, MPU_REG_ACCEL_XOUT_H, buf, MPU_BURST_READ_LEN)) {
        return false;
    }

    int16_t ax_raw = pack_be_i16(buf[0],  buf[1]);
    int16_t ay_raw = pack_be_i16(buf[2],  buf[3]);
    int16_t az_raw = pack_be_i16(buf[4],  buf[5]);
    /* buf[6..7] = temperature, intentionally ignored. */
    int16_t gx_raw = pack_be_i16(buf[8],  buf[9]);
    int16_t gy_raw = pack_be_i16(buf[10], buf[11]);
    int16_t gz_raw = pack_be_i16(buf[12], buf[13]);

    out->ax = ((float)ax_raw / ACCEL_LSB_PER_G) * G_MS2;
    out->ay = ((float)ay_raw / ACCEL_LSB_PER_G) * G_MS2;
    out->az = ((float)az_raw / ACCEL_LSB_PER_G) * G_MS2;
    out->gx = (float)gx_raw / GYRO_LSB_PER_DPS;
    out->gy = (float)gy_raw / GYRO_LSB_PER_DPS;
    out->gz = (float)gz_raw / GYRO_LSB_PER_DPS;
    return true;
}

/* --- imu_source.h implementation ---------------------------------------- */

void imu_source_init(void) {
    hal_i2c_init();
    /* Best-effort wake. Failures here usually mean a wiring problem;
     * the bring-up procedure will catch them on the bench. */
    (void)mpu_init_one(MPU_ADDR_LOWER);
    (void)mpu_init_one(MPU_ADDR_UPPER);
}

bool imu_source_read(ImuRaw *out_lower, ImuRaw *out_upper) {
    if (out_lower == NULL || out_upper == NULL) {
        return false;
    }
    if (!mpu_read_one(MPU_ADDR_LOWER, out_lower)) {
        return false;
    }
    if (!mpu_read_one(MPU_ADDR_UPPER, out_upper)) {
        return false;
    }
    return true;
}
