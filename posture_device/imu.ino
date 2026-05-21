/*
 * MPU-6050 driver — bring-up and burst register reads over I2C.
 * Both sensors share the Wire bus; only their I2C addresses differ.
 */

#define MPU_REG_PWR_MGMT_1    0x6B
#define MPU_REG_SMPLRT_DIV    0x19
#define MPU_REG_CONFIG        0x1A
#define MPU_REG_GYRO_CONFIG   0x1B
#define MPU_REG_ACCEL_CONFIG  0x1C
#define MPU_REG_ACCEL_XOUT_H  0x3B

// Default full-scale ranges (we don't change them):
//   Accel: +/-2 g       -> 16384 LSB / g
//   Gyro:  +/-250 deg/s -> 131   LSB / (deg/s)
#define MPU_ACCEL_LSB_PER_G    16384.0f
#define MPU_GYRO_LSB_PER_DPS   131.0f
#define G_TO_MS2               9.80665f
#define DEG_TO_RAD_F           0.017453292519943295f

/* Write one byte to an MPU register. */
static void mpuWriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

/* Bring one MPU-6050 out of sleep and set sample rate divider + DLPF. */
void imuInit(uint8_t addr) {
    mpuWriteReg(addr, MPU_REG_PWR_MGMT_1,   0x00);  // wake, internal 8 MHz osc
    mpuWriteReg(addr, MPU_REG_SMPLRT_DIV,   0x07);  // sample rate / 8
    mpuWriteReg(addr, MPU_REG_CONFIG,       0x03);  // DLPF ~44 Hz gyro / 44 Hz accel
    mpuWriteReg(addr, MPU_REG_GYRO_CONFIG,  0x00);  // +/-250 deg/s
    mpuWriteReg(addr, MPU_REG_ACCEL_CONFIG, 0x00);  // +/-2 g
}

/* Burst-read 14 bytes from ACCEL_XOUT_H (accel, temp, gyro) and convert to
 * SI units. Returns false on any I2C error or short read. */
bool imuRead(uint8_t addr, IMUSample *out) {
    Wire.beginTransmission(addr);
    Wire.write(MPU_REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return false;

    uint8_t got = Wire.requestFrom((int)addr, 14);
    if (got < 14) return false;

    int16_t ax = (int16_t)((Wire.read() << 8) | Wire.read());
    int16_t ay = (int16_t)((Wire.read() << 8) | Wire.read());
    int16_t az = (int16_t)((Wire.read() << 8) | Wire.read());
    (void)((Wire.read() << 8) | Wire.read());          // temperature: discard
    int16_t gx = (int16_t)((Wire.read() << 8) | Wire.read());
    int16_t gy = (int16_t)((Wire.read() << 8) | Wire.read());
    int16_t gz = (int16_t)((Wire.read() << 8) | Wire.read());

    out->ax = ((float)ax / MPU_ACCEL_LSB_PER_G) * G_TO_MS2;
    out->ay = ((float)ay / MPU_ACCEL_LSB_PER_G) * G_TO_MS2;
    out->az = ((float)az / MPU_ACCEL_LSB_PER_G) * G_TO_MS2;
    out->gx = ((float)gx / MPU_GYRO_LSB_PER_DPS) * DEG_TO_RAD_F;
    out->gy = ((float)gy / MPU_GYRO_LSB_PER_DPS) * DEG_TO_RAD_F;
    out->gz = ((float)gz / MPU_GYRO_LSB_PER_DPS) * DEG_TO_RAD_F;
    return true;
}
