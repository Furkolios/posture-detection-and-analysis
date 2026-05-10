/*
 * hal_i2c_msp432.c — EUSCI_B1 I2C master @ 100 kHz.
 *
 * Pin choice: P6.4 = SDA, P6.5 = SCL (LaunchPad J3.10 / J3.9). These
 * are EUSCI_B1's I2C function pins. Module choice rationale: leaves
 * EUSCI_B0 free for any future booster-pack peripheral and avoids the
 * back-channel UART module group.
 *
 * Clock source: SMCLK at 12 MHz. byteCounterThreshold = 0 means we
 * manage byte counts manually (DriverLib's automatic stop generation
 * needs a known length up front, which we have). For a 100 kHz bus
 * clock from a 12 MHz source, dataRate = 12_000_000 / 100_000 = 120
 * is what DriverLib infers from EUSCI_B_I2C_SET_DATA_RATE_100KBPS.
 *
 * Bus protocol (MPU-6050 register read):
 *   START | addr|W | reg | RESTART | addr|R | data... | STOP
 *
 * NACK detection: every transmit must be followed by a check of the
 * NACK interrupt flag. If asserted, we abort the transaction (send
 * STOP and return false) so the bus is left in a clean state.
 *
 * Pull-ups: the LaunchPad does NOT have I2C pull-ups built in. You
 * MUST add 4.7kΩ pull-ups from SDA and SCL to 3.3V on the breadboard
 * unless your MPU breakout already has them. Without pull-ups the bus
 * stays low and every transaction will hang.
 */

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "hal_i2c.h"

#define I2C_BASE        EUSCI_B1_BASE
#define I2C_PORT        GPIO_PORT_P6
#define I2C_PINS        (GPIO_PIN4 | GPIO_PIN5)
#define I2C_PIN_FUNC    GPIO_PRIMARY_MODULE_FUNCTION

/* Generous timeout for any single byte transfer. At 100 kHz a byte
 * takes ~90 µs nominally; a slave that's stretching the clock heavily
 * shouldn't exceed a few ms. 50 ms is well over the worst case and
 * prevents the firmware locking up on a wiring fault. The unit is
 * "polling iterations" — calibrated empirically; tighten if needed. */
#define I2C_BUSY_TIMEOUT_ITERS  500000u

static const eUSCI_I2C_MasterConfig s_i2c_cfg = {
    EUSCI_B_I2C_CLOCKSOURCE_SMCLK,
    12000000u,                                  /* SMCLK = 12 MHz */
    EUSCI_B_I2C_SET_DATA_RATE_100KBPS,
    0,                                          /* byteCounterThreshold */
    EUSCI_B_I2C_NO_AUTO_STOP
};

void hal_i2c_init(void) {
    GPIO_setAsPeripheralModuleFunctionInputPin(I2C_PORT, I2C_PINS, I2C_PIN_FUNC);
    I2C_initMaster(I2C_BASE, &s_i2c_cfg);
    I2C_enableModule(I2C_BASE);
}

/* --- helpers ------------------------------------------------------------ */

/* Wait for an arbitrary EUSCI flag (or NACK, whichever comes first).
 * Returns true if the wanted flag came up before NACK and before
 * timeout; false otherwise. The NACK and timeout cases are the
 * caller's responsibility to clean up (usually with a STOP). */
static bool wait_flag_or_nack(uint16_t flag) {
    for (uint32_t i = 0u; i < I2C_BUSY_TIMEOUT_ITERS; ++i) {
        uint_fast16_t status = I2C_getInterruptStatus(I2C_BASE,
                                   flag | EUSCI_B_I2C_NAK_INTERRUPT);
        if (status & EUSCI_B_I2C_NAK_INTERRUPT) {
            I2C_clearInterruptFlag(I2C_BASE, EUSCI_B_I2C_NAK_INTERRUPT);
            return false;
        }
        if (status & flag) {
            I2C_clearInterruptFlag(I2C_BASE, flag);
            return true;
        }
    }
    return false;
}

/* --- public API --------------------------------------------------------- */

bool hal_i2c_read_bytes(uint8_t slave_addr_7bit,
                        uint8_t reg,
                        uint8_t *buf,
                        size_t n) {
    if (buf == NULL || n == 0u) {
        return false;
    }

    I2C_setSlaveAddress(I2C_BASE, slave_addr_7bit);
    I2C_setMode(I2C_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    I2C_clearInterruptFlag(I2C_BASE,
        EUSCI_B_I2C_TRANSMIT_INTERRUPT0 |
        EUSCI_B_I2C_RECEIVE_INTERRUPT0  |
        EUSCI_B_I2C_NAK_INTERRUPT       |
        EUSCI_B_I2C_STOP_INTERRUPT);

    /* --- Phase 1: transmit register address ---------------------------- */
    I2C_masterSendStart(I2C_BASE);
    if (!wait_flag_or_nack(EUSCI_B_I2C_TRANSMIT_INTERRUPT0)) {
        I2C_masterSendMultiByteStop(I2C_BASE);
        return false;
    }
    I2C_masterSendMultiByteNext(I2C_BASE, reg);
    if (!wait_flag_or_nack(EUSCI_B_I2C_TRANSMIT_INTERRUPT0)) {
        I2C_masterSendMultiByteStop(I2C_BASE);
        return false;
    }

    /* --- Phase 2: repeated start, read N bytes ------------------------- */
    I2C_setMode(I2C_BASE, EUSCI_B_I2C_RECEIVE_MODE);
    I2C_masterReceiveStart(I2C_BASE);

    for (size_t i = 0u; i < n; ++i) {
        if (i == n - 1u) {
            /* Issue STOP before reading the last byte (EUSCI quirk:
             * the stop must be queued before the last RX flag fires). */
            I2C_masterReceiveMultiByteStop(I2C_BASE);
        }
        if (!wait_flag_or_nack(EUSCI_B_I2C_RECEIVE_INTERRUPT0)) {
            return false;
        }
        buf[i] = I2C_masterReceiveMultiByteNext(I2C_BASE);
    }
    return true;
}

bool hal_i2c_write_bytes(uint8_t slave_addr_7bit,
                         uint8_t reg,
                         const uint8_t *buf,
                         size_t n) {
    if (buf == NULL || n == 0u) {
        return false;
    }

    I2C_setSlaveAddress(I2C_BASE, slave_addr_7bit);
    I2C_setMode(I2C_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    I2C_clearInterruptFlag(I2C_BASE,
        EUSCI_B_I2C_TRANSMIT_INTERRUPT0 |
        EUSCI_B_I2C_NAK_INTERRUPT       |
        EUSCI_B_I2C_STOP_INTERRUPT);

    /* START + register address. */
    I2C_masterSendStart(I2C_BASE);
    if (!wait_flag_or_nack(EUSCI_B_I2C_TRANSMIT_INTERRUPT0)) {
        I2C_masterSendMultiByteStop(I2C_BASE);
        return false;
    }
    I2C_masterSendMultiByteNext(I2C_BASE, reg);
    if (!wait_flag_or_nack(EUSCI_B_I2C_TRANSMIT_INTERRUPT0)) {
        I2C_masterSendMultiByteStop(I2C_BASE);
        return false;
    }

    /* Payload. */
    for (size_t i = 0u; i < n - 1u; ++i) {
        I2C_masterSendMultiByteNext(I2C_BASE, buf[i]);
        if (!wait_flag_or_nack(EUSCI_B_I2C_TRANSMIT_INTERRUPT0)) {
            I2C_masterSendMultiByteStop(I2C_BASE);
            return false;
        }
    }
    /* Final byte + STOP. */
    I2C_masterSendMultiByteFinish(I2C_BASE, buf[n - 1u]);

    /* Wait for STOP to fully clock out before returning, so the bus
     * is idle when the next transaction begins. */
    while (I2C_masterIsStopSent(I2C_BASE) == EUSCI_B_I2C_SENDING_STOP) {
        /* spin */
    }
    return true;
}
