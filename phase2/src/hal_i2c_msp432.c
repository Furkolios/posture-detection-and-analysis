#include "hal_i2c.h"

/* MSP432 I2C HAL placeholders. Phase 2 keeps these as compile-only
 * stubs so the firmware target builds without DriverLib in CI. The
 * real device build (`make TARGET=msp432`) is where Ahmet Faruk fills
 * these in.
 *
 * Suggested wiring: EUSCI_B0 in master mode at 100 kHz. P1.6 = SDA,
 * P1.7 = SCL on the LaunchPad. */

void hal_i2c_init(void) {
    /* TODO Phase 2 device build:
     *   - GPIO_setAsPeripheralModuleFunctionInputPin(P1.6 / P1.7)
     *   - I2C_initMaster(EUSCI_B0_BASE, &cfg)   // cfg.dataRate = 100 kHz
     *   - I2C_enableModule(EUSCI_B0_BASE)
     */
}

bool hal_i2c_read_bytes(uint8_t slave_addr_7bit,
                        uint8_t reg,
                        uint8_t *buf,
                        size_t n) {
    (void)slave_addr_7bit;
    (void)reg;
    (void)buf;
    (void)n;
    /* TODO Phase 2: I2C_setSlaveAddress + transmitStart + repeatedStart
     * + receive loop. Return false on NACK. */
    return false;
}

bool hal_i2c_write_bytes(uint8_t slave_addr_7bit,
                         uint8_t reg,
                         const uint8_t *buf,
                         size_t n) {
    (void)slave_addr_7bit;
    (void)reg;
    (void)buf;
    (void)n;
    return false;
}
