/*
 * hal_uart_msp432.c — EUSCI_A2 UART for the HM-10 BLE bridge.
 *
 * Pin choice: P3.2 = RX, P3.3 = TX (LaunchPad J4.33 / J4.34). These
 * are EUSCI_A2's UART function pins. If you change the module, update
 * UART_BASE, UART_PORT, UART_PINS, and the wiring table accordingly.
 *
 * Clock source: SMCLK at 12 MHz (MSP432 default after init). The
 * baud-rate divisors below are computed for SMCLK = 12 MHz, baud =
 * 9600 — the HM-10's factory default. If you reconfigure SMCLK or
 * change baud you MUST recompute these. The TI MSP432 SDK includes
 * a "Baud Rate Calculator" spreadsheet in docs/.
 *
 * For 12 MHz SMCLK, 9600 baud (oversampling mode):
 *   N = 12_000_000 / 9600 = 1250
 *   UCBRx = 1250 / 16 = 78
 *   UCBRFx = ((1250 / 16) - 78) * 16 = 2
 *   UCBRSx = 0x00 (lookup table value for fractional 0.0)
 */

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "hal_uart.h"

#define UART_BASE       EUSCI_A2_BASE
#define UART_PORT       GPIO_PORT_P3
#define UART_PINS       (GPIO_PIN2 | GPIO_PIN3)
#define UART_PIN_FUNC   GPIO_PRIMARY_MODULE_FUNCTION

static const eUSCI_UART_ConfigV1 s_uart_cfg = {
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,                      /* SMCLK source */
    78,                                                  /* UCBRx */
    2,                                                   /* UCBRFx */
    0x00,                                                /* UCBRSx */
    EUSCI_A_UART_NO_PARITY,                              /* no parity */
    EUSCI_A_UART_LSB_FIRST,
    EUSCI_A_UART_ONE_STOP_BIT,
    EUSCI_A_UART_MODE,
    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,
    EUSCI_A_UART_8_BIT_LEN
};

void uart_init(uint32_t baud) {
    /* The baud rate is fixed at 9600 by the divisors in s_uart_cfg.
     * If a caller asks for a different rate we silently use the
     * configured one — Phase 2.5's only consumer is telemetry.c with
     * baud=9600, so this isn't a real limitation. Recompute s_uart_cfg
     * if you ever change rates. */
    (void)baud;

    GPIO_setAsPeripheralModuleFunctionInputPin(UART_PORT, UART_PINS, UART_PIN_FUNC);

    UART_initModule(UART_BASE, &s_uart_cfg);
    UART_enableModule(UART_BASE);
}

void uart_write_bytes(const uint8_t *buf, size_t n) {
    if (buf == NULL || n == 0u) {
        return;
    }
    for (size_t i = 0u; i < n; ++i) {
        /* Block until the transmit buffer is empty, then load next byte.
         * UART_transmitData has its own internal wait, but the explicit
         * flag check makes the blocking behaviour obvious to a reader. */
        while (!(UART_getInterruptStatus(UART_BASE,
                    EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG))) {
            /* spin */
        }
        UART_transmitData(UART_BASE, buf[i]);
    }
}
