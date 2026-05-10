#include "hal_uart.h"

/* MSP432 UART driver placeholders. Phase 2 (Ahmet Faruk) will wire
 * EUSCI_A0 to the HM-10 and fill in real DriverLib calls. The pin
 * mapping is tentative: P1.2/P1.3 are the LaunchPad's back-channel
 * UART, but for the wearable we'll use a pair routed to the HM-10. */

void uart_init(uint32_t baud) {
    (void)baud;
    /* TODO Phase 2:
     *   - configure clock source
     *   - compute UCBRx, UCBRFx, UCBRSx for `baud`
     *   - GPIO_setAsPeripheralModuleFunctionInputPin for UART pins
     *   - UART_initModule + UART_enableModule
     */
}

void uart_write_bytes(const uint8_t *buf, size_t n) {
    (void)buf;
    (void)n;
    /* TODO Phase 2:
     *   for (size_t i = 0; i < n; ++i) {
     *       while (!(UART_getInterruptStatus(EUSCI_A0_BASE,
     *                EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG))) { }
     *       UART_transmitData(EUSCI_A0_BASE, buf[i]);
     *   }
     */
}
