#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdint.h>
#include <stddef.h>

/* Configure the UART that bridges to the HM-10 BLE module.
 *   baud : line rate in bits per second (HM-10 default is 9600) */
void uart_init(uint32_t baud);

/* Write n bytes to the UART. Blocks until all bytes are queued. */
void uart_write_bytes(const uint8_t *buf, size_t n);

#endif /* HAL_UART_H */
