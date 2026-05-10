#include <stdio.h>
#include <unistd.h>
#include "hal_uart.h"

/* Host stub. Writes raw packet bytes to stdout so test_phase1.py can
 * pipe them in and parse them. fflush after every write keeps latency
 * reasonable when piped. */

void uart_init(uint32_t baud) {
    (void)baud;  /* meaningless on host, kept for signature parity */
}

void uart_write_bytes(const uint8_t *buf, size_t n) {
    if (buf == NULL || n == 0u) {
        return;
    }
    /* Use the underlying file descriptor to avoid stdio formatting
     * mishaps with binary data. */
    ssize_t written = 0;
    size_t remaining = n;
    while (remaining > 0u) {
        ssize_t w = write(STDOUT_FILENO, buf + written, remaining);
        if (w <= 0) {
            break;  /* pipe closed or interrupted; nothing more we can do */
        }
        written   += w;
        remaining -= (size_t)w;
    }
    fflush(stdout);
}
