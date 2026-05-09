#include <string.h>
#include "telemetry.h"
#include "hal_uart.h"

/* HM-10 default UART rate. Configurable later via AT+BAUD if needed. */
#define TELEMETRY_BAUD_RATE  9600u

/* Number of bytes covered by the checksum (everything before the
 * checksum byte itself). */
#define TELEMETRY_CHECKSUM_SPAN  6u

void telemetry_init(void) {
    uart_init(TELEMETRY_BAUD_RATE);
}

uint8_t telemetry_compute_checksum(const TelemetryPacket *pkt) {
    const uint8_t *bytes = (const uint8_t *)pkt;
    uint8_t cs = 0u;
    for (size_t i = 0u; i < TELEMETRY_CHECKSUM_SPAN; ++i) {
        cs ^= bytes[i];
    }
    return cs;
}

void telemetry_send(float angle_deg, PostureState state) {
    TelemetryPacket pkt;
    /* Zero the struct so any padding (there should be none thanks to
     * __attribute__((packed)), but defence in depth) doesn't leak. */
    memset(&pkt, 0, sizeof(pkt));

    pkt.sync      = TELEMETRY_SYNC_BYTE;
    pkt.angle_deg = angle_deg;
    pkt.state     = (uint8_t)state;
    pkt.checksum  = telemetry_compute_checksum(&pkt);

    uart_write_bytes((const uint8_t *)&pkt, TELEMETRY_PACKET_SIZE);
}
