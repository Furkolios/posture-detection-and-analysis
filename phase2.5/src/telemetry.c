#include <string.h>
#include "telemetry.h"
#include "hal_uart.h"

/* HM-10 default UART rate. */
#define TELEMETRY_BAUD_RATE          9600u

/* Number of bytes covered by V1 / V2 checksums (everything before the
 * checksum byte itself). */
#define TELEMETRY_V1_CHECKSUM_SPAN   6u
#define TELEMETRY_V2_CHECKSUM_SPAN  10u

void telemetry_init(void) {
    uart_init(TELEMETRY_BAUD_RATE);
}

/* --- V1 --- frozen ------------------------------------------------------ */

uint8_t telemetry_compute_checksum(const TelemetryPacket *pkt) {
    const uint8_t *bytes = (const uint8_t *)pkt;
    uint8_t cs = 0u;
    for (size_t i = 0u; i < TELEMETRY_V1_CHECKSUM_SPAN; ++i) {
        cs ^= bytes[i];
    }
    return cs;
}

void telemetry_send(float angle_deg, PostureState state) {
    TelemetryPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.sync      = TELEMETRY_SYNC_BYTE;
    pkt.angle_deg = angle_deg;
    pkt.state     = (uint8_t)state;
    pkt.checksum  = telemetry_compute_checksum(&pkt);
    uart_write_bytes((const uint8_t *)&pkt, TELEMETRY_PACKET_SIZE);
}

/* --- V2 ------------------------------------------------------------------ */

uint8_t telemetry_compute_checksum_v2(const TelemetryPacketV2 *pkt) {
    const uint8_t *bytes = (const uint8_t *)pkt;
    uint8_t cs = 0u;
    for (size_t i = 0u; i < TELEMETRY_V2_CHECKSUM_SPAN; ++i) {
        cs ^= bytes[i];
    }
    return cs;
}

void telemetry_send_v2(float pitch_deg, float roll_deg, PostureState state) {
    TelemetryPacketV2 pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.sync      = TELEMETRY_SYNC_BYTE;
    pkt.pitch_deg = pitch_deg;
    pkt.roll_deg  = roll_deg;
    pkt.state     = (uint8_t)state;
    pkt.checksum  = telemetry_compute_checksum_v2(&pkt);
    uart_write_bytes((const uint8_t *)&pkt, TELEMETRY_PACKET_V2_SIZE);
}
