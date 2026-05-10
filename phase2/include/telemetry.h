#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "posture_types.h"

/* Sync byte every telemetry packet starts with — same in V1 and V2. */
#define TELEMETRY_SYNC_BYTE  ((uint8_t)0xA5)

/* Wire-format version emitted by telemetry_send_v2(). The legacy
 * telemetry_send() always writes V1. */
#define TELEMETRY_PACKET_VERSION  2

/* Initialise the telemetry subsystem (calls uart_init internally).
 * Call once at startup. */
void telemetry_init(void);

/* --- V1, frozen Phase 1 surface ----------------------------------------- */

/* Build a V1 TelemetryPacket from the current angle and posture state,
 * compute its checksum, and write all 7 bytes to the UART. */
void telemetry_send(float angle_deg, PostureState state);

/* Compute the XOR checksum of bytes [0..5] of a V1 packet. */
uint8_t telemetry_compute_checksum(const TelemetryPacket *pkt);

/* --- V2, Phase 2 ------------------------------------------------------- */

/* Build a V2 TelemetryPacketV2 carrying both pitch and roll, compute
 * its checksum, and write all 11 bytes to the UART. */
void telemetry_send_v2(float pitch_deg, float roll_deg, PostureState state);

/* Compute the XOR checksum of bytes [0..9] of a V2 packet. */
uint8_t telemetry_compute_checksum_v2(const TelemetryPacketV2 *pkt);

#endif /* TELEMETRY_H */
