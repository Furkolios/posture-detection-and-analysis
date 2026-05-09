#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "posture_types.h"

/* Sync byte every telemetry packet starts with.
 * 0xA5 = 0b10100101 — chosen for an easy bit-pattern visual on a logic
 * analyser and so it's unlikely to appear inside a float payload. */
#define TELEMETRY_SYNC_BYTE  ((uint8_t)0xA5)

/* Initialise the telemetry subsystem (calls uart_init internally).
 * Call once at startup. */
void telemetry_init(void);

/* Build a TelemetryPacket from the current angle and posture state,
 * compute its checksum, and write all 7 bytes to the UART. */
void telemetry_send(float angle_deg, PostureState state);

/* Compute the XOR checksum over the first 6 bytes of a packet.
 * Exposed so the test harness can verify packets without duplicating
 * the formula. */
uint8_t telemetry_compute_checksum(const TelemetryPacket *pkt);

#endif /* TELEMETRY_H */
