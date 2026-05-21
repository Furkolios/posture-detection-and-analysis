/*
 * Telemetry — assemble and send the 11-byte packet over Serial1 (HM-10).
 *
 * Packet layout (all little-endian):
 *   [0]    0xA5  sync
 *   [1-4]  pitch_deg (float32)
 *   [5-8]  roll_deg  (float32)
 *   [9]    state (uint8, 0-5)
 *   [10]   checksum (XOR of bytes 0..9)
 */

#define BLE_BAUD       9600
#define PKT_SYNC_BYTE  0xA5
#define PKT_SIZE       11

/* Bring up the BLE-side UART. HM-10 default is 9600/8N1. */
void telemetryInit() {
    Serial1.begin(BLE_BAUD);
}

/* Pack one frame and push it in a single write() call. The Cortex-M4 on
 * the MSP432 is little-endian IEEE-754, matching the wire format, so memcpy
 * is enough — no manual byte shuffling. */
void telemetrySend(float pitch_deg, float roll_deg, PostureState state) {
    uint8_t pkt[PKT_SIZE];

    pkt[0] = PKT_SYNC_BYTE;
    memcpy(&pkt[1], &pitch_deg, 4);
    memcpy(&pkt[5], &roll_deg,  4);
    pkt[9] = (uint8_t)state;

    uint8_t xor_sum = 0;
    for (uint8_t i = 0; i < 10; i++) xor_sum ^= pkt[i];
    pkt[10] = xor_sum;

    Serial1.write(pkt, PKT_SIZE);
}
