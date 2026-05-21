# Posture Device — Teammate Receiver Guide

The wearable streams 20 packets per second over BLE. This guide covers how
to get that stream into your laptop and decode it.

## 1. Pairing the HM-10

The HM-10 advertises as a Bluetooth-LE peripheral with a name like `HMSoft`
or `BT05`. After pairing, the OS exposes it as a virtual serial port.

### Windows
1. Settings → Bluetooth & devices → Add device → Bluetooth.
2. Pick `HMSoft` (or whatever name is configured).
3. Open Device Manager → Ports (COM & LPT). A new `COMx` will appear.
4. Use that COM port name in the Python snippet below.

### macOS
1. System Settings → Bluetooth → connect to `HMSoft`.
2. The port lives at `/dev/cu.HMSoft-*`. The exact suffix varies — run
   `ls /dev/cu.*` to see it.

### Linux
1. `sudo bluetoothctl`, then `scan on`, find the HM-10 MAC, `pair <MAC>`,
   `trust <MAC>`.
2. Bind it to an rfcomm device: `sudo rfcomm bind 0 <MAC>` → use
   `/dev/rfcomm0`.
3. Add your user to the `dialout` group (or run with `sudo`) so PySerial
   can open the port.

In all three cases the link is **9600 baud, 8N1, no flow control**.

## 2. Packet format

Each packet is **11 bytes**, little-endian:

| Offset | Size | Type    | Meaning                                       |
|--------|------|---------|-----------------------------------------------|
| 0      | 1    | uint8   | Sync byte, always `0xA5`                      |
| 1–4    | 4    | float32 | `pitch_deg` — forward(+) / backward(-) tilt   |
| 5–8    | 4    | float32 | `roll_deg`  — lateral tilt                    |
| 9      | 1    | uint8   | Posture state (see encoding below)            |
| 10     | 1    | uint8   | XOR of bytes 0–9                              |

Always re-sync on a bad checksum: scan forward one byte at a time until
the next `0xA5` and try again.

## 3. Posture state encoding

| Value | Name                   | Meaning                                     |
|-------|------------------------|---------------------------------------------|
| 0     | `POSTURE_GOOD`         | Within neutral tolerance                    |
| 1     | `POSTURE_MILD_SLOUCH`  | Forward spine curve ≥ 10°                   |
| 2     | `POSTURE_FULL_SLOUCH`  | Forward spine curve ≥ 25°                   |
| 3     | `POSTURE_LEAN_FORWARD` | Sustained forward lean (15° – 24°)          |
| 4     | `POSTURE_LEAN_BACK`    | Backward lean ≥ 15°                         |
| 5     | `POSTURE_LATERAL_TILT` | Lateral (left/right) deviation ≥ 15°        |

## 4. Physical interpretation

- **Lower sensor (I²C `0x68`, `AD0` to GND)** — clipped near the hip /
  pelvis.
- **Upper sensor (I²C `0x69`, `AD0` to V+)** — clipped on the upper back.
- `pitch_deg` and `roll_deg` in the packet are the **relative** orientation
  of the upper sensor with respect to the lower one, then offset by the
  neutral baseline recorded during the **first 2 s** after power-on. So the
  wearer should stand in their target "good" posture during boot — when the
  device beeps once, calibration is done and live classification starts.
- Positive `pitch_deg` ⇒ upper back rotated forward relative to the hips
  (the slouch direction).
- The sign of `roll_deg` depends on how the upper sensor is physically
  mounted — verify empirically by tilting left vs. right.

## 5. Minimal Python receiver

```python
import serial
import struct

PORT = "COM5"           # Windows; use /dev/cu.HMSoft-* on macOS, /dev/rfcomm0 on Linux
BAUD = 9600
SYNC = 0xA5
STATE_NAMES = {
    0: "GOOD",
    1: "MILD_SLOUCH",
    2: "FULL_SLOUCH",
    3: "LEAN_FORWARD",
    4: "LEAN_BACK",
    5: "LATERAL_TILT",
}

def read_packet(ser):
    # Scan forward a byte at a time until we land on the sync byte,
    # then read the rest of the packet and verify the checksum.
    while True:
        b = ser.read(1)
        if not b or b[0] != SYNC:
            continue
        rest = ser.read(10)
        if len(rest) < 10:
            continue
        pkt = b + rest
        xor_sum = 0
        for v in pkt[:10]:
            xor_sum ^= v
        if xor_sum != pkt[10]:
            continue  # bad packet — re-sync on the next 0xA5
        pitch, roll = struct.unpack("<ff", pkt[1:9])
        return pitch, roll, pkt[9]

def main():
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        while True:
            pitch, roll, state = read_packet(ser)
            print(f"pitch={pitch:+6.1f}°  roll={roll:+6.1f}°  state={STATE_NAMES.get(state, '?')}")

if __name__ == "__main__":
    main()
```

If the receiver sees nothing, check that the HM-10's status LED is **solid**
(connected) rather than blinking (still advertising) — only the central
that paired with it will see the data stream.
