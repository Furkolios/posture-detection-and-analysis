# Posture Device — Teammate Guide

This README is for **Cemil** and **Ahmet Faruk**. The MSP432 firmware
is a black box from your perspective: it streams bytes over BLE, you
parse them. This document describes that stream and how to receive it.

---

## What the device is

A small wearable with two MPU-6050 IMUs (one on the upper back, one at
the hip), an MSP432 LaunchPad, and an HM-10 BLE module in a belt pouch.
On power-on it:

1. Calibrates for **2 seconds** while the user stands straight.
2. Runs Madgwick fusion on both IMUs at **20 Hz** (50 ms loop).
3. Classifies posture into one of six states.
4. Streams an 11-byte telemetry packet over BLE every 50 ms.
5. Buzzes when posture is bad.

You receive the byte stream. Everything else is internal.

---

## Pairing the HM-10 as a serial port

The HM-10 is a Bluetooth Low Energy module that, once paired, appears
as a **virtual serial port** on the host computer. From your code's
perspective it is exactly a serial connection at 9600 baud.

### Linux

Power the device on, then in a terminal:

```bash
# Find the HM-10
sudo bluetoothctl
# In the bluetoothctl prompt:
scan on
# Wait for a device named "HMSoft" or "HM-10" to appear with its MAC.
pair AA:BB:CC:DD:EE:FF        # use the MAC you saw
trust AA:BB:CC:DD:EE:FF
exit

# Bind it to a serial device:
sudo rfcomm bind 0 AA:BB:CC:DD:EE:FF
# Now /dev/rfcomm0 is your serial port.
```

Then use `/dev/rfcomm0` in the Python snippet below.

### macOS

Open **System Settings → Bluetooth**, click the HM-10 to pair (no PIN
required for the original HM-10; some clones use 000000 or 1234). Once
paired, it appears as `/dev/tty.HMSoft-DevB` or similar. List with:

```bash
ls /dev/tty.* | grep -i hm
```

### Windows

Open **Settings → Devices → Bluetooth & other devices → Add
Bluetooth device**, pick the HM-10. Windows assigns it a COM port
(usually `COM3`, `COM4`, or higher). Find the exact one in **Device
Manager → Ports (COM & LPT)**.

---

## Reading the stream

A minimal Python parser. Drop into a script, change the `PORT`, run.

```python
import serial, struct

PORT = "/dev/rfcomm0"     # Linux:   /dev/rfcomm0
                          # macOS:   /dev/tty.HMSoft-DevB
                          # Windows: COM3 (or whatever Device Manager shows)
BAUD = 9600

STATE_NAMES = {
    0: "GOOD",
    1: "MILD_SLOUCH",
    2: "FULL_SLOUCH",
    3: "LEAN_FORWARD",
    4: "LEAN_BACK",
    5: "LATERAL_TILT",
}

with serial.Serial(PORT, BAUD, timeout=1) as ser:
    while True:
        # 1. Resync to the next 0xA5 sync byte.
        b = ser.read(1)
        if not b or b[0] != 0xA5:
            continue

        # 2. Read the remaining 10 bytes of the packet.
        rest = ser.read(10)
        if len(rest) < 10:
            continue

        # 3. Parse: pitch (float), roll (float), state (uint8), checksum (uint8).
        pitch, roll = struct.unpack_from("<ff", rest, 0)
        state    = rest[8]
        checksum = rest[9]

        # 4. Verify checksum: XOR of bytes [0..9] should equal the checksum byte.
        all_bytes = b + rest
        calc = 0
        for x in all_bytes[:10]:
            calc ^= x
        if calc != checksum:
            continue   # bad packet, skip silently

        print(f"pitch={pitch:6.1f}°  roll={roll:6.1f}°  "
              f"state={STATE_NAMES.get(state, '???')}")
```

Expected output (user standing, then slouching):

```
pitch=  0.2°  roll= -0.1°  state=GOOD
pitch=  0.3°  roll=  0.0°  state=GOOD
pitch=  8.7°  roll= -0.2°  state=LEAN_FORWARD
pitch= 14.5°  roll= -0.3°  state=MILD_SLOUCH
pitch= 27.1°  roll= -0.4°  state=FULL_SLOUCH
```

---

## Wire format reference

Every packet is exactly **11 bytes**, little-endian, sent at ~20 Hz:

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 1 | sync byte | Always `0xA5`. Use to resync if you fall out of frame. |
| 1 | 4 | `pitch_deg` | IEEE-754 float, little-endian. |
| 5 | 4 | `roll_deg`  | IEEE-754 float, little-endian. |
| 9 | 1 | `state`    | One of the 6 values in the table below. |
| 10 | 1 | checksum   | XOR of bytes `[0..9]`. |

### State encoding

| Value | Constant | Meaning |
|---|---|---|
| 0 | `GOOD` | User is in or close to their calibrated neutral pose. |
| 1 | `MILD_SLOUCH` | Forward bend ≥ 10° from neutral. |
| 2 | `FULL_SLOUCH` | Forward bend ≥ 25° from neutral. |
| 3 | `LEAN_FORWARD` | Small intentional forward bow (< MILD threshold but off-neutral). |
| 4 | `LEAN_BACK`    | Backward lean ≥ 10° from neutral. |
| 5 | `LATERAL_TILT` | Sideways tilt ≥ 15° (overrides any forward/back judgment). |

State values 0–2 are the same as Phase 1's three-state encoding for
backward compatibility with any old code.

---

## What pitch and roll mean physically

Two IMUs, one on the **hip** ("lower") and one on the **upper back**
("upper"). The firmware computes the *relative* orientation of the
upper IMU as seen from the lower IMU's frame, then extracts:

- **`pitch_deg`** — forward/backward spinal bend.
  - **Positive** = bending **forward** (slouch direction).
  - **Negative** = leaning **backward**.
- **`roll_deg`** — lateral spinal tilt.
  - **Positive** = tilting to one side, **negative** = the other.
  - The exact sign convention depends on which way the IMU was
    soldered to its breakout; expect to confirm empirically during
    bring-up. The visualizer can flip the sign if it's backward.

Both angles are **relative to the user's calibrated neutral pose**, not
absolute. So `pitch=0` doesn't mean "back is vertical" — it means "back
is in the same orientation it was during calibration." That's the
right thing for a posture coach: we care about deviation from the
user's chosen neutral.

---

## The calibration window

For the **first 2 seconds** after the device powers on, packets are
still flowing but the firmware is recording the user's neutral
orientation. **The user must be standing as straight as possible**
during this window. Whatever orientation the IMUs see at the end of
the 2 seconds becomes "neutral pitch = 0, neutral roll = 0."

During calibration the device sends `state = GOOD` and the
still-converging pitch/roll values. Cemil's visualizer can either:
- ignore the first ~2 seconds of packets, or
- show a "calibrating, hold still" indicator that disappears when
  pitch and roll settle.

There is no end-of-calibration marker in the wire format — sorry. If
that turns out to be inconvenient, ping me and I'll add one (would
need to be additive to keep V2 stable, e.g. a `calibration_done`
state value or a sync byte variant).

---

## Known limitations (be patient with these)

- **Constants are tuned on synthetic data, not real bodies.** The
  thresholds (10°, 25°, 15°) and the Madgwick `beta` gain (0.1) are
  reasonable starting values from the literature. After real-world
  testing we may need to retune them. If your visualizer shows the
  user clearly slouched but the firmware says `GOOD`, that's a
  threshold issue, not a parser bug — let me know.

- **Small post-calibration drift.** Madgwick's gyro integration drifts
  a few degrees per minute. The accelerometer correction term cancels
  this out *eventually* but a long static session (e.g. user sits
  perfectly still for 5 minutes) might show a degree or two of slow
  baseline drift. This is normal. For longer sessions consider
  adding a "recalibrate" button to the visualizer; the firmware
  side of that is a feature for later.

- **`LEAN_FORWARD` vs `MILD_SLOUCH`.** With the rule-based classifier
  these two states are basically "small forward bend" vs "bigger
  forward bend." The proposal mentions a Phase 3 ML extension to
  distinguish *intentional* forward leans (reading, working at a
  desk) from *unintentional* slouching. For now, treat `LEAN_FORWARD`
  as a soft warning and `MILD_SLOUCH`/`FULL_SLOUCH` as harder ones.

- **BLE drop-outs.** If the serial read returns 0 bytes for a few
  hundred ms, BLE is fine — that's normal for HM-10 packet pacing.
  If it stays silent for >1 second, the device is probably
  out-of-range or has lost power. Resync by reading until you see
  another `0xA5`.

---

## Sanity-checking with a known-good script

Before plugging into the visualizer, confirm the device is working
with the minimal script above. Power on, wait 2 seconds, then watch
the printed values:

- Standing straight: pitch ≈ 0, roll ≈ 0, state = GOOD.
- Slouching forward: pitch climbs to ~15°-30°, state goes
  GOOD → LEAN_FORWARD → MILD_SLOUCH → FULL_SLOUCH as the bend deepens.
- Tilting head/shoulders sideways: roll grows, state = LATERAL_TILT.

If those don't match, the wiring (which IMU is which) might be swapped
or the IMU axes might need a sign flip. Tell Furkan and we'll look at
it together.

---

## Questions / debugging

If telemetry is silent, in priority order, check:
1. Is the device powered (LED on the LaunchPad lit)?
2. Is the HM-10 paired and the serial port name correct?
3. Are you reading at **9600 baud**, not 115200?
4. Does the byte you see have the value `0xA5`? If not, you're
   probably reading the wrong port.

If telemetry is flowing but the values are nonsense (NaN, huge
numbers, state always 0), that's almost always a wiring problem with
the IMUs — let Furkan/Faruk debug on the bench before the visualizer
side spends time on it.
