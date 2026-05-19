# Posture visualizer

Live stick-figure visualizer for the Phase 2.5 telemetry stream.

The app expects the firmware's V2 packet:

```text
0      sync byte, 0xA5
1..4   pitch_deg, little-endian float
5..8   roll_deg, little-endian float
9      posture state
10     XOR checksum over bytes 0..9
```

State values must match `include/posture_types.h`:

```text
0 POSTURE_GOOD
1 POSTURE_MILD_SLOUCH
2 POSTURE_FULL_SLOUCH
3 POSTURE_LEAN_FORWARD
4 POSTURE_LEAN_BACK
5 POSTURE_LATERAL_TILT
```

The parser accepts V2 packets and explicitly drops valid legacy V1
7-byte packets so they do not permanently desync the V2 stream.

## Run

Demo mode, no hardware needed:

```bash
python visualizer/posture_visualizer.py --demo
```

Real HM-10 serial port:

```bash
python visualizer/posture_visualizer.py --port COM3
```

Linux example:

```bash
python visualizer/posture_visualizer.py --port /dev/rfcomm0
```

Install `pyserial` only when reading from real hardware:

```bash
python -m pip install pyserial
```

## Visual mapping

- `pitch` bends the figure forward/backward.
- `roll` shifts the spine sideways.
- State controls the color and label.

Thresholds mirror `include/classifier.h`:

```text
LEAN_FORWARD starts at 5 deg
MILD_SLOUCH starts at 10 deg
FULL_SLOUCH starts at 25 deg
LEAN_BACK starts at -10 deg
LATERAL_TILT starts at abs(roll) >= 15 deg
```
