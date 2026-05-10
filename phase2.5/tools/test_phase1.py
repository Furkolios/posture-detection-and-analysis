#!/usr/bin/env python3
"""
test_phase1.py — Phase 1 host-side telemetry validation script.

Spawns the firmware host binary (build/firmware_host), reads its stdout
as a stream of fixed-width 7-byte telemetry packets, and validates:

  1. Packets arrive at a reasonable rate.
  2. Every packet's sync byte is 0xA5.
  3. Every packet's checksum matches the XOR of bytes [0..5].
  4. Angle is finite and within a plausible range.
  5. State byte is a valid PostureState enum value.

Pass/fail summary is printed; exit code is 0 iff every check passed.

Usage:
    python3 tools/test_phase1.py [path/to/firmware_host]
"""

import struct
import subprocess
import sys
import time
from pathlib import Path

PACKET_SIZE        = 7
SYNC_BYTE          = 0xA5
VALID_STATES       = {0, 1, 2}
ANGLE_PLAUSIBLE_DEG = 90.0       # |angle| should never exceed this
TARGET_PACKETS     = 30          # stop after collecting this many
RATE_TIMEOUT_SEC   = 10.0        # fail if we can't collect them in this
MIN_PACKETS_PER_SEC = 5.0        # 50 ms loop -> 20 Hz; allow generous floor


def parse_packet(buf: bytes):
    """Unpack a 7-byte packet. Returns (sync, angle, state, checksum)."""
    return struct.unpack("<BfBB", buf)


def xor_checksum(buf: bytes) -> int:
    cs = 0
    for b in buf[:6]:
        cs ^= b
    return cs


def main() -> int:
    if len(sys.argv) >= 2:
        binary = Path(sys.argv[1])
    else:
        binary = Path(__file__).resolve().parent.parent / "build" / "firmware_host"

    if not binary.exists():
        print(f"[FAIL] binary not found: {binary}")
        print("       run `make host` first")
        return 1

    # Tell the firmware to terminate after TARGET_PACKETS iterations.
    env = {"PHASE1_MAIN_ITERATIONS": str(TARGET_PACKETS)}
    # Note: PHASE1_MAIN_ITERATIONS is a compile-time macro; the Makefile's
    # `host-test` target builds with -DPHASE1_MAIN_ITERATIONS=N. If you want
    # this script to control it dynamically, swap the macro for getenv.
    # For now we rely on the Makefile to have set a finite value.

    print(f"running {binary}")
    t0 = time.monotonic()
    proc = subprocess.Popen(
        [str(binary)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    packets = []
    bad_sync = 0
    bad_checksum = 0
    bad_angle = 0
    bad_state = 0

    try:
        while len(packets) < TARGET_PACKETS:
            if time.monotonic() - t0 > RATE_TIMEOUT_SEC:
                break
            buf = proc.stdout.read(PACKET_SIZE)
            if not buf or len(buf) < PACKET_SIZE:
                break

            sync, angle, state, checksum = parse_packet(buf)

            if sync != SYNC_BYTE:
                bad_sync += 1
            expected_cs = xor_checksum(buf)
            if checksum != expected_cs:
                bad_checksum += 1
            if not (-ANGLE_PLAUSIBLE_DEG <= angle <= ANGLE_PLAUSIBLE_DEG):
                bad_angle += 1
            if state not in VALID_STATES:
                bad_state += 1

            packets.append((sync, angle, state, checksum))
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()

    elapsed = time.monotonic() - t0
    rate = len(packets) / elapsed if elapsed > 0 else 0.0

    print()
    print(f"collected {len(packets)} packet(s) in {elapsed:.2f}s "
          f"({rate:.1f} pkt/s)")

    failures = 0
    def check(ok: bool, label: str) -> None:
        nonlocal failures
        if ok:
            print(f"  [PASS] {label}")
        else:
            print(f"  [FAIL] {label}")
            failures += 1

    check(len(packets) >= TARGET_PACKETS,
          f"collected at least {TARGET_PACKETS} packets")
    check(rate >= MIN_PACKETS_PER_SEC,
          f"rate >= {MIN_PACKETS_PER_SEC} pkt/s")
    check(bad_sync == 0,    f"all sync bytes are 0x{SYNC_BYTE:02X} ({bad_sync} bad)")
    check(bad_checksum == 0, f"all checksums valid ({bad_checksum} bad)")
    check(bad_angle == 0,   f"all angles within +/- {ANGLE_PLAUSIBLE_DEG}° "
                            f"({bad_angle} bad)")
    check(bad_state == 0,   f"all states are valid enum members ({bad_state} bad)")

    print()
    if failures == 0:
        print("ALL CHECKS PASSED")
        return 0
    print(f"{failures} CHECK(S) FAILED")
    return 1


if __name__ == "__main__":
    sys.exit(main())
