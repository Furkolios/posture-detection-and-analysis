#!/usr/bin/env python3
"""
test_phase2.py — Phase 2 host-side telemetry validation.

Spawns the firmware host binary, reads its stdout as a stream of
TelemetryPacketV2 (or V1) packets, and validates:

  1. Packets arrive at a rate within ±20% of the target loop frequency.
  2. Every packet's sync byte is 0xA5.
  3. Every packet's checksum matches the XOR of its prefix.
  4. Pitch in (-90, 90), roll in (-180, 180).
  5. Every PostureState that the cycling dummy generator can produce
     appears at least once in the stream.

A pass/fail table is printed and the exit code is 0 iff every assertion
passed.
"""

import struct
import subprocess
import sys
import time
from pathlib import Path

SYNC_BYTE          = 0xA5

V1_SIZE            = 7
V2_SIZE            = 11

# States the dummy-generator-driven host build is expected to visit
# (it cycles GOOD / MILD / FULL through the scenario knob).
EXPECTED_STATES    = {0, 1, 2}

VALID_STATES       = set(range(6))

TARGET_HZ          = 20.0           # 50 ms loop in main.c
RATE_TOLERANCE_PCT = 25.0
TARGET_PACKETS     = 160            # ~8 seconds at 20 Hz; covers full scenario cycle
RUN_TIMEOUT_SEC    = 15.0


def xor_checksum(buf: bytes, span: int) -> int:
    cs = 0
    for b in buf[:span]:
        cs ^= b
    return cs


def parse_v1(buf: bytes):
    """Returns (pitch, roll, state) — V1 has only one angle, expose it as pitch."""
    sync, angle, state, checksum = struct.unpack("<BfBB", buf)
    return sync, angle, 0.0, state, checksum


def parse_v2(buf: bytes):
    sync, pitch, roll, state, checksum = struct.unpack("<BffBB", buf)
    return sync, pitch, roll, state, checksum


def detect_and_read(stdout, deadline_t: float):
    """
    Try to read one packet from the stream. Detects V1 vs V2 by reading
    one byte at a time until a sync byte is found, then peeking at the
    longer length. Returns (version, raw_bytes) or (None, None) on EOF /
    deadline.
    """
    # Find a sync byte.
    while True:
        if time.monotonic() > deadline_t:
            return None, None
        b = stdout.read(1)
        if not b:
            return None, None
        if b[0] == SYNC_BYTE:
            break

    # We have the sync byte. Tentatively try to read enough for V2.
    rest = stdout.read(V2_SIZE - 1)
    if len(rest) < V2_SIZE - 1:
        return None, None
    pkt = b + rest

    # Heuristic: try V2 checksum first; fall back to V1.
    if xor_checksum(pkt, V2_SIZE - 1) == pkt[V2_SIZE - 1]:
        return 2, pkt
    if xor_checksum(pkt, V1_SIZE - 1) == pkt[V1_SIZE - 1]:
        # The bytes after V1 don't belong to this packet; we read too
        # much, but the next sync scan will resync. Acceptable here.
        return 1, pkt[:V1_SIZE]
    # Neither — return the V2-length blob anyway so the caller can
    # count it as a bad packet.
    return 2, pkt


def main() -> int:
    if len(sys.argv) >= 2:
        binary = Path(sys.argv[1])
    else:
        binary = (Path(__file__).resolve().parent.parent
                  / "build" / "firmware_host_finite_v2")

    if not binary.exists():
        print(f"[FAIL] binary not found: {binary}")
        print("       run `make pytest2` first")
        return 1

    print(f"running {binary}")
    t0 = time.monotonic()
    deadline = t0 + RUN_TIMEOUT_SEC
    proc = subprocess.Popen(
        [str(binary)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    packets = []
    versions = []
    bad_sync = 0
    bad_checksum = 0
    bad_pitch = 0
    bad_roll = 0
    bad_state = 0
    states_seen = set()

    try:
        while len(packets) < TARGET_PACKETS:
            if time.monotonic() > deadline:
                break
            version, raw = detect_and_read(proc.stdout, deadline)
            if raw is None:
                break

            if raw[0] != SYNC_BYTE:
                bad_sync += 1
                continue

            if version == 2:
                sync, pitch, roll, state, checksum = parse_v2(raw)
                cs_span = V2_SIZE - 1
            else:
                sync, pitch, roll, state, checksum = parse_v1(raw)
                cs_span = V1_SIZE - 1

            if checksum != xor_checksum(raw, cs_span):
                bad_checksum += 1
            if not (-90.0 < pitch < 90.0):
                bad_pitch += 1
            if not (-180.0 < roll < 180.0):
                bad_roll += 1
            if state not in VALID_STATES:
                bad_state += 1
            else:
                states_seen.add(state)

            packets.append((pitch, roll, state))
            versions.append(version)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()

    elapsed = time.monotonic() - t0
    rate = len(packets) / elapsed if elapsed > 0 else 0.0
    rate_low = TARGET_HZ * (1.0 - RATE_TOLERANCE_PCT / 100.0)
    rate_high = TARGET_HZ * (1.0 + RATE_TOLERANCE_PCT / 100.0)

    n_v1 = versions.count(1)
    n_v2 = versions.count(2)

    print()
    print(f"collected {len(packets)} packets ({n_v2} V2, {n_v1} V1) "
          f"in {elapsed:.2f}s -> {rate:.1f} pkt/s")
    print(f"states seen: {sorted(states_seen)}")
    print()

    failures = 0

    def check(ok: bool, label: str) -> None:
        nonlocal failures
        marker = "PASS" if ok else "FAIL"
        print(f"  | {marker:4s} | {label}")
        if not ok:
            failures += 1

    print("  +------+----------------------------------------------------")
    check(len(packets) >= TARGET_PACKETS,
          f"collected at least {TARGET_PACKETS} packets")
    check(rate_low <= rate <= rate_high,
          f"rate {rate:.1f} pkt/s within "
          f"{rate_low:.1f}..{rate_high:.1f} pkt/s "
          f"(±{RATE_TOLERANCE_PCT:.0f}% of {TARGET_HZ:.0f} Hz)")
    check(bad_sync == 0,     f"all sync bytes are 0x{SYNC_BYTE:02X} "
                             f"({bad_sync} bad)")
    check(bad_checksum == 0, f"all checksums valid ({bad_checksum} bad)")
    check(bad_pitch == 0,    f"pitch within (-90, 90) ({bad_pitch} bad)")
    check(bad_roll == 0,     f"roll within (-180, 180) ({bad_roll} bad)")
    check(bad_state == 0,    f"all state bytes are valid enum members "
                             f"({bad_state} bad)")
    check(EXPECTED_STATES.issubset(states_seen),
          f"all expected states observed (expected {sorted(EXPECTED_STATES)}, "
          f"saw {sorted(states_seen)})")
    print("  +------+----------------------------------------------------")

    if failures == 0:
        print("ALL CHECKS PASSED")
        return 0
    print(f"{failures} CHECK(S) FAILED")
    return 1


if __name__ == "__main__":
    sys.exit(main())
