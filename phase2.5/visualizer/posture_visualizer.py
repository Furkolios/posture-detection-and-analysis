#!/usr/bin/env python3
"""Live posture visualizer for the Phase 2.5 telemetry stream.

The firmware emits V2 packets by default:
    0xA5 + pitch(float32 LE) + roll(float32 LE) + state(uint8) + xor checksum

This app reads those packets from an HM-10 serial port and draws a stick
figure whose spine bends with pitch and leans sideways with roll. If no
serial port is provided, it runs a deterministic demo signal.
"""

from __future__ import annotations

import argparse
import math
import struct
import time
import tkinter as tk
from dataclasses import dataclass
from typing import Optional

try:
    import serial  # type: ignore
except ImportError:  # pyserial is optional for demo mode.
    serial = None


SYNC_BYTE = 0xA5
V1_SIZE = 7
V2_SIZE = 11
BAUD_RATE = 9600

THRESHOLD_MILD_DEG = 10.0
THRESHOLD_FULL_DEG = 25.0
THRESHOLD_LEAN_BACK_DEG = -10.0
THRESHOLD_LATERAL_DEG = 15.0
LEAN_FORWARD_MIN_DEG = THRESHOLD_MILD_DEG * 0.5
MAX_VISUAL_SLOUCH_DEG = 32.0

STATE_NAMES = {
    0: "GOOD",
    1: "MILD_SLOUCH",
    2: "FULL_SLOUCH",
    3: "LEAN_FORWARD",
    4: "LEAN_BACK",
    5: "LATERAL_TILT",
}

STATE_LABELS = {
    0: "Dik",
    1: "Hafif kambur",
    2: "Belirgin kambur",
    3: "One egilme",
    4: "Arkaya egilme",
    5: "Yana egilme",
}

STATE_COLORS = {
    0: "#2f9e44",
    1: "#f08c00",
    2: "#e03131",
    3: "#f59f00",
    4: "#1971c2",
    5: "#7048e8",
}


@dataclass
class TelemetrySample:
    pitch: float
    roll: float
    state: int
    source: str
    v1_ignored: int = 0


class TelemetryParser:
    """Incremental parser that accepts V2 packets and explicitly drops V1."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.v1_ignored = 0
        self.bad_packets = 0

    @staticmethod
    def _checksum(data: bytes | bytearray) -> int:
        value = 0
        for b in data:
            value ^= b
        return value

    def feed(self, data: bytes) -> list[TelemetrySample]:
        self._buf.extend(data)
        samples: list[TelemetrySample] = []

        while True:
            sync_index = self._buf.find(bytes([SYNC_BYTE]))
            if sync_index < 0:
                self._buf.clear()
                break
            if sync_index > 0:
                del self._buf[:sync_index]
            if len(self._buf) < V1_SIZE:
                break

            if len(self._buf) >= V2_SIZE:
                candidate = self._buf[:V2_SIZE]
                state = candidate[9]
                if state in STATE_NAMES and self._checksum(candidate[:10]) == candidate[10]:
                    pitch, roll = struct.unpack_from("<ff", candidate, 1)
                    samples.append(
                        TelemetrySample(
                            pitch=pitch,
                            roll=roll,
                            state=state,
                            source="V2",
                            v1_ignored=self.v1_ignored,
                        )
                    )
                    del self._buf[:V2_SIZE]
                    continue

            candidate_v1 = self._buf[:V1_SIZE]
            if self._checksum(candidate_v1[:6]) == candidate_v1[6]:
                self.v1_ignored += 1
                del self._buf[:V1_SIZE]
                continue

            if len(self._buf) < V2_SIZE:
                break

            self.bad_packets += 1
            del self._buf[0]

        return samples


class SerialTelemetrySource:
    def __init__(self, port: str) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run demo mode or install pyserial.")
        self._ser = serial.Serial(port, BAUD_RATE, timeout=0)
        self._parser = TelemetryParser()

    def read_latest(self) -> Optional[TelemetrySample]:
        waiting = self._ser.in_waiting
        data = self._ser.read(waiting or 1)
        samples = self._parser.feed(data)
        if not samples:
            return None
        latest = samples[-1]
        latest.v1_ignored = self._parser.v1_ignored
        return latest

    def close(self) -> None:
        self._ser.close()


class DemoTelemetrySource:
    def __init__(self) -> None:
        self._start = time.monotonic()

    def read_latest(self) -> TelemetrySample:
        t = time.monotonic() - self._start
        cycle = (math.sin(t * 0.55) + 1.0) * 0.5
        pitch = -4.0 + cycle * 35.0
        roll = math.sin(t * 0.85) * 10.0
        if math.sin(t * 0.22) > 0.82:
            roll = math.sin(t * 2.4) * 20.0
        state = classify_like_firmware(pitch, roll)
        return TelemetrySample(pitch=pitch, roll=roll, state=state, source="demo")


def classify_like_firmware(pitch: float, roll: float) -> int:
    if abs(roll) >= THRESHOLD_LATERAL_DEG:
        return 5
    if pitch >= THRESHOLD_FULL_DEG:
        return 2
    if pitch >= THRESHOLD_MILD_DEG:
        return 1
    if pitch <= THRESHOLD_LEAN_BACK_DEG:
        return 4
    if pitch >= LEAN_FORWARD_MIN_DEG:
        return 3
    return 0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class PostureVisualizerApp:
    def __init__(self, root: tk.Tk, source: SerialTelemetrySource | DemoTelemetrySource) -> None:
        self.root = root
        self.source = source
        self.sample = TelemetrySample(0.0, 0.0, 0, "starting")
        self.visual_pitch = 0.0
        self.visual_roll = 0.0
        self.last_packet_time = time.monotonic()

        root.title("Posture Visualizer")
        root.minsize(880, 560)
        root.configure(bg="#f7f7f2")

        self.canvas = tk.Canvas(root, bg="#f7f7f2", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._tick()

    def _tick(self) -> None:
        try:
            latest = self.source.read_latest()
        except Exception as exc:
            latest = TelemetrySample(0.0, 0.0, 0, f"serial error: {exc}")

        if latest is not None:
            self.sample = latest
            self.last_packet_time = time.monotonic()
        self._draw()
        self.root.after(33, self._tick)

    def _draw(self) -> None:
        c = self.canvas
        c.delete("all")
        w = max(c.winfo_width(), 1)
        h = max(c.winfo_height(), 1)
        cx = w * 0.48
        ground_y = h * 0.82
        color = STATE_COLORS.get(self.sample.state, "#343a40")
        muted = "#676f77"
        ink = "#20252b"

        self.visual_pitch += (self.sample.pitch - self.visual_pitch) * 0.16
        self.visual_roll += (self.sample.roll - self.visual_roll) * 0.16
        pitch = clamp(self.visual_pitch, -20.0, 40.0)
        roll = clamp(self.visual_roll, -25.0, 25.0)
        slouch = clamp(pitch / MAX_VISUAL_SLOUCH_DEG, 0.0, 1.0)
        lean_back = clamp(-pitch / 18.0, 0.0, 1.0)
        roll_shift = roll * 2.4

        hip = (cx, ground_y - 96)
        chest = (
            hip[0] + roll_shift * 0.45 + slouch * 112 - lean_back * 54,
            hip[1] - 170 + slouch * 44 + lean_back * 8,
        )
        neck = (
            chest[0] + roll_shift * 0.20 + slouch * 24 - lean_back * 24,
            chest[1] - 58 + slouch * 38,
        )
        head = (
            neck[0] + slouch * 12 - lean_back * 15,
            neck[1] - 36 + slouch * 28,
        )

        spine_points: list[float] = []
        posture_curve = max(slouch, lean_back * 0.65)
        neutral_lower_control = (
            hip[0] + (neck[0] - hip[0]) * 0.18,
            hip[1] + (neck[1] - hip[1]) * 0.36,
        )
        neutral_upper_control = (
            hip[0] + (neck[0] - hip[0]) * 0.82,
            hip[1] + (neck[1] - hip[1]) * 0.66,
        )
        curved_lower_control = (
            hip[0] + roll_shift * 0.08 + slouch * 8 - lean_back * 32,
            hip[1] - 112 + slouch * 8,
        )
        curved_upper_control = (
            neck[0] - slouch * 82 + lean_back * 48,
            neck[1] + 42 + slouch * 28 + lean_back * 8,
        )
        curve_mix = clamp(posture_curve / 0.16, 0.0, 1.0)
        curve_mix = curve_mix * curve_mix * (3.0 - 2.0 * curve_mix)
        lower_control = (
            neutral_lower_control[0] + (curved_lower_control[0] - neutral_lower_control[0]) * curve_mix,
            neutral_lower_control[1] + (curved_lower_control[1] - neutral_lower_control[1]) * curve_mix,
        )
        upper_control = (
            neutral_upper_control[0] + (curved_upper_control[0] - neutral_upper_control[0]) * curve_mix,
            neutral_upper_control[1] + (curved_upper_control[1] - neutral_upper_control[1]) * curve_mix,
        )

        for i in range(22):
            t = i / 21.0
            inv = 1.0 - t
            x = (
                inv**3 * hip[0]
                + 3 * inv**2 * t * lower_control[0]
                + 3 * inv * t**2 * upper_control[0]
                + t**3 * neck[0]
            )
            y = (
                inv**3 * hip[1]
                + 3 * inv**2 * t * lower_control[1]
                + 3 * inv * t**2 * upper_control[1]
                + t**3 * neck[1]
            )
            spine_points.extend((x, y))

        c.create_line(60, ground_y, w - 60, ground_y, fill="#d6d3ca", width=2)
        self._draw_scale(w, h)

        left_foot = (hip[0] - 72, ground_y)
        right_foot = (hip[0] + 74, ground_y)
        knee_l = (hip[0] - 38, hip[1] + 76)
        knee_r = (hip[0] + 42, hip[1] + 76)
        shoulder_l = (chest[0] - 54, chest[1] + 18)
        shoulder_r = (chest[0] + 58, chest[1] + 14)
        hand_l = (shoulder_l[0] - 36 + slouch * 40, shoulder_l[1] + 96)
        hand_r = (shoulder_r[0] + 34 + slouch * 46, shoulder_r[1] + 98)

        c.create_line(*left_foot, *knee_l, *hip, *knee_r, *right_foot, fill=ink, width=9, smooth=True)
        c.create_line(*shoulder_l, *chest, *shoulder_r, fill=ink, width=9, smooth=True)
        c.create_line(*shoulder_l, *hand_l, fill=ink, width=7, smooth=True)
        c.create_line(*shoulder_r, *hand_r, fill=ink, width=7, smooth=True)
        c.create_line(
            *spine_points,
            fill=color,
            width=13,
            capstyle=tk.ROUND,
        )
        c.create_oval(head[0] - 31, head[1] - 31, head[0] + 31, head[1] + 31, outline=ink, width=8)

        self._draw_info_panel(w, h, color, muted)

    def _draw_scale(self, w: int, h: int) -> None:
        x0 = w - 220
        y0 = h * 0.22
        y1 = h * 0.74
        self.canvas.create_line(x0, y0, x0, y1, fill="#adb5bd", width=3)
        marks = [
            (-10, "back"),
            (0, "0"),
            (LEAN_FORWARD_MIN_DEG, "lean"),
            (THRESHOLD_MILD_DEG, "mild"),
            (THRESHOLD_FULL_DEG, "full"),
        ]
        for deg, label in marks:
            y = y1 - ((deg + 10.0) / 40.0) * (y1 - y0)
            self.canvas.create_line(x0 - 9, y, x0 + 9, y, fill="#868e96", width=2)
            self.canvas.create_text(x0 + 18, y, text=f"{label} {deg:g}", anchor="w", fill="#495057", font=("Segoe UI", 10))

        pitch = clamp(self.sample.pitch, -10.0, 30.0)
        y = y1 - ((pitch + 10.0) / 40.0) * (y1 - y0)
        self.canvas.create_oval(x0 - 10, y - 10, x0 + 10, y + 10, fill=STATE_COLORS.get(self.sample.state, "#343a40"), outline="")

    def _draw_info_panel(self, w: int, h: int, color: str, muted: str) -> None:
        state_name = STATE_NAMES.get(self.sample.state, "UNKNOWN")
        state_label = STATE_LABELS.get(self.sample.state, "Bilinmeyen")
        stale = time.monotonic() - self.last_packet_time > 1.2
        source = self.sample.source
        if stale and source != "demo":
            source = "waiting for packets"

        x = 42
        y = 42
        self.canvas.create_text(x, y, text=state_label, anchor="nw", fill=color, font=("Segoe UI", 30, "bold"))
        self.canvas.create_text(x, y + 50, text=state_name, anchor="nw", fill=muted, font=("Segoe UI", 13, "bold"))
        self.canvas.create_text(
            x,
            y + 92,
            text=f"pitch {self.sample.pitch:5.1f} deg    roll {self.sample.roll:5.1f} deg",
            anchor="nw",
            fill="#20252b",
            font=("Consolas", 16),
        )
        self.canvas.create_text(
            x,
            h - 58,
            text=f"source: {source}    ignored V1 packets: {self.sample.v1_ignored}",
            anchor="sw",
            fill=muted,
            font=("Segoe UI", 10),
        )

    def _on_close(self) -> None:
        close = getattr(self.source, "close", None)
        if close is not None:
            close()
        self.root.destroy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live stick-figure visualizer for posture telemetry.")
    parser.add_argument("--port", help="Serial port for the HM-10, for example COM3 or /dev/rfcomm0.")
    parser.add_argument("--demo", action="store_true", help="Force demo mode even when a port is provided.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.port and not args.demo:
        source: SerialTelemetrySource | DemoTelemetrySource = SerialTelemetrySource(args.port)
    else:
        source = DemoTelemetrySource()

    root = tk.Tk()
    PostureVisualizerApp(root, source)
    root.mainloop()


if __name__ == "__main__":
    main()
