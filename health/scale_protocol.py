"""Decoder for the scale's observed 19-byte completed-result frame.

The supplied V2 document describes a different ``02 40 ... 03`` protocol.
The age, height and weight offsets match test-object and adult measurements.
The final two payload bytes vary and their meaning is unconfirmed, so they
must not be treated as a checksum. The serial adapter separately requires
recent activity before forwarding a result to the BMI writer.
"""

from __future__ import annotations

from dataclasses import dataclass


FRAME_LENGTH = 19
HEADER = b"\xab\xba\x01"
TRAILER = b"\x0a\x0d"
ACTIVITY_LENGTH = 7
ACTIVITY_HEADER = b"\xa5\x5a"


@dataclass(frozen=True)
class ScaleMeasurement:
    age_years: int
    height_cm: int
    weight_kg: float
    raw: bytes


def decode_result_frame(frame: bytes) -> ScaleMeasurement | None:
    """Decode a complete result with the structure seen in real captures."""
    if len(frame) != FRAME_LENGTH or not frame.startswith(HEADER) or not frame.endswith(TRAILER):
        return None
    if frame[7:14] != bytes(7) or frame[14] not in (0, 1):
        return None
    age, height = frame[3], frame[4]
    weight = int.from_bytes(frame[5:7], "big") / 100
    if not (1 <= age <= 120 and 1 <= height <= 250 and 0 < weight <= 500):
        return None
    return ScaleMeasurement(age, height, weight, frame)


def result_frames(data: bytes) -> list[ScaleMeasurement]:
    """Find complete observed result frames in a raw serial capture."""
    measurements = []
    position = 0
    while (position := data.find(HEADER, position)) >= 0:
        result = decode_result_frame(data[position:position + FRAME_LENGTH])
        if result is not None:
            measurements.append(result)
            position += FRAME_LENGTH
        else:
            position += 1
    return measurements


class ScaleFrameStream:
    """Reassemble activity and completed-result frames across serial reads."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> tuple[int, list[ScaleMeasurement]]:
        self._buffer.extend(data)
        activity_count = 0
        results: list[ScaleMeasurement] = []
        while len(self._buffer) >= 2:
            activity_at = self._buffer.find(ACTIVITY_HEADER)
            result_at = self._buffer.find(HEADER[:2])
            starts = [index for index in (activity_at, result_at) if index >= 0]
            if not starts:
                self._buffer[:] = self._buffer[-1:]
                break
            start = min(starts)
            if start:
                del self._buffer[:start]
            if self._buffer.startswith(ACTIVITY_HEADER):
                if len(self._buffer) < ACTIVITY_LENGTH:
                    break
                frame = bytes(self._buffer[:ACTIVITY_LENGTH])
                checksum = 0
                for value in frame[:-1]:
                    checksum ^= value
                if checksum == frame[-1]:
                    activity_count += 1
                    del self._buffer[:ACTIVITY_LENGTH]
                else:
                    del self._buffer[0]
            else:
                if len(self._buffer) < FRAME_LENGTH:
                    break
                result = decode_result_frame(bytes(self._buffer[:FRAME_LENGTH]))
                if result is not None:
                    results.append(result)
                    del self._buffer[:FRAME_LENGTH]
                else:
                    del self._buffer[0]
        return activity_count, results
