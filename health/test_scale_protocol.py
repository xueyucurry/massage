import os
import pty
import unittest
from unittest.mock import patch

from PyQt5.QtCore import QCoreApplication

from scale_protocol import ScaleFrameStream, decode_result_frame, result_frames
from scale_serial import ScaleSerialAdapter


FIRST_CAPTURE = bytes.fromhex("ab ba 01 19 9f 02 80 00 00 00 00 00 00 00 00 19 1e 0a 0d")
SECOND_CAPTURE = bytes.fromhex("ab ba 01 19 58 02 80 00 00 00 00 00 00 00 00 00 c0 0a 0d")


def adult_result_frame() -> bytes:
    frame = bytearray(SECOND_CAPTURE)
    frame[4] = 170
    frame[5:7] = (7000).to_bytes(2, "big")
    frame[14:17] = b"\x01\x4e\xc9"  # Observed tail layout; its meaning is unknown.
    return bytes(frame)


class ScaleProtocolTests(unittest.TestCase):
    def test_screen_values_match_second_real_capture(self):
        measurement = decode_result_frame(SECOND_CAPTURE)
        self.assertIsNotNone(measurement)
        self.assertEqual((measurement.age_years, measurement.height_cm, measurement.weight_kg), (25, 88, 6.4))

    def test_both_test_object_captures_have_the_expected_fields(self):
        first, second = result_frames(b"\x55" + FIRST_CAPTURE + b"\xa5\x5a" + SECOND_CAPTURE)
        self.assertEqual(first.height_cm, 159)
        self.assertEqual(first.weight_kg, 6.4)
        self.assertEqual(second.height_cm, 88)

    def test_corrupt_or_incomplete_result_is_rejected(self):
        self.assertIsNone(decode_result_frame(SECOND_CAPTURE[:-1]))
        corrupt = bytearray(SECOND_CAPTURE)
        corrupt[8] = 1  # Reserved bytes are zero in every observed result.
        self.assertIsNone(decode_result_frame(bytes(corrupt)))
        self.assertEqual(result_frames(b"\xab\xba\x01" + bytes(corrupt)), [])
        zero_weight = bytearray(adult_result_frame())
        zero_weight[5:7] = b"\x00\x00"
        self.assertIsNone(decode_result_frame(bytes(zero_weight)))

    def test_adult_result_accepts_variable_opaque_tail(self):
        result = decode_result_frame(adult_result_frame())
        self.assertIsNotNone(result)
        self.assertEqual((result.age_years, result.height_cm, result.weight_kg), (25, 170, 70.0))

    def test_stream_reassembles_split_activity_and_result_frames(self):
        stream = ScaleFrameStream()
        activity = bytes.fromhex("a5 5a 06 03 03 36 cf")
        parts = [b"\x55" + activity[:3], activity[3:] + activity + SECOND_CAPTURE[:7], SECOND_CAPTURE[7:]]
        counts, measurements = [], []
        for part in parts:
            count, ready = stream.feed(part)
            counts.append(count)
            measurements.extend(ready)
        self.assertEqual(sum(counts), 2)
        self.assertEqual(len(measurements), 1)
        self.assertEqual(measurements[0].weight_kg, 6.4)

    def test_serial_adapter_emits_only_a_result_after_activity(self):
        app = QCoreApplication.instance() or QCoreApplication([])
        self.assertIsNotNone(app)
        master, slave = pty.openpty()
        adapter = ScaleSerialAdapter(port=os.ttyname(slave))
        received = []
        adapter.measurementReceived.connect(received.append)
        activity = bytes.fromhex("a5 5a 06 03 03 36 cf")
        try:
            adapter.start()
            os.write(master, adult_result_frame())
            adapter._poll()
            self.assertEqual(received, [])
            os.write(master, activity * 3 + adult_result_frame())
            adapter._poll()
            self.assertEqual(len(received), 1)
            self.assertEqual((received[0].height_cm, received[0].weight_kg), (170, 70.0))
        finally:
            adapter.stop()
            os.close(master)
            os.close(slave)

    def test_activity_frames_must_all_be_within_previous_20_seconds(self):
        app = QCoreApplication.instance() or QCoreApplication([])
        self.assertIsNotNone(app)
        master, slave = pty.openpty()
        adapter = ScaleSerialAdapter(port=os.ttyname(slave))
        received = []
        adapter.measurementReceived.connect(received.append)
        activity = bytes.fromhex("a5 5a 06 03 03 36 cf")
        try:
            adapter.start()
            for at in (0, 15, 30):
                os.write(master, activity)
                with patch("scale_serial.time.monotonic", return_value=at):
                    adapter._poll()
            os.write(master, adult_result_frame())
            with patch("scale_serial.time.monotonic", return_value=30):
                adapter._poll()
            self.assertEqual(received, [])

            os.write(master, activity * 2 + adult_result_frame())
            with patch("scale_serial.time.monotonic", return_value=31):
                adapter._poll()
            self.assertEqual(len(received), 1)
        finally:
            adapter.stop()
            os.close(master)
            os.close(slave)


if __name__ == "__main__":
    unittest.main()
