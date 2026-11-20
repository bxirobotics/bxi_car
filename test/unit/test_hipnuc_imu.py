#!/usr/bin/env python3
"""Exercise the HI91 serial and timestamp handling without an IMU."""

import fcntl
import importlib.util
import math
import os
from pathlib import Path
import pty
import select
import struct
import termios
import unittest
from unittest.mock import patch


SPEC = importlib.util.spec_from_file_location(
    "hipnuc_imu", Path(__file__).resolve().parents[2] / "tools/hipnuc_imu.py")
imu = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(imu)
WALL_EPOCH_NS = 1_700_000_000_000_000_000


def frame(tick, accel=(1., -2., 0.5), gyro=(180., -90., 45.),
          quat=(0.75, 0.0, 0.0, 0.60), tag=0x91):
    header = bytes.fromhex("5a a5 4c 00")
    payload = (struct.pack("<BHbfI", tag, 0xa55a, 20, 1001.0, tick)
               + struct.pack("<16f", *accel, *gyro, 0., 0., 0., 0., 0., 0., *quat))
    crc = 0
    for byte in header + payload:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return header + crc.to_bytes(2, "little") + payload


def stamp(clock, host_ms, *ticks, wall_offset_ns=0):
    samples = [imu.Sample(tick, (0., 0., 9.8), (0., 0., 0.), (1., 0., 0., 0.))
               for tick in ticks]
    host_ns = host_ms * imu.MS_NS
    return clock.stamp(samples, host_ns, WALL_EPOCH_NS + host_ns + wall_offset_ns)


class DecoderTest(unittest.TestCase):
    def test_fragmentation_resynchronizes_and_preserves_units(self):
        decoder = imu.Decoder()
        raw = frame(0x12345678)
        corrupted = bytearray(frame(14))
        corrupted[-1] ^= 0x80
        wrong_type = frame(16, tag=0x90)
        noise = b"junk\x5a\xa5\xff\xff\xff\xff"
        self.assertEqual(decoder.feed(noise + corrupted + wrong_type + raw[:1]), [])
        self.assertEqual(decoder.feed(raw[1:5]), [])
        self.assertEqual(decoder.feed(raw[5:40]), [])
        [sample] = decoder.feed(raw[40:])
        self.assertEqual(sample.tick_ms, 0x12345678)
        self.assertEqual(sample.acceleration, (9.8, -19.6, 4.9))
        self.assertEqual(sample.gyro, (math.pi, -math.pi / 2, math.pi / 4))
        self.assertAlmostEqual(math.hypot(*sample.quaternion), 1.0)
        self.assertAlmostEqual(sample.quaternion[0] / sample.quaternion[3], 1.25)

    def test_crc_valid_but_invalid_measurements_are_rejected(self):
        for raw in (frame(10, accel=(math.inf, 0., 0.)), frame(10, gyro=(math.nan, 0., 0.)),
                    frame(10, quat=(0., 0., 0., 0.)), frame(10, quat=(1.2, 0., 0., 0.)),
                    frame(10, quat=(math.nan, 0., 0., 0.))):
            with self.assertRaisesRegex(imu.ImuError, "Invalid HI91"):
                imu.Decoder().feed(raw)


class ClockTest(unittest.TestCase):
    def test_wrap_is_continuous_and_delayed_data_keeps_an_older_stamp(self):
        clock = imu.SampleClock()
        self.assertIsNone(stamp(clock, 1000, 0xfffffffe, 0))
        self.assertEqual(stamp(clock, 1050, 48, 50), WALL_EPOCH_NS + 1_050_000_000)
        delayed = stamp(clock, 1070, 56, 58)
        self.assertGreaterEqual(delayed, WALL_EPOCH_NS + 1_058_000_000)
        self.assertLess(delayed, WALL_EPOCH_NS + 1_059_000_000)

    def test_fails_on_duplicate_reorder_or_device_reset_within_or_across_reads(self):
        for ticks in ((1000, 1000), (1000, 998)):
            with self.assertRaisesRegex(imu.ImuError, "duplicated, reordered, or reset"):
                stamp(imu.SampleClock(), 1000, *ticks)
        clock = imu.SampleClock()
        self.assertIsNone(stamp(clock, 1000, 1_000_000))
        with self.assertRaisesRegex(imu.ImuError, "duplicated, reordered, or reset"):
            stamp(clock, 1002, 2)

    def test_fails_on_stale_burst_excess_lag_and_fast_replay(self):
        with self.assertRaisesRegex(imu.ImuError, "buffered or advancing"):
            stamp(imu.SampleClock(), 1000, 0, 52)
        clock = imu.SampleClock()
        self.assertIsNone(stamp(clock, 1000, 0))
        with self.assertRaisesRegex(imu.ImuError, "buffered or advancing"):
            stamp(clock, 1070, 18)
        clock = imu.SampleClock()
        # Each read moves only 1 ms ahead of the host, so per-read clamping would miss this.
        for offset in range(26):
            self.assertIsNone(stamp(clock, 1000 + offset, offset * 2))
        with self.assertRaisesRegex(imu.ImuError, "buffered or advancing"):
            stamp(clock, 1026, 52)

    def test_fails_on_large_wall_clock_step(self):
        clock = imu.SampleClock()
        self.assertIsNone(stamp(clock, 1000, 0))
        with self.assertRaisesRegex(imu.ImuError, "wall clock stepped"):
            stamp(clock, 1002, 2, wall_offset_ns=100 * imu.MS_NS)


class SerialTest(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.addCleanup(os.close, self.slave)
        self.addCleanup(os.close, self.master)
        self.port = os.ttyname(self.slave)

    def test_unconfigured_usb_serial_stops_before_opening_the_port(self):
        with patch.object(imu, "USB_SERIAL", ""), self.assertRaisesRegex(imu.ImuError, "CAR2_IMU_USB_SERIAL"):
            with imu.open_serial("/nonexistent-imu-port"):
                self.fail("opened an unconfigured port")

    def test_reads_passively_configures_921600_and_restores_settings(self):
        original = termios.tcgetattr(self.slave)
        with imu.open_serial(self.port, verify_identity=False) as fd:
            flags = fcntl.fcntl(fd, fcntl.F_GETFL)
            self.assertEqual(flags & os.O_ACCMODE, os.O_RDONLY)
            attrs = termios.tcgetattr(fd)
            self.assertEqual(attrs[4:6], [termios.B921600, termios.B921600])
            payload = frame(88)
            os.write(self.master, payload)
            self.assertEqual(select.select([fd], [], [], 0.2)[0], [fd])
            self.assertEqual(os.read(fd, len(payload)), payload)
        self.assertEqual(termios.tcgetattr(self.slave), original)
        with imu.open_serial(self.port, verify_identity=False):
            pass

    def test_refuses_a_locked_port_and_default_identity_check_rejects_pty(self):
        # A pseudo terminal has no matching USB parent in sysfs.
        with patch.object(imu, "USB_SERIAL", "test-adapter"), self.assertRaises((imu.ImuError, OSError)):
            with imu.open_serial(self.port):
                self.fail("opened a port with the wrong identity")
        original = termios.tcgetattr(self.slave)
        fcntl.flock(self.slave, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with self.assertRaises(BlockingIOError):
            with imu.open_serial(self.port, verify_identity=False):
                self.fail("opened a locked serial port")
        self.assertEqual(termios.tcgetattr(self.slave), original)

    def test_silence_times_out(self):
        with imu.open_serial(self.port, verify_identity=False) as fd:
            with self.assertRaisesRegex(imu.ImuError, "Timed out"):
                next(imu.read_samples(fd))

    def test_the_reader_publishes_only_the_newest_sample_per_read(self):
        current_ms = 1000
        batches = iter(((1000, frame(0)), (1025, frame(24)),
                        (1052, frame(48) + frame(50) + frame(52))))

        def ready(*_):
            nonlocal current_ms, payload
            current_ms, payload = next(batches)
            return ([self.slave], [], [])

        payload = b""
        with (patch.object(imu.select, "select", side_effect=ready),
              patch.object(imu.os, "read", side_effect=lambda *_: payload),
              patch.object(imu.time, "monotonic_ns", side_effect=lambda: current_ms * imu.MS_NS),
              patch.object(imu.time, "time_ns",
                           side_effect=lambda: WALL_EPOCH_NS + current_ms * imu.MS_NS)):
            timestamp_ns, sample = next(imu.read_samples(self.slave))
        self.assertEqual(sample.tick_ms, 52)
        self.assertEqual(timestamp_ns, WALL_EPOCH_NS + 1_052_000_000)

    def test_excess_bytes_cannot_be_retimestamped(self):
        reader, writer = os.pipe()
        self.addCleanup(os.close, reader)
        self.addCleanup(os.close, writer)
        os.write(writer, frame(2) * (1 + imu.MAX_BURST_BYTES // imu.FRAME_SIZE))
        with self.assertRaisesRegex(imu.ImuError, "backlog is too large"):
            next(imu.read_samples(reader))


if __name__ == "__main__":
    unittest.main()
