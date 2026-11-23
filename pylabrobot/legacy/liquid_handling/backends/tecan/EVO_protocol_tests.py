"""Offline checks of the existing EVO response layout, not a transport specification.

The local MCA command manual (394562 V1.0, RFV and error-code sections)
specifies string report data and controller errors, but not USB framing. Fixtures
use the backend's existing one-byte prefix/two-byte module/status/body/suffix
layout. Delimiter values, status-bit semantics and packet reassembly remain
unverified; these tests must not be treated as hardware-protocol validation.
"""

import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock

from pylabrobot.legacy.liquid_handling.backends.tecan.EVO_backend import (
  EVOBackend,
)
from pylabrobot.legacy.liquid_handling.backends.tecan.errors import TecanError


class EVOTimeoutTests(unittest.TestCase):
  def test_default_read_timeout_is_120_seconds(self):
    self.assertEqual(EVOBackend().io.read_timeout, 120)

  def test_read_timeout_can_be_overridden(self):
    self.assertEqual(EVOBackend(read_timeout=30).io.read_timeout, 30)


class EVOResponseTests(unittest.TestCase):
  def setUp(self):
    # Bypass construction entirely: parsing must never instantiate/open USB.
    self.evo = object.__new__(EVOBackend)

  def test_rejects_invalid_utf8_payload(self):
    with self.assertRaises(UnicodeDecodeError):
      self.evo.parse_response(b"\x02W1\x8012\xff34\x00")

  def test_rejects_truncated_fixed_fields(self):
    frame = b"\x02W1\x80\x00"
    for length in range(len(frame)):
      with self.subTest(length=length):
        with self.assertRaisesRegex(ValueError, "truncated"):
          self.evo.parse_response(frame[:length])

  def test_empty_framed_success(self):
    self.assertEqual(
      self.evo.parse_response(b"\x02W1\x80\x00"), {"module": "W1", "data": []}
    )

  def test_signed_multi_value_data(self):
    self.assertEqual(
      self.evo.parse_response(b"\x02C5\x80-32768,0,32767,-1\x00"),
      {"module": "C5", "data": [-32768, 0, 32767, -1]},
    )

  def test_firmware_string_data(self):
    self.assertEqual(
      self.evo.parse_response(b"\x02W1\x80MCAGU-V1.00-01/2007\x00"),
      {"module": "W1", "data": ["MCAGU-V1.00-01/2007"]},
    )

  def test_mixed_values_skip_empty_fields(self):
    self.assertEqual(
      self.evo.parse_response(b"\x02W1\x80,-12,,XYZGP,0,\x00"),
      {"module": "W1", "data": [-12, "XYZGP", 0]},
    )

  def test_utf8_string_payload(self):
    self.assertEqual(
      self.evo.parse_response(b"\x02W1\x80caf\xc3\xa9\x00"),
      {"module": "W1", "data": ["caf\u00e9"]},
    )

  def test_controller_errors_preserve_module_and_code(self):
    for module in ("C1", "C5", "W1"):
      for code in (1, 3, 127):
        with self.subTest(module=module, code=code):
          frame = b"\x02" + module.encode() + bytes([code ^ 128]) + b"\x00"
          with self.assertRaises(TecanError) as caught:
            self.evo.parse_response(frame)
          self.assertEqual(caught.exception.module, module)
          self.assertEqual(caught.exception.error_code, code)

  def test_controller_error_precedes_payload_decoding(self):
    with self.assertRaises(TecanError) as caught:
      self.evo.parse_response(b"\x02C5\x83\xff\x00")
    self.assertEqual(caught.exception.error_code, 3)
    self.assertEqual(caught.exception.message, "Invalid operand")

  def test_existing_status_xor_semantics_are_unchanged(self):
    # No manual support for replacing XOR with a mask or accepting low-bit success.
    for status in (0, 3, 127):
      with self.subTest(status=status):
        with self.assertRaises(TecanError) as caught:
          self.evo.parse_response(b"\x02W1" + bytes([status]) + b"\x00")
        self.assertEqual(caught.exception.error_code, status ^ 128)

  def test_delimiter_values_remain_unvalidated(self):
    # Characterize a limitation rather than invent delimiter rules from a DCU manual.
    self.assertEqual(
      self.evo.parse_response(b"\x01W1\x8042\x03"), {"module": "W1", "data": [42]}
    )


class EVOSendResponseTests(unittest.IsolatedAsyncioTestCase):
  def setUp(self):
    self.evo = object.__new__(EVOBackend)
    self.evo._cache = {}
    self.io = SimpleNamespace(write=AsyncMock(), read=AsyncMock())
    self.evo.io = self.io  # type: ignore[assignment]

  async def test_rejects_mismatched_response_module(self):
    for status in (b"\x80", b"\x83"):
      with self.subTest(status=status):
        self.io.read.return_value = b"\x02C5" + status + b"\x00"
        with self.assertRaisesRegex(ValueError, "Expected.*W1.*C5"):
          await self.evo.send_command("W1", "RFV")

  async def test_matching_response_module(self):
    self.io.read.return_value = b"\x02W1\x80XYZGP\x00"
    self.assertEqual(
      await self.evo.send_command("W1", "RFV"), {"module": "W1", "data": ["XYZGP"]}
    )
    self.io.write.assert_awaited_once_with(b"\x02W1RFV\x00", timeout=None)
    self.io.read.assert_awaited_once_with(timeout=None)

  async def test_matching_controller_error_is_preserved(self):
    self.io.read.return_value = b"\x02W1\x83\x00"
    with self.assertRaises(TecanError) as caught:
      await self.evo.send_command("W1", "RFV")
    self.assertEqual(caught.exception.module, "W1")
    self.assertEqual(caught.exception.error_code, 3)

  async def test_truncated_response_uses_parser_error(self):
    self.io.read.return_value = b"\x02W"
    with self.assertRaisesRegex(ValueError, "truncated"):
      await self.evo.send_command("W1", "RFV")

  async def test_no_wait_does_not_read_response(self):
    self.assertIsNone(await self.evo.send_command("W1", "RFV", wait=False))
    self.io.write.assert_awaited_once_with(b"\x02W1RFV\x00", timeout=None)
    self.io.read.assert_not_awaited()

  async def test_timeouts_are_forwarded(self):
    self.io.read.return_value = b"\x02W1\x800\x00"
    self.assertEqual(
      await self.evo.send_command("W1", "RFV", read_timeout=2, write_timeout=3),
      {"module": "W1", "data": [0]},
    )
    self.io.write.assert_awaited_once_with(b"\x02W1RFV\x00", timeout=3)
    self.io.read.assert_awaited_once_with(timeout=2)
