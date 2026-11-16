"""Offline tests for the EVO's two door-lock command sequences."""
import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock, call

from pylabrobot.legacy.liquid_handling.backends.tecan.EVO_backend import EVOBackend
from pylabrobot.legacy.liquid_handling.backends.tecan.errors import TecanError


class EVODoorTests(unittest.IsolatedAsyncioTestCase):
  def setUp(self):
    self.evo = object.__new__(EVOBackend)
    self.evo._cache = {}
    self.io = SimpleNamespace(write=AsyncMock(), read=AsyncMock(return_value=b"\x02O1\x80\x00"))
    self.evo.io = self.io  # type: ignore[assignment]

  async def test_open_door_error_warns_and_stops(self):
    self.io.read.return_value = b"\x02O1\x92\x00"
    with self.assertLogs("pylabrobot.legacy.liquid_handling.backends.tecan.EVO_backend", level="WARNING") as logs:
      with self.assertRaises(TecanError) as caught:
        await self.evo.door_lock()
    self.assertEqual(caught.exception.error_code, 18)
    self.assertIn("close the door", " ".join(logs.output).lower())
    self.assertEqual(self.io.write.await_count, 1)

  async def test_second_lock_failure_warns_and_does_not_retry(self):
    self.io.read.side_effect = [b"\x02O1\x80\x00", b"\x02O1\x92\x00"]
    with self.assertLogs("pylabrobot.legacy.liquid_handling.backends.tecan.EVO_backend", level="WARNING") as logs:
      with self.assertRaises(TecanError):
        await self.evo.door_lock()
    self.assertEqual(self.io.write.await_count, 2)
    self.assertIn("partial or unknown", " ".join(logs.output))

  async def test_unlock_failure_stops_without_retry(self):
    self.io.read.return_value = b"\x02O1\x83\x00"
    with self.assertRaises(TecanError):
      await self.evo.door_unlock()
    self.assertEqual(self.io.write.await_count, 1)

  async def test_other_lock_error_is_not_mislabelled_door_open(self):
    self.io.read.return_value = b"\x02O1\x83\x00"
    with self.assertNoLogs("pylabrobot.legacy.liquid_handling.backends.tecan.EVO_backend", level="WARNING"):
      with self.assertRaises(TecanError) as caught:
        await self.evo.door_lock()
    self.assertEqual(caught.exception.error_code, 3)
    self.assertEqual(self.io.write.await_count, 1)

  async def test_open_sends_right_then_left_unlock(self):
    await self.evo.door_unlock()
    self.assertEqual(self.io.write.await_args_list, [
      call(b"\x02O1ALO1,0\x00", timeout=None), call(b"\x02O1ALO2,0\x00", timeout=None),
    ])

  async def test_lock_sends_right_then_left(self):
    await self.evo.door_lock()
    self.assertEqual(self.io.write.await_args_list, [
      call(b"\x02O1ALO1,1\x00", timeout=None), call(b"\x02O1ALO2,1\x00", timeout=None),
    ])
