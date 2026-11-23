"""Offline setup regressions: every USB operation is replaced before setup."""

import unittest
from unittest.mock import AsyncMock, call

from pylabrobot.legacy.liquid_handling import LiquidHandler
from pylabrobot.legacy.liquid_handling.backends.tecan.EVO_backend import (
  EVOArm,
  EVOBackend,
  Mca,
)
from pylabrobot.legacy.liquid_handling.backends.tecan.errors import TecanError
from pylabrobot.resources import EVO200Deck


class EVOSetupTests(unittest.IsolatedAsyncioTestCase):
  def setUp(self):
    self.backend = EVOBackend(diti_count=8)
    self.backend.io.setup = AsyncMock()
    self.backend.io.stop = AsyncMock()
    self.backend.send_command = AsyncMock(return_value={"data": []})
    EVOArm._pos_cache.clear()

  def tearDown(self):
    EVOArm._pos_cache.clear()

  async def test_mca_setup_uses_backend_module_and_no_liha_axis_command(self):
    self.assertTrue(await self.backend.setup_arm(EVOBackend.MCA))
    self.assertEqual(self.backend.send_command.await_args_list, [
      call(EVOBackend.MCA, command="PIB"),
      call(EVOBackend.MCA, command="PIA"),
    ])

  async def test_mca_parking_sends_operator_validated_sequence(self):
    # Operator visually validated this sequence during lh.setup() (operator_decisions.md).
    await self.backend._park_mca()
    self.assertEqual(self.backend.send_command.await_args_list, [
      call(EVOBackend.MCA, command="PIA"),
      call(EVOBackend.MCA, command="PAA", params=[6000, 1000, None]),
      call(EVOBackend.MCA, command="BMA", params=[0, 0, 0]),
    ])

  async def test_liha_setup_retains_initialization_and_x_release(self):
    self.assertTrue(await self.backend.setup_arm(EVOBackend.LIHA))
    self.assertEqual(self.backend.send_command.await_args_list, [
      call(EVOBackend.LIHA, command="PIA"),
      call(EVOBackend.LIHA, command="BMX", params=[2]),
    ])

  async def test_setup_arm_propagates_fault_without_retry(self):
    fault = TecanError("injected initialization failure", "W1", 1)
    self.backend.send_command.side_effect = fault
    with self.assertRaises(TecanError):
      await self.backend.setup_arm(EVOBackend.MCA)
    self.assertEqual(self.backend.send_command.await_count, 1)

  async def test_setup_arm_propagates_timeout_without_retry(self):
    self.backend.send_command.side_effect = TimeoutError("injected timeout")
    with self.assertRaises(TimeoutError):
      await self.backend.setup_arm(EVOBackend.MCA)
    self.assertEqual(self.backend.send_command.await_count, 1)

  async def test_unimplemented_arm_returns_false(self):
    self.backend.send_command.side_effect = TecanError("not implemented", "W1", 5)
    self.assertFalse(await self.backend.setup_arm(EVOBackend.MCA))
    self.assertEqual(self.backend.send_command.await_count, 1)

  async def test_full_lh_setup_reaches_all_arms_and_flush(self):
    async def reply(module, command, params=None, **kwargs):
      if command == "RNT":
        return {"data": [8]}
      if command == "RPA":
        return {"data": [6000, 1000, 2370, 690]}
      if command == "RPX":
        return {"data": [13559 if params == [5] else (9000 if module == "C1" else 45)]}
      if command == "RPY":
        return {"data": [2825] * 8}
      if command == "RPZ":
        return {"data": [2100] * 8}
      return {"data": []}

    self.backend.send_command.side_effect = reply
    lh = LiquidHandler(backend=self.backend, deck=EVO200Deck())
    await lh.setup()
    self.assertTrue(lh.setup_finished)
    self.assertTrue(self.backend.liha_connected)
    self.assertTrue(self.backend.mca_connected)
    self.assertTrue(self.backend.roma_connected)
    self.assertIsInstance(self.backend.mca, Mca)
    self.assertEqual(self.backend.num_channels, 8)
    commands = self.backend.send_command.await_args_list
    for module in (EVOBackend.LIHA, EVOBackend.MCA, EVOBackend.ROMA):
      self.assertIn(call(module, command="PIA"), commands)
    self.assertEqual(sum(c.args == ("W1",) and c.kwargs.get("command") == "PIA"
                         for c in commands), 2)  # setup_arm PIA + parking PIA
    self.assertTrue(any(c.kwargs.get("command") == "PID" for c in commands))
    mca = [c.kwargs.get("command") for c in commands if c.kwargs.get("module", c.args[0] if c.args else None) == "W1"]
    # MCA: setup_arm, parking, then head drive (plunger) initialization as in EVOware
    self.assertEqual(mca[mca.index("BMA") + 1:mca.index("BMA") + 8],
                     ["RPA", "PAA", "PID", "ASH", "ASH", "SLP", "PPA"])
    await lh.stop()
    self.backend.io.stop.assert_awaited_once()

  async def test_skip_initialization_sends_reports_only(self):
    async def reply(module, command, params=None, **kwargs):
      if command == "RNT":
        return {"data": [8]}
      if command in ("RPX", "RPY", "RPZ"):
        return {"data": [2100] * 8}
      return {"data": ["FW"]}

    self.backend.send_command.side_effect = reply
    lh = LiquidHandler(backend=self.backend, deck=EVO200Deck())
    await lh.setup(skip_initialization=True)
    commands = {c.kwargs.get("command") for c in self.backend.send_command.await_args_list}
    self.assertTrue(commands <= {"RFV", "RNT", "RPX", "RPY", "RPZ"}, commands)
    self.assertTrue(self.backend.mca_connected and self.backend.head96_installed)
    self.assertEqual(len(lh.head96), 96)
    self.assertEqual(self.backend.num_channels, 8)


if __name__ == "__main__":
  unittest.main()
