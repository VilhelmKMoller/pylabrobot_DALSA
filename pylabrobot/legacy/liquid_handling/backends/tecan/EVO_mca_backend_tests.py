"""EVOBackend MCA methods through LiquidHandler, against a simulated controller."""

import unittest

from pylabrobot.legacy.liquid_handling import LiquidHandler
from pylabrobot.legacy.liquid_handling.backends.tecan import mca
from pylabrobot.legacy.liquid_handling.backends.tecan.EVO_backend import EVOBackend
from pylabrobot.legacy.liquid_handling.backends.tecan.errors import TecanError
from pylabrobot.legacy.liquid_handling.backends.tecan.mca_tests import DATA, _TipDiameter, capture_deck


class FakeController:
  """Answers RPA/RPX/RPZ/T40RDP4 and tracks MCA axes and plunger like the firmware."""

  def __init__(self):
    self.mca = [6000, 1000, mca.Z_TRAVEL, 690]
    self.plunger = 50000
    self.x = {"C5": 45, "C1": 9000}
    self.sent = []
    self.fail_on = None

  async def send_command(self, module, command, params=None, **kw):
    params = list(params or [])
    self.sent.append((module, command, params))
    if self.fail_on and (module, command) == self.fail_on:
      raise TecanError("injected", module, 1)
    if module == "W1":
      if command == "RPA":
        sel = params[0]
        return {"module": "W1", "data": list(self.mca) if sel == 0 else [4999, 3512, 1200, 400]}
      if command == "PAA":
        for i, v in enumerate(params):
          if v is not None:
            self.mca[i] = v
      elif command == "PPA":
        self.plunger = params[0]
      elif command == "PPR":
        self.plunger += params[0]
      elif command == "T40RDP4":
        return {"module": "W1", "data": [-(self.plunger + 1000)]}
      elif command == "ATF":
        self.mca[2] = params[2]
      elif command in ("PID",):
        self.plunger = 0
      elif command == "ASH":
        pass
      elif command == "ATD":
        self.plunger = 0
      return {"module": "W1", "data": []}
    if command == "RPX":
      return {"module": module, "data": [self.x[module]]}
    if command == "RPZ":
      return {"module": module, "data": [2100] * 8 if module == "C5" else [2464 if params[0] == 0 else 2540]}
    if command == "PAX":
      self.x[module] = params[0]
      return {"module": module, "data": []}
    raise AssertionError(f"unexpected {module}{command}{params}")


class McaBackendTests(_TipDiameter, unittest.IsolatedAsyncioTestCase):
  async def asyncSetUp(self):
    import asyncio as _asyncio
    from unittest.mock import patch

    real_sleep = _asyncio.sleep

    async def no_wait(_):
      await real_sleep(0)

    p = patch("pylabrobot.legacy.liquid_handling.backends.tecan.EVO_backend.asyncio.sleep", no_wait)
    p.start()
    self.addCleanup(p.stop)
    self.deck, self.racks, _, self.dw, _ = capture_deck()
    _, _, _, _, self.troughs = capture_deck(cleaning=True)
    self.backend = EVOBackend(diti_count=0)
    self.backend.load_evoware_config(DATA / "evoware")
    self.fake = FakeController()
    self.backend.send_command = self.fake.send_command
    self.backend._head96_installed = True
    self.backend._num_channels = 8
    self.lh = LiquidHandler(backend=self.backend, deck=self.deck)
    # set up LiquidHandler state without the hardware setup() of the backend
    self.lh.head96 = {}
    from pylabrobot.legacy.tip_tracker import TipTracker

    self.lh.head = {c: TipTracker(thing=f"Channel {c}") for c in range(8)}
    self.lh.head96 = {c: TipTracker(thing=f"Channel {c}") for c in range(96)}
    self.backend.set_heads(head=self.lh.head, head96=self.lh.head96)
    self.lh._setup_finished = True

  def w1(self, *commands):
    return [(c, p) for m, c, p in self.fake.sent if m == "W1" and (not commands or c in commands)]

  async def test_pick_up_aspirate_dispense_return(self):
    await self.lh.pick_up_tips96(self.racks[4])
    self.assertTrue(all(t.has_tip for t in self.lh.head96.values()))
    self.assertIn(("ATF", [714, 520, 2370, 0, 90]), self.w1("ATF"))
    self.assertEqual(self.fake.x["C1"], 11871 + 2440)  # RoMa pushed before the MCA XY move

    await self.lh.aspirate96(self.dw, volume=100)
    self.assertEqual(self.fake.plunger, 50000 - 4000 - 21080 - 2000)
    await self.lh.dispense96(self.dw, volume=100)
    self.assertEqual(self.fake.plunger, 50000)

    await self.lh.return_tips96()
    self.assertIn(("ATD", []), self.w1("ATD"))
    self.assertFalse(any(t.has_tip for t in self.lh.head96.values()))
    self.assertEqual(self.fake.plunger, 50000)

  async def test_xy_never_moves_below_travel_height(self):
    await self.lh.pick_up_tips96(self.racks[4])
    await self.lh.aspirate96(self.dw, volume=100)
    await self.lh.dispense96(self.dw, volume=100)
    z = mca.Z_TRAVEL
    for c, p in self.w1("PAA", "ATF"):
      if c == "ATF":
        z = p[2]  # tip fetch ends at Zretract
        continue
      if len(p) >= 3 and p[2] is not None:
        z = p[2]
      if p and p[0] is not None and (len(p) < 2 or p[1] is not None):
        target = (p[0], p[1])
        # XY moves only at travel height, or to the XY the head is already at
        self.assertEqual(z, mca.Z_TRAVEL, f"XY move {target} at Z {z}")

  async def test_mix96_in_trough(self):
    await self.lh.pick_up_tips96(self.racks[4])
    await self.backend.mix96(self.troughs[9], volume=175, repetitions=5)
    pprs = [p[0] for c, p in self.w1("PPR")]
    self.assertEqual(pprs, [-4000] + [-35000, 35000] * 5)
    self.assertEqual(self.fake.mca[2], mca.Z_TRAVEL)
    slp = [p for c, p in self.w1("SLP")]
    self.assertIn([1152000, 10000, 3200000], slp)  # liquid class: 50 uL/s in
    self.assertIn([1200000, 80000, 3200000], slp)  # 400 uL/s out

  async def test_mix96_flow_rate_sets_both_strokes(self):
    await self.lh.pick_up_tips96(self.racks[4])
    await self.backend.mix96(self.troughs[9], volume=175, repetitions=1, flow_rate=25)
    speeds = [p[1] for c, p in self.w1("SLP")][-2:]
    self.assertEqual(speeds, [5000, 5000])

  async def test_plunger_mismatch_stops_and_marks_unknown(self):
    await self.lh.pick_up_tips96(self.racks[4])
    self.fake.plunger = 49000  # something moved the plunger
    with self.assertRaisesRegex(TecanError, "plunger"):
      await self.lh.aspirate96(self.dw, volume=100)
    self.assertFalse(any(c == "PPR" for c, _ in self.w1()))
    self.assertIsNone(self.backend._mca_plunger().position)
    with self.assertRaisesRegex(RuntimeError, "unknown"):
      await self.lh.aspirate96(self.dw, volume=100)

  async def test_firmware_error_stops_sequence(self):
    self.fake.fail_on = ("W1", "ATF")
    with self.assertRaises(TecanError):
      await self.lh.pick_up_tips96(self.racks[4])
    self.assertFalse(any(t.has_tip for t in self.lh.head96.values()))
    self.assertEqual(self.w1()[-1][0], "ATF")

  async def test_failed_tip_fetch_raises_head(self):
    from pylabrobot.legacy.liquid_handling.backends.tecan.errors import TecanTipsNotFetchedError

    orig = self.fake.send_command

    async def atf_fails(module, command, params=None, **kw):
      if (module, command) == ("W1", "ATF"):
        self.fake.mca[2] = params[1]  # head ends at Zend
        self.fake.sent.append((module, command, list(params)))
        raise TecanTipsNotFetchedError("Tips not fetched", "W1", 25)
      return await orig(module, command, params)

    self.backend.send_command = atf_fails
    with self.assertRaises(TecanTipsNotFetchedError):
      await self.lh.pick_up_tips96(self.racks[4])
    self.assertEqual(self.fake.mca[2], mca.Z_TRAVEL)
    self.assertEqual(self.w1()[-1], ("PAA", [None, None, mca.Z_TRAVEL]))
    self.assertFalse(any(t.has_tip for t in self.lh.head96.values()))

  async def test_drop_with_liquid_refused(self):
    await self.lh.pick_up_tips96(self.racks[4])
    await self.lh.aspirate96(self.dw, volume=100)
    with self.assertRaisesRegex(RuntimeError, "hold liquid"):
      await self.backend.drop_tips96(type("D", (), {"resource": self.racks[4], "offset": None})())

  async def test_multi_dispense_and_empty(self):
    await self.lh.pick_up_tips96(self.racks[4])
    await self.lh.aspirate96(self.troughs[9], volume=100, multi_dispense=True)
    for _ in range(4):
      await self.lh.dispense96(self.troughs[15], volume=25)
    self.assertNotEqual(self.fake.plunger, 50000)  # excess and air gap remain
    await self.backend.mca_empty_tips(self.troughs[9])
    self.assertEqual(self.fake.plunger, 50000)
    await self.lh.return_tips96()

  async def test_aspirate_with_mix(self):
    from pylabrobot.lib.liquid_handling.mix import Mix

    await self.lh.pick_up_tips96(self.racks[4])
    await self.lh.aspirate96(self.troughs[9], volume=100, mix=Mix(volume=150, repetitions=2, flow_rate=50))
    pprs = [p[0] for c, p in self.w1("PPR")]
    self.assertEqual(pprs, [-4000, -30000, 30000, -30000, 30000, -21080, -2000])

  async def test_head_drive_init(self):
    await self.backend.mca_initialize_head_drive()
    self.assertEqual([c for c, _ in self.w1() if c != "RPA"], ["PAA", "PID", "ASH", "ASH", "SLP", "PPA"])
    self.assertEqual(self.fake.plunger, 50000)

  async def test_head_drive_init_refused_when_lowered(self):
    self.fake.mca[2] = 1500
    with self.assertRaisesRegex(RuntimeError, "travel height"):
      await self.backend.mca_initialize_head_drive()
    self.assertEqual([c for c, _ in self.w1()], ["RPA"])

  async def test_move_to_resource(self):
    await self.backend.mca_move_to(self.troughs[9])
    self.assertEqual(self.fake.mca[:3], [2561, 2327, mca.Z_TRAVEL])

  async def test_plunger_jitter_within_tolerance_accepted(self):
    await self.lh.pick_up_tips96(self.racks[4])
    self.fake.plunger = 50002  # reads -51002 instead of -51000
    await self.lh.aspirate96(self.dw, volume=100)

  async def test_geometry_requires_evoware_config(self):
    backend = EVOBackend()
    with self.assertRaisesRegex(RuntimeError, "load_evoware_config"):
      backend.mca_geometry

  async def test_drop_into_trash_not_supported(self):
    await self.lh.pick_up_tips96(self.racks[4])
    with self.assertRaises(NotImplementedError):
      await self.backend.drop_tips96(type("D", (), {"resource": self.dw, "offset": None})())


if __name__ == "__main__":
  unittest.main()
