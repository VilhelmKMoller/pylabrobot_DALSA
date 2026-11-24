"""EVOBackend LiHa operations against a simulated controller (offline).

The wash, and the LiquidHandler operations pick_up_tips / aspirate / dispense / drop_tips on the
deck of the 2026-11-24 visual validation: plain "DiTi 1000ul LiHa" at grid 46 site 1, "Trough 300ml
MCA96" at grid 15 site 3, a 96 Well DeepWell at grid 21 site 3, the DiTi waste chute at grid 1.
"""

import json
import unittest
from unittest.mock import patch

from pylabrobot.legacy.liquid_handling import LiquidHandler
from pylabrobot.legacy.liquid_handling.backends.tecan import liha
from pylabrobot.legacy.liquid_handling.backends.tecan.EVO_backend import EVOBackend
from pylabrobot.legacy.liquid_handling.backends.tecan.errors import (
  TecanLihaNoLiquidError,
  TecanLihaTipNotFetchedError,
  error_code_to_exception,
)
from pylabrobot.legacy.liquid_handling.backends.tecan.evoware_liquid_classes import (
  LiquidClassLibrary,
)
from pylabrobot.legacy.liquid_handling.backends.tecan.liha_firmware import LihaFirmwareError
from pylabrobot.legacy.liquid_handling.backends.tecan.liha_tests import DATA
from pylabrobot.legacy.liquid_handling.backends.tecan.mca_tests import _TipDiameter
from pylabrobot.legacy.liquid_handling.errors import ChannelizedError
from pylabrobot.legacy.liquid_handling.standard import Drop, SingleChannelDispense
from pylabrobot.legacy.tip_tracker import TipTracker
from pylabrobot.resources import Coordinate, DeepWell_96_Well, EVO200Deck, Trash
from pylabrobot.resources.errors import HasTipError, NoTipError
from pylabrobot.resources.tecan.plate_carriers import MP_3Pos
from pylabrobot.resources.tecan.plates import Trough_300ml_MCA96
from pylabrobot.resources.tecan.tip_carriers import MCA96_Carrier_4_pos
from pylabrobot.resources.tecan.tip_racks import DiTi_1000ul_LiHa, DiTi_1000ul_LiHa_box
from pylabrobot.resources.tecan.wash import DiTi_Waste_3_Trough_Wash, Washstation_2Grid_DiTi_Waste

SURFACE = 1034  # liquid surface Z the fake reports after MDT/MET (as logged in the trough)


class FakeController:
  """Simulates the LiHa (DiTis, plungers, Z) and the reads the X coordinator needs; records traffic.

  ``fail``: {C5 command: (error code, REE reply or None)}, raised once on that command. For AGT the
  channels in ``agt_misses`` are not fetched."""

  def __init__(self, mounted=0, plunger=30):
    self.mounted = mounted
    self.plunger = [plunger] * 8
    self.z = [2100] * 8
    self.x = {"C5": 45, "C1": 9000}
    self.y = [1031, 90]  # LiHa Y and tip spacing (RPY0)
    self.mca = [6000, 1000, 2370, 900]
    self.sent = []
    self.fail = {}
    self.agt_misses = 0
    self.ree = "@" * 11

  @staticmethod
  def _tips(values):
    return [(i, v) for i, v in enumerate(values[:8]) if v is not None]

  async def send_command(self, module, command, params=None, **kw):
    params = list(params or [])
    self.sent.append((module, command, params))
    if module == "C5" and command in self.fail:
      code, ree = self.fail.pop(command)
      if command == "AGT":
        self.mounted |= params[0] & ~self.agt_misses
      if ree is not None:
        self.ree = ree
      raise error_code_to_exception("C5", code)
    if command == "RPX":
      return {"module": module, "data": [self.x[module]]}
    if command == "RPA":
      return {"module": module, "data": list(self.mca)}
    if command == "PAX":
      self.x[module] = params[0]
      return {"module": module, "data": []}
    if module == "W1" and command == "PAA":  # the X coordinator moves the MCA aside / up
      for i, v in enumerate(params[:3]):
        if v is not None:
          self.mca[i] = v
      return {"module": module, "data": []}
    if module != "C5":
      if command == "RPZ":
        return {"module": module, "data": [2464 if params[0] == 0 else 2540]}
      return {"module": module, "data": []}
    data: list = []
    if command == "RTS":
      data = [self.mounted]
    elif command == "REE":
      data = [self.ree]
    elif command == "RPZ":
      data = list(self.z) if params[0] == 0 else [2100] * 8
    elif command == "RPP":
      data = list(self.plunger)
    elif command == "RPY":
      data = list(self.y)
    elif command == "PAA":
      self.x["C5"] = params[0]
      self.y = [params[1], params[2]]
      for i, z in self._tips(params[3:]):
        self.z[i] = z
    elif command in ("MAZ", "PAZ"):
      for i, z in self._tips(params):
        self.z[i] = z
    elif command in ("MDT", "MET"):
      for i in range(8):
        if params[0] >> i & 1:
          self.z[i] = SURFACE
    elif command == "PPA":
      for i, v in self._tips(params):
        self.plunger[i] = v
    elif command in ("PPR", "MTR"):
      for i, v in self._tips(params):
        self.plunger[i] += v
    elif command == "AGT":
      self.mounted |= params[0] & ~self.agt_misses
    elif command == "AST":
      self.mounted &= ~params[0]
    return {"module": module, "data": data}


class _Backend(unittest.IsolatedAsyncioTestCase):
  def setUp(self):
    super().setUp()
    import asyncio as _asyncio

    real_sleep = _asyncio.sleep

    async def no_wait(_):
      await real_sleep(0)

    p = patch("pylabrobot.legacy.liquid_handling.backends.tecan.EVO_backend.asyncio.sleep", no_wait)
    p.start()
    self.addCleanup(p.stop)
    self.backend = EVOBackend(diti_count=8)
    self.backend.load_evoware_config(DATA / "evoware")
    self.backend._liha_connected = True
    self.backend._num_channels = 8
    self.backend._z_range = 2100
    self.fake = FakeController()
    self.backend.send_command = self.fake.send_command  # type: ignore[method-assign,assignment]

  def c5(self, *codes):
    out = [liha.format_command(c, p) for m, c, p in self.fake.sent if m == "C5"]
    return [c for c in out if not codes or c[:3] in codes]


class LihaWashBackendTests(_Backend):
  async def test_wash_sends_evoware_sequence(self):
    await self.backend.liha_wash_tips()
    fixture = json.loads((DATA / "liha_logs" / "wash_12ml_8ml_grid1.json").read_text())
    expected = liha.normalize_log([cmd for cmd, *_ in fixture["pairs"]])
    sent = [c for c in self.c5() if not c.startswith(("RPX", "RPY", "RPZ"))]
    self.assertEqual(sent[0], "RTS")  # tip check before anything moves
    self.assertEqual(sent[1:], expected)
    self.assertEqual(self.fake.x["C5"], -18)

  async def test_refused_with_tips_mounted(self):
    self.fake.mounted = 255
    with self.assertRaisesRegex(RuntimeError, "disposable tips mounted"):
      await self.backend.liha_wash_tips()
    self.assertEqual(self.c5(), ["RTS"])

  async def test_needs_config(self):
    self.backend.evoware_config = None  # type: ignore[assignment]
    with self.assertRaisesRegex(RuntimeError, "load_evoware_config"):
      await self.backend.liha_wash_tips()


def validation_deck():
  """The 2026-11-24 deck. PLR lists carrier sites front first: index = sites - EVOware site."""
  deck = EVO200Deck()
  carrier = MCA96_Carrier_4_pos("mca_carrier_grid46")
  deck.assign_child_resource(carrier, rails=46)
  rack = DiTi_1000ul_LiHa("liha_tips")
  carrier[3] = rack  # EVOware site 1 (back)
  mp15 = MP_3Pos("mp_grid15")
  deck.assign_child_resource(mp15, rails=15)
  trough = Trough_300ml_MCA96("trough_grid15")
  mp15[0] = trough  # EVOware site 3
  mp21 = MP_3Pos("mp_grid21")
  deck.assign_child_resource(mp21, rails=21)
  plate = DeepWell_96_Well("deepwell_grid21")
  mp21[0] = plate  # EVOware site 3
  # carrier 322 replaces PLR's default wash station at grid 1
  deck.get_resource("wash_station").unassign()
  wash = DiTi_Waste_3_Trough_Wash("wash_carrier_grid1")
  deck.assign_child_resource(wash, rails=1)
  wash[0] = Washstation_2Grid_DiTi_Waste("trash")  # EVOware site 7: the DiTi chute
  return deck, rack, trough, plate


class DeckModelTests(_TipDiameter, unittest.TestCase):
  """The PLR resources of the 2026-11-24 deck match the instrument's EVOware configuration."""

  def setUp(self):
    super().setUp()
    from pylabrobot.legacy.liquid_handling.backends.tecan import evoware_config

    self.config = evoware_config.load(DATA / "evoware")

  def test_carrier_322_sites_match_carrier_cfg(self):
    """PLR site i = EVOware site 7 - i; x = site x, y = carrier depth - site y - site depth,
    z = carrier z + site z, size = width x depth (all / 10)."""
    evo = self.config.carrier("DiTi Waste 3 Trough + Wash")
    plr = DiTi_Waste_3_Trough_Wash("c")
    self.assertEqual(len(plr.sites), 7)
    self.assertEqual(evo.sites[7].width, 0)  # Carrier.cfg's empty 8th site
    self.assertEqual((plr.off_x, plr.off_y), (evo.x / 10, evo.y / 10))
    for index, holder in plr.sites.items():
      s = evo.sites[6 - index]
      with self.subTest(evoware_site=7 - index):
        loc = holder.location
        self.assertAlmostEqual(loc.x, s.x / 10)
        self.assertAlmostEqual(loc.y, (3520 - s.y - s.depth) / 10)
        self.assertAlmostEqual(loc.z, (evo.z + s.z) / 10)
        self.assertAlmostEqual(holder.get_size_x(), s.width / 10)
        self.assertAlmostEqual(holder.get_size_y(), s.depth / 10)

  def test_liha_tip_rack_matches_carrier_cfg(self):
    lw = self.config.labware_def("DiTi 1000ul LiHa")
    rack = DiTi_1000ul_LiHa("r")
    self.assertEqual(
      (rack.z_start, rack.z_dispense, rack.z_max, rack.area),
      (lw.z_start, lw.z_dispense, lw.z_max, lw.area),
    )
    self.assertEqual(round(rack.get_tip("A1").get_size_z() * 10), lw.tip_length)

  def test_deck_resolves_to_the_evoware_sites(self):
    deck, rack, trough, plate = validation_deck()
    backend = EVOBackend(diti_count=8)
    backend.load_evoware_config(DATA / "evoware")
    self.assertEqual(backend._liha_site(deck.get_trash_area()), liha.DITI_WASTE)
    self.assertEqual(backend._liha_site(rack), RACK)
    self.assertEqual(backend._liha_site(trough), TROUGH)
    self.assertEqual(backend._liha_site(plate), PLATE)


RACK = liha.LihaSite(46, 0, "MCA96 Carrier 4 pos", "DiTi 1000ul LiHa")
MIX_CLASS = ">> Water Mix Gentle <<\t11"  # the instrument's mix class (test_data CustomLCs.XML)
TROUGH = liha.LihaSite(15, 2, "MP 3Pos", "Trough 300ml MCA96")
PLATE = liha.LihaSite(21, 2, "MP 3Pos", "96 Well DeepWell")
DITI = "DiTi 1000ul LiHa"


class LihaOperationTests(_TipDiameter, _Backend):
  def setUp(self):
    _Backend.setUp(self)
    _TipDiameter.setUp(self)  # does not chain to super()
    self.deck, self.rack, self.trough, self.plate = validation_deck()
    self.lh = LiquidHandler(backend=self.backend, deck=self.deck)
    # LiquidHandler state without the hardware setup() of the backend
    self.lh.head = {c: TipTracker(thing=f"Channel {c}") for c in range(8)}
    self.lh.head96 = {}
    self.backend.set_heads(head=self.lh.head, head96=self.lh.head96)
    self.backend.set_deck(self.deck)
    self.lh._setup_finished = True
    self.geo = self.backend._liha_geometry()
    self.lib = LiquidClassLibrary.from_database(DATA / "evoware")

  def moves(self):
    """C5 commands without the reports (RPX/RPZ/RPP from the backend and the X coordinator, RTS)."""
    return [c for c in self.c5() if not c.startswith(("RPX", "RPY", "RPZ", "RPP", "RTS", "REE"))]

  @staticmethod
  def without_rts(commands):
    """Planned commands as sent: no RTS, trailing empty fields trimmed by the firmware layer."""
    return [c.rstrip(",") for c in commands if c != "RTS"]

  async def test_full_cycle_sends_the_planned_commands(self):
    tips = list(range(1, 9))
    lc = self.lib.liha("Water free dispense", 150)

    await self.lh.pick_up_tips(self.rack["A1:H1"])
    self.assertTrue(all(t.has_tip for t in self.lh.head.values()))
    self.assertEqual(self.fake.mounted, 255)
    expected = liha.plan_strings(liha.plan_get_ditis(self.geo, RACK, 1, 1, tips))
    self.assertIn("PAA11347,-348,90," + ",".join(["2100"] * 8), expected)
    self.assertIn("AGT255,783,210,0", expected)
    self.assertEqual(self.moves(), self.without_rts(expected))
    self.assertEqual(self.c5()[0], "RTS")  # tip check before anything moves

    self.fake.sent.clear()
    await self.lh.aspirate([self.trough.get_item(0)] * 8, vols=[150] * 8, spread="custom")
    asp = liha.plan_aspirate(self.geo, lc, TROUGH, 1, 1, tips, 150, DITI, [30] * 8)
    expected = liha.plan_strings(asp.search) + liha.plan_strings(asp.finish([SURFACE] * 8))
    self.assertIn("PAA3563,2022,96," + ",".join(["1411"] * 8), expected)
    self.assertEqual(self.moves(), self.without_rts(expected))
    held = liha.aspirated_steps(lc, self.geo, 150, 30)
    self.assertEqual(self.fake.plunger, [held] * 8)

    self.fake.sent.clear()
    await self.lh.dispense(self.plate["A1:H1"], vols=[150] * 8)
    disp = liha.plan_dispense(self.geo, lc, PLATE, 1, 1, tips, DITI, [held] * 8)
    self.assertIsNone(disp.finish_args)  # free dispense
    self.assertEqual(self.moves(), self.without_rts(liha.plan_strings(disp.first)))
    self.assertEqual(self.fake.plunger, [60] * 8)  # everything above the 20 uL STAG

    self.fake.sent.clear()
    await self.lh.discard_tips()
    self.assertFalse(any(t.has_tip for t in self.lh.head.values()))
    self.assertEqual(self.fake.mounted, 0)
    expected = liha.plan_strings(liha.plan_drop_ditis(self.geo, tips))
    self.assertIn("PAA126,2810,90," + ",".join(["1075"] * 8), expected)
    self.assertEqual(self.moves(), self.without_rts(expected))

  async def test_wet_contact_dispense_follows_the_detected_surface(self):
    tips = [1, 2, 3, 4]
    channels = [0, 1, 2, 3]
    lc = self.lib.liha("Water wet contact", 100)
    await self.lh.pick_up_tips(self.rack["A1:D1"], use_channels=channels)
    await self.lh.aspirate(
      [self.trough.get_item(0)] * 4,
      vols=[100] * 4,
      spread="custom",
      use_channels=channels,
      liquid_class="Water wet contact",
    )
    held = list(self.fake.plunger)
    self.fake.sent.clear()
    await self.lh.dispense(self.plate["A2:D2"], vols=[100] * 4, use_channels=channels)
    disp = liha.plan_dispense(self.geo, lc, PLATE, 2, 1, tips, DITI, held)
    self.assertIsNotNone(disp.finish_args)
    expected = liha.plan_strings(disp.first) + liha.plan_strings(disp.finish([SURFACE] * 8))
    self.assertEqual(self.moves(), self.without_rts(expected))
    self.assertTrue(any(c.startswith("MET15,") for c in self.moves()))

  async def test_every_liha_x_move_clears_the_path_first(self):
    order = []
    coordinator = self.backend.x_coordinator()
    real_clear = coordinator.clear_path

    async def clear(name, target):
      order.append(("clear", name, target))
      return await real_clear(name, target)

    coordinator.clear_path = clear  # type: ignore[method-assign]
    real_send = self.fake.send_command

    async def send(module, command, params=None, **kw):
      if module == "C5" and command == "PAA":
        order.append(("PAA", "liha", params[0]))
      return await real_send(module, command, params, **kw)

    self.backend.send_command = send  # type: ignore[method-assign,assignment]
    await self.lh.pick_up_tips(self.rack["A1:H1"])
    await self.lh.aspirate([self.trough.get_item(0)] * 8, vols=[150] * 8, spread="custom")
    await self.lh.dispense(self.plate["A1:H1"], vols=[150] * 8)
    await self.lh.discard_tips()
    self.assertEqual(len(order), 8)
    for i in range(0, len(order), 2):
      self.assertEqual(order[i], ("clear", "liha", order[i + 1][2]))
      self.assertEqual(order[i + 1][0], "PAA")

  async def test_missed_ditis_are_reported_per_channel(self):
    self.fake.fail["AGT"] = (25, None)
    self.fake.agt_misses = 0b00100100  # channels 2 and 5
    with self.assertRaises(ChannelizedError) as ctx:
      await self.lh.pick_up_tips(self.rack["A1:H1"])
    self.assertEqual(set(ctx.exception.errors), {2, 5})
    self.assertTrue(all(isinstance(e, NoTipError) for e in ctx.exception.errors.values()))
    # the firmware layer asked which DiTis are on, then the tips went straight up (as EVOware's
    # abort: RTS, REE, PAZ to the Z range)
    self.assertEqual(self.c5()[-2:], ["RTS", "PAZ" + ",".join(["2100"] * 8)])
    self.assertIn("raised to Z 2100", " ".join(getattr(ctx.exception.__cause__, "__notes__", [])))

  async def test_error_25_with_every_diti_on_is_not_a_pickup(self):
    """2026-11-23, AGT start 710: error 25 although RTS shows all 8 DiTis on (EVOware 2024-09-20
    retried the tips REE flagged: error 27). PLR counts none as picked up and
    points to liha_discard_mounted_tips."""
    self.fake.fail["AGT"] = (25, None)  # every requested DiTi on: RTS cannot say which failed
    with self.assertRaises(TecanLihaTipNotFetchedError) as ctx:
      await self.lh.pick_up_tips(self.rack["A1:H1"])
    self.assertEqual(self.fake.mounted, 255)
    notes = " ".join(getattr(ctx.exception, "__notes__", []))
    self.assertIn("liha_discard_mounted_tips", notes)
    self.assertIn("[0, 1, 2, 3, 4, 5, 6, 7]", notes)
    self.assertFalse(any(self.lh.head[c].has_tip for c in range(8)))
    # recovery: drop them into the chute, then pick up again
    self.fake.sent.clear()
    self.assertEqual(await self.backend.liha_discard_mounted_tips(), 255)
    self.assertIn("AST255,0", self.c5())
    self.assertEqual(self.fake.mounted, 0)
    self.assertEqual(await self.backend.liha_discard_mounted_tips(), 0)  # nothing on: no motion
    await self.lh.pick_up_tips(self.rack["A2:H2"])
    self.assertEqual(self.fake.mounted, 255)

  async def test_discard_mounted_tips_spans_the_gaps_and_clears_tracking(self):
    await self.lh.pick_up_tips(self.rack["A1:H1"])
    self.fake.mounted = 0b00100100  # channels 2 and 5 only (e.g. after a failed set-back)
    self.fake.sent.clear()
    self.assertEqual(await self.backend.liha_discard_mounted_tips(), 0b00100100)
    self.assertIn("AST60,0", self.c5())  # tips 3-6
    self.assertEqual(self.fake.mounted, 0)
    self.assertFalse(any(self.lh.head[c].has_tip for c in (2, 3, 4, 5)))
    self.assertTrue(self.lh.head[0].has_tip)  # outside the span: PLR's tracking unchanged

  def _dispense_op(self, channel):
    return SingleChannelDispense(
      resource=self.plate.get_well(channel),
      offset=Coordinate.zero(),
      tip=self.lh.head[channel].get_tip(),
      volume=150,
      flow_rate=None,
      liquid_height=None,
      blow_out_air_volume=None,
      mix=None,
    )

  async def test_no_liquid_is_reported_per_channel(self):
    await self.lh.pick_up_tips(self.rack["A1:H1"])
    self.fake.fail["MDT"] = (9, "@@@II@@@@@@")  # Z1, Z2: no liquid
    with self.assertRaises(ChannelizedError) as ctx:
      await self.lh.aspirate([self.trough.get_item(0)] * 8, vols=[150] * 8, spread="custom")
    self.assertEqual(set(ctx.exception.errors), {0, 1})
    for error in ctx.exception.errors.values():
      self.assertIsInstance(error, TecanLihaNoLiquidError)
    self.assertEqual(self.c5()[-2:], ["REE", "PAZ" + ",".join(["2100"] * 8)])
    self.assertEqual(self.fake.z, [2100] * 8)
    self.assertFalse(any(c.startswith("MTR") for c in self.c5()))
    # the contents are unknown now: no dispense, only dropping the DiTis
    with self.assertRaisesRegex(RuntimeError, "known aspiration"):
      await self.backend.dispense([self._dispense_op(c) for c in range(8)], list(range(8)))
    await self.lh.discard_tips()
    self.assertEqual(self.fake.mounted, 0)

  async def test_detect_liquid_reports_the_surface_and_moves_no_plunger(self):
    tips = list(range(1, 9))
    await self.lh.pick_up_tips(self.rack["A1:H1"])
    plunger = list(self.fake.plunger)
    self.fake.sent.clear()
    found = await self.backend.liha_detect_liquid([self.trough.get_item(0)] * 8, list(range(8)))
    self.assertEqual(found, {c: SURFACE for c in range(8)})
    lc = self.lib.liha("Water free dispense", 100)
    search, retract = liha.plan_detect_liquid(self.geo, lc, TROUGH, 1, 1, tips, DITI)
    expected = liha.plan_strings(search) + liha.plan_strings(retract)
    self.assertEqual(self.moves(), self.without_rts(expected))
    self.assertIn("SBL" + ",".join(["0"] * 8), expected)  # stop at the surface
    self.assertIn("MDT255,,,," + ",".join(["0"] * 8), expected)
    self.assertFalse(self.c5("PPR", "MTR", "PPA", "PVL"))
    self.assertEqual(self.fake.plunger, plunger)
    self.assertEqual(self.fake.z, [self.geo.z(TROUGH, "z_travel", DITI)] * 8)
    # the tips still hold only air: an aspirate may follow
    await self.lh.aspirate([self.trough.get_item(0)] * 8, vols=[150] * 8, spread="custom")

  async def test_detect_liquid_without_liquid(self):
    await self.lh.pick_up_tips(self.rack["A1:H1"])
    self.fake.fail["MDT"] = (9, "@@@IIIIIIII")
    with self.assertRaises(ChannelizedError) as ctx:
      await self.backend.liha_detect_liquid([self.trough.get_item(0)] * 8, list(range(8)))
    self.assertEqual(set(ctx.exception.errors), set(range(8)))
    self.assertEqual(self.fake.z, [2100] * 8)  # raised after the error

  async def test_no_raise_after_a_crash(self):
    """Error 10 (drive no load, e.g. a collision): the arm stays where it stopped."""
    await self.lh.pick_up_tips(self.rack["A1:H1"])
    self.fake.fail["PAA"] = (10, None)
    with self.assertRaises(Exception):
      await self.lh.aspirate([self.trough.get_item(0)] * 8, vols=[150] * 8, spread="custom")
    self.assertFalse(any(c.startswith("PAZ") for c in self.c5()))

  async def test_refused_before_anything_is_sent(self):
    await self.lh.pick_up_tips(self.rack["A1:H1"])
    self.fake.sent.clear()
    for what, kwargs in [
      ("flow_rate", dict(flow_rates=[100] * 8)),
      ("liquid_height", dict(liquid_height=[2] * 8)),
      ("blow_out_air_volume", dict(blow_out_air_volume=[5] * 8)),
    ]:
      with self.subTest(what), self.assertRaisesRegex(NotImplementedError, what):
        await self.lh.aspirate(self.plate["A1:H1"], vols=[100] * 8, **kwargs)
    with self.assertRaisesRegex(NotImplementedError, "same volume"):
      await self.lh.aspirate(self.plate["A1:H1"], vols=[100] * 7 + [50])
    with self.assertRaisesRegex(NotImplementedError, "consecutive rows"):
      await self.lh.aspirate(self.plate["A1:G1"] + self.plate["A2:A2"], vols=[100] * 8)
    with self.assertRaisesRegex(NotImplementedError, "offsets"):
      await self.lh.aspirate(self.plate["A1:H1"], vols=[100] * 8, offsets=[Coordinate(1, 0, 0)] * 8)
    self.assertEqual(self.c5(), [])

  async def test_partial_dispense_refused(self):
    await self.lh.pick_up_tips(self.rack["A1:H1"])
    await self.lh.aspirate([self.trough.get_item(0)] * 8, vols=[150] * 8, spread="custom")
    self.fake.sent.clear()
    with self.assertRaisesRegex(NotImplementedError, "partial dispense"):
      await self.lh.dispense(self.plate["A1:H1"], vols=[100] * 8)
    self.assertEqual(self.c5(), [])

  async def test_second_aspiration_refused(self):
    await self.lh.pick_up_tips(self.rack["A1:H1"])
    await self.lh.aspirate([self.trough.get_item(0)] * 8, vols=[150] * 8, spread="custom")
    with self.assertRaisesRegex(RuntimeError, "dispense first"):
      await self.lh.aspirate([self.trough.get_item(0)] * 8, vols=[150] * 8, spread="custom")

  async def test_ditis_already_on_refused(self):
    self.fake.mounted = 0b1
    with self.assertRaises(ChannelizedError) as ctx:
      await self.lh.pick_up_tips(self.rack["A1:H1"])
    self.assertEqual(list(ctx.exception.errors), [0])
    self.assertIsInstance(ctx.exception.errors[0], HasTipError)
    self.assertEqual(self.c5(), ["RTS"])

  async def test_return_tips_sets_them_back_and_they_can_be_picked_up_again(self):
    """EVOware "Set DiTis Back": the fetch position at Z 1075, AST; then "Pick Up DiTis" = fetch."""
    tips = list(range(1, 9))
    await self.lh.pick_up_tips(self.rack["A3:H3"])
    plunger = list(self.fake.plunger)
    self.fake.sent.clear()
    await self.lh.return_tips()
    expected = liha.plan_strings(liha.plan_set_ditis_back(self.geo, RACK, 3, 1, tips))
    fetch = liha.plan_strings(liha.plan_get_ditis(self.geo, RACK, 3, 1, tips))
    paa = next(c for c in fetch if c.startswith("PAA"))
    self.assertIn(paa.rsplit(",", 8)[0] + "," + ",".join(["1075"] * 8), expected)
    self.assertEqual(self.moves(), self.without_rts(expected))
    self.assertEqual(self.c5("AST"), ["AST255,0"])
    self.assertEqual(self.fake.mounted, 0)
    self.assertEqual(self.fake.plunger, plunger)  # the plungers are not touched
    self.assertTrue(all(self.rack.get_item(f"{r}3").has_tip() for r in "ABCDEFGH"))
    self.fake.sent.clear()
    await self.lh.pick_up_tips(self.rack["A3:H3"])
    self.assertEqual(self.moves(), self.without_rts(fetch))
    self.assertEqual(self.fake.mounted, 255)

  async def test_set_back_partial_column_and_box_correction(self):
    box = self._conductive_box()
    await self.lh.pick_up_tips(box["C5:E5"], use_channels=[2, 3, 4])
    self.fake.sent.clear()
    await self.lh.drop_tips(box["C5:E5"], use_channels=[2, 3, 4])
    z = ["2100", "2100", "1075", "1075", "1075", "2100", "2100", "2100"]
    # X includes the box's +3 mm; the eject Z is EVOware's fixed 1075 (no box Z correction)
    self.assertEqual(self.c5("PAA"), ["PAA11737,-348,90," + ",".join(z)])
    self.assertEqual(self.c5("AST"), ["AST28,0"])
    self.assertEqual(self.fake.mounted, 0)

  async def test_ditis_with_a_channel_left_out(self):
    """Get DiTis / Set DiTis Back with tips 1-3 and 5 (EVOware: mask 247 and 23 logged)."""
    spots = [self.rack.get_item(w) for w in ("A1", "B1", "C1", "E1")]
    await self.lh.pick_up_tips(spots, use_channels=[0, 1, 2, 4])
    self.assertEqual(self.c5("AGT"), ["AGT23,783,210,0"])
    self.assertEqual(self.fake.mounted, 23)
    self.fake.sent.clear()
    await self.lh.drop_tips(spots, use_channels=[0, 1, 2, 4])
    z = ["1075"] * 3 + ["2100", "1075"] + ["2100"] * 3
    self.assertEqual(self.c5("PAA"), ["PAA11347,-348,90," + ",".join(z)])
    self.assertEqual(self.c5("AST"), ["AST23,0"])
    # liquid steps with a channel left out were never logged: refused before anything is sent
    await self.lh.pick_up_tips(spots, use_channels=[0, 1, 2, 4])
    self.fake.sent.clear()
    with self.assertRaisesRegex(NotImplementedError, "consecutive"):
      await self.lh.aspirate(
        [self.trough.get_item(0)] * 4, vols=[100] * 4, spread="custom", use_channels=[0, 1, 2, 4]
      )
    self.assertEqual(self.c5(), ["RTS", "RPP0"])  # tip and plunger reads only

  async def test_dispense_over_every_second_row(self):
    """EVOware's checker pattern: tips 1-4 over rows B, D, F, H, spacing twice the row pitch."""
    tips = [1, 2, 3, 4]
    lc = self.lib.liha("Water free dispense", 100)
    await self.lh.pick_up_tips(self.rack["A1:D1"], use_channels=[0, 1, 2, 3])
    await self.lh.aspirate(
      [self.trough.get_item(0)] * 4, vols=[100] * 4, spread="custom", use_channels=[0, 1, 2, 3]
    )
    held = list(self.fake.plunger)
    self.fake.sent.clear()
    wells = [self.plate.get_item(w) for w in ("B1", "D1", "F1", "H1")]
    await self.lh.dispense(wells, vols=[100] * 4, use_channels=[0, 1, 2, 3])
    disp = liha.plan_dispense(self.geo, lc, PLATE, 1, 2, tips, DITI, held, row_stride=2)
    self.assertEqual(self.moves(), self.without_rts(liha.plan_strings(disp.first)))
    spacing = self.geo.y_spacing(PLATE)
    self.assertEqual(int(self.c5("PAA")[0].split(",")[2]), 2 * spacing)
    with self.assertRaisesRegex(NotImplementedError, "consecutive rows"):
      await self.lh.aspirate(wells, vols=[100] * 4, use_channels=[0, 1, 2, 3])

  async def test_set_back_refused_with_liquid_in_the_tips(self):
    await self.lh.pick_up_tips(self.rack["A1:H1"])
    await self.lh.aspirate([self.trough.get_item(0)] * 8, vols=[150] * 8, spread="custom")
    self.fake.sent.clear()
    with self.assertRaisesRegex(RuntimeError, "dispense first"):
      await self.backend.drop_tips(
        [
          Drop(
            resource=self.rack.get_item(f"{r}1"),
            offset=Coordinate.zero(),
            tip=self.lh.head[i].get_tip(),
          )
          for i, r in enumerate("ABCDEFGH")
        ],
        list(range(8)),
      )
    self.assertEqual(self.c5(), [])
    self.assertEqual(self.fake.mounted, 255)

  async def test_site_offset_moves_only_that_site(self):
    self.backend.liha_site_offsets[RACK] = (10, 0)
    await self.lh.pick_up_tips(self.rack["A1:H1"])
    self.assertIn("PAA11357,-348,90," + ",".join(["2100"] * 8), self.c5("PAA"))
    await self.lh.aspirate([self.trough.get_item(0)] * 8, vols=[150] * 8, spread="custom")
    self.assertTrue(any(c.startswith("PAA3563,2022,96,") for c in self.c5("PAA")))

  async def test_site_z_offset_lowers_the_pickup(self):
    self.backend.liha_site_z_offsets[RACK] = -10
    await self.lh.pick_up_tips(self.rack["A1:H1"])
    self.assertEqual(self.c5("AGT"), ["AGT255,773,210,0"])

  def _conductive_box(self):
    """The box loaded on 2026-11-23: conductive tips in columns 1 and 12 only."""
    carrier = self.rack.parent.parent
    self.rack.unassign()
    box = DiTi_1000ul_LiHa_box("liha_box", conductive_columns=(1, 12))
    carrier[3] = box
    return box

  def test_box_conductive_columns(self):
    box = DiTi_1000ul_LiHa_box("b", conductive_columns=(1, 12))
    for column in range(1, 13):
      for row in "AH":
        tip = box.get_item(f"{row}{column}").get_tip()
        self.assertEqual(tip.is_conductive, column in (1, 12), f"{row}{column}")
    self.assertIsNone(DiTi_1000ul_LiHa_box("c").get_item("A1").get_tip().is_conductive)
    with self.assertRaises(ValueError):
      DiTi_1000ul_LiHa_box("d", conductive_columns=(0,))

  async def test_detecting_steps_refused_with_non_conductive_ditis(self):
    box = self._conductive_box()
    await self.lh.pick_up_tips(box["A2:H2"])  # fetching and dropping work with any column
    self.fake.sent.clear()
    with self.assertRaisesRegex(ValueError, "conductive"):
      await self.lh.aspirate([self.trough.get_item(0)] * 8, vols=[150] * 8, spread="custom")
    with self.assertRaisesRegex(ValueError, "conductive"):
      await self.backend.liha_mix(self.plate["A1:H1"], 150, 3, list(range(8)), MIX_CLASS)
    self.assertEqual(self.c5(), [])
    await self.lh.discard_tips()
    self.assertEqual(self.fake.mounted, 0)
    # a conductive column works: mix, aspirate, wet-contact dispense
    await self.lh.pick_up_tips(box["A12:H12"])
    await self.backend.liha_mix(self.plate["A1:H1"], 150, 3, list(range(8)), MIX_CLASS)
    await self.lh.aspirate(self.plate["A1:H1"], vols=[150] * 8, liquid_class="Water wet contact")
    await self.lh.dispense(self.plate["A1:H1"], vols=[150] * 8)
    self.assertEqual(self.fake.plunger, [60] * 8)

  async def test_liha_box_carries_its_correction(self):
    """The operator's 1000ul-diti-liha-box applies its measured correction wherever it is used."""
    carrier = self.rack.parent.parent
    self.rack.unassign()
    box = DiTi_1000ul_LiHa_box("liha_box")
    carrier[3] = box
    self.assertEqual(box.model, "1000ul-diti-liha-box")
    await self.lh.pick_up_tips(box["A1:H1"])
    self.assertIn("PAA11377,-348,90," + ",".join(["2100"] * 8), self.c5("PAA"))
    self.assertEqual(self.c5("AGT"), ["AGT255,700,210,0"])
    # other labware keeps the EVOware positions
    await self.lh.aspirate([self.trough.get_item(0)] * 8, vols=[150] * 8, spread="custom")
    self.assertTrue(any(c.startswith("PAA3563,2022,96,") for c in self.c5("PAA")))

  async def test_drop_only_into_the_chute(self):
    await self.lh.pick_up_tips(self.rack["A1:H1"])
    other = Trash("other_trash", size_x=20, size_y=100, size_z=10)
    self.deck.assign_child_resource(other, location=Coordinate(1200, 400, 0))
    self.fake.sent.clear()
    with self.assertRaises(ValueError):
      await self.lh.drop_tips([other] * 8)
    self.assertEqual(self.c5(), [])
    self.assertEqual(self.fake.mounted, 255)

  async def _mix(self, wells="A1:H1"):
    await self.backend.liha_mix(self.plate[wells], 150, 3, list(range(8)), MIX_CLASS)

  async def test_mix_then_aspirate_in_the_same_wells(self):
    """EVOware Mix (3 x 150 uL at z_max), then Aspirate where the tips are: no move, no air gap."""
    tips = list(range(1, 9))
    await self.lh.pick_up_tips(self.rack["A1:H1"])
    self.fake.sent.clear()
    await self._mix()
    mix = liha.plan_mix(
      self.geo, self.lib.liha(MIX_CLASS, 150), PLATE, 1, 1, tips, 150, 3, DITI, [30] * 8
    )
    expected = liha.plan_strings(mix.search) + liha.plan_strings(mix.finish([SURFACE] * 8))
    self.assertEqual(self.moves(), self.without_rts(expected))
    z_max = self.geo.z(PLATE, "z_max", DITI)
    self.assertIn("PAZ" + ",".join([str(z_max)] * 8), expected)
    self.assertEqual(
      self.c5("MTR"), ["MTR" + ",".join(["450"] * 8), "MTR" + ",".join(["-450"] * 8)] * 3
    )
    self.assertEqual(self.fake.plunger, [75] * 8)  # 30 + top-up to the 20 uL STAG + 5 uL LAG
    self.assertEqual(self.fake.z, [mix.retract_z] * 8)

    self.fake.sent.clear()
    await self.lh.aspirate(self.plate["A1:H1"], vols=[150] * 8, liquid_class="Water wet contact")
    lc = self.lib.liha("Water wet contact", 150)
    asp = liha.plan_aspirate(
      self.geo, lc, PLATE, 1, 1, tips, 150, DITI, [75] * 8, after_mix_z=mix.retract_z
    )
    expected = liha.plan_strings(asp.search) + liha.plan_strings(asp.finish([SURFACE] * 8))
    self.assertEqual(self.moves(), self.without_rts(expected))
    self.assertEqual(
      self.c5("PAA", "PPR", "PVL"), ["PVL" + ",".join(["0"] * 8), "PPR" + ",".join(["30"] * 8)]
    )  # only the TAG
    self.assertEqual(self.fake.plunger, [liha.aspirated_steps(lc, self.geo, 150, 75, True)] * 8)
    await self.lh.dispense(self.plate["A1:H1"], vols=[150] * 8)
    self.assertEqual(self.fake.plunger, [60] * 8)

  async def test_aspirate_in_other_wells_after_a_mix_moves_first(self):
    await self.lh.pick_up_tips(self.rack["A1:H1"])
    await self._mix("A1:H1")
    self.fake.sent.clear()
    await self.lh.aspirate(self.plate["A2:H2"], vols=[150] * 8, liquid_class="Water wet contact")
    self.assertEqual(len(self.c5("PAA")), 1)
    self.assertIn("PPR" + ",".join(["15"] * 8), self.c5("PPR"))  # LAG again (75 >= STAG)

  async def test_aspirate_after_a_mix_moves_back_if_the_arm_was_pushed(self):
    await self.lh.pick_up_tips(self.rack["A1:H1"])
    await self._mix()
    self.fake.x["C5"] += 500  # e.g. the X coordinator moved the LiHa aside for another arm
    self.fake.sent.clear()
    await self.lh.aspirate(self.plate["A1:H1"], vols=[150] * 8, liquid_class="Water wet contact")
    self.assertEqual(len(self.c5("PAA")), 1)

  async def test_mix_refused_before_anything_is_sent(self):
    with self.assertRaisesRegex(RuntimeError, "pick_up_tips"):
      await self._mix()
    await self.lh.pick_up_tips(self.rack["A1:H1"])
    self.fake.sent.clear()
    for cycles in (0, 1.5, True):
      with self.subTest(cycles=cycles), self.assertRaises(ValueError):
        await self.backend.liha_mix(self.plate["A1:H1"], 150, cycles, list(range(8)), MIX_CLASS)  # type: ignore[arg-type]
    with self.assertRaisesRegex(NotImplementedError, "consecutive rows"):
      await self.backend.liha_mix(
        self.plate["A1:G1"] + self.plate["A2:A2"], 150, 3, list(range(8)), MIX_CLASS
      )
    self.assertEqual(self.c5(), [])

  async def test_mix_at_the_liquid_level_tracks(self):
    """A class that aspirates at the liquid level ("Water free dispense") mixes where MET found the
    surface, tracking each stroke (EVO_20260205_141912)."""
    await self.lh.pick_up_tips(self.rack["A1:H1"])
    self.fake.sent.clear()
    await self.backend.liha_mix(self.plate["A1:H1"], 30, 2, list(range(8)), "Water free dispense")
    self.assertFalse(self.c5("PAZ"))  # no move to z_max
    stz = [c for c in self.c5() if c.startswith("STZ")]
    self.assertEqual(len(stz), 4)
    self.assertTrue(stz[0].startswith("STZ-") and stz[1].startswith("STZ") and stz[1] != "STZ0")
    met = next(c for c in self.c5() if c.startswith("MET"))
    self.assertNotIn(",0,0,0,0,0,0,0,0", met)  # Zadd = submerge + tracking

  async def test_mix_without_liquid_is_reported_per_channel(self):
    await self.lh.pick_up_tips(self.rack["A1:H1"])
    self.fake.fail["MET"] = (9, "@@@II@@@@@@")
    with self.assertRaises(ChannelizedError) as ctx:
      await self._mix()
    self.assertEqual(set(ctx.exception.errors), {0, 1})
    self.assertFalse(any(c.startswith("MTR") for c in self.c5()))
    with self.assertRaisesRegex(RuntimeError, "dispense first"):
      await self.lh.aspirate(self.plate["A1:H1"], vols=[150] * 8, liquid_class="Water wet contact")
    await self.lh.discard_tips()

  async def test_firmware_layer_refuses_a_bad_plan_before_the_first_command(self):
    plan = liha.Plan()
    plan.add("SHZ", *[2100] * 8)
    plan.add("AGT", 255, 783, 900, 0)  # SearchDist above Zstart
    with self.assertRaises(LihaFirmwareError):
      await self.backend._liha_run(plan)
    self.assertEqual(self.c5(), [])


if __name__ == "__main__":
  unittest.main()
