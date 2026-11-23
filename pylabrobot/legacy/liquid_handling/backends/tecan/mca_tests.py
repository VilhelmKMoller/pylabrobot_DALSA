"""MCA command plans vs. EVOware USB captures (golden tests, offline).

Geometry comes from the instrument's EVOware configuration (``test_data/evoware``,
a subset of its Carrier.cfg and evoware.inf); expected commands come from the
USB captures in ``test_data/mca_captures``.
"""

import difflib
import json
from pathlib import Path
import unittest

from pylabrobot.legacy.liquid_handling.backends.tecan import evoware_config, mca
from pylabrobot.legacy.liquid_handling.backends.tecan.evoware_liquid_classes import LiquidClassLibrary
from pylabrobot.resources import Coordinate, EVO200Deck
from pylabrobot.resources.tecan import tip_creators
from pylabrobot.resources.tecan.plate_carriers import MP_3Pos
from pylabrobot.resources.tecan.plates import DeepWell_96_Well, Trough_300ml_MCA96
from pylabrobot.resources.tecan.tip_carriers import MCA96_Carrier_4_pos
from pylabrobot.resources.tecan.tip_racks import DiTi_50ul_MCA96, DiTi_200ul_MCA96

DATA = Path(__file__).parent / "test_data"
CAPTURES = DATA / "mca_captures"
TEST_TIP_DIAMETER = 5.0  # test only: PLR's MCA96 tips have no diameter defined
TIP200 = "DiTi_200ul_MCA96_tip"
TIP50 = "DiTi_50ul_MCA96_tip"


def geometry():
  return mca.McaGeometry(evoware_config.load(DATA / "evoware"))


LIBRARY = LiquidClassLibrary.from_database(DATA / "evoware")
RACK = {TIP200: "DiTi 200ul MCA96", TIP50: "DiTi 50ul MCA96"}


def liquid_class(tip, volume):
  return LIBRARY.mca(mca.DEFAULT_CLASS[RACK[tip]], volume)


def capture(name):
  return [c for c, _, _ in json.loads((CAPTURES / f"{name}.json").read_text())["pairs"]]


def wire(plan):
  return [mca.format_command(c, p) for c, p in plan.commands]


class _TipDiameter:
  """Give PLR's MCA96 tips a diameter for the duration of the tests."""

  def setUp(self):
    self._orig = tip_creators.TecanTip.__init__

    def init(tip, *a, **k):
      if k.get("diameter") is None:
        k["diameter"] = TEST_TIP_DIAMETER
      self._orig(tip, *a, **k)

    tip_creators.TecanTip.__init__ = init
    self.addCleanup(setattr, tip_creators.TecanTip, "__init__", self._orig)
    self.geo = geometry()


def capture_deck(cleaning=False):
  """Deck matching the EVOware captures: REFINES worktable (DeepWell at grid 21
  site 3), or with ``cleaning`` the tip-cleaning layout (300 ml troughs at grids
  9/15/21 site 3). Tip boxes: 200 uL on grid 46 sites 1-4, 50 uL on grid 58 site 4.
  Returns (deck, {evoware site: 200 uL rack}, 50 uL rack, deepwell, {grid: trough})."""
  deck = EVO200Deck()
  tc46 = MCA96_Carrier_4_pos("tips46")
  tc58 = MCA96_Carrier_4_pos("tips58")
  deck.assign_child_resource(tc46, rails=46)
  deck.assign_child_resource(tc58, rails=58)
  racks200 = {}
  for site in (1, 2, 3, 4):
    racks200[site] = DiTi_200ul_MCA96(f"tips200_site{site}")
    tc46[4 - site] = racks200[site]  # PLR index 0 = front = EVOware site 4
  rack50 = DiTi_50ul_MCA96("tips50")
  tc58[0] = rack50
  troughs, dw = {}, None
  for rails in (9, 15, 21):
    mp = MP_3Pos(f"mp{rails}")
    deck.assign_child_resource(mp, rails=rails)
    if rails == 21 and not cleaning:
      dw = DeepWell_96_Well("deepwell")
      mp[0] = dw  # EVOware site 3
    else:
      troughs[rails] = Trough_300ml_MCA96(f"trough{rails}")
      mp[0] = troughs[rails]
  return deck, racks200, rack50, dw, troughs


TRAVEL = (6000, 1000, mca.Z_TRAVEL)  # head raised elsewhere


class ConfigTests(unittest.TestCase):
  def test_parse(self):
    cfg = evoware_config.load(DATA / "evoware")
    self.assertEqual(cfg.arms["MCA"], evoware_config.ArmOffsets(498, 305, 2105))
    c = cfg.carrier("MCA96 Carrier 4 pos")
    self.assertEqual((c.id, c.x, c.y, c.z, len(c.sites)), (325, 125, 515, 60, 4))
    self.assertEqual([s.x for s in c.sites], [100, 98, 94, 92])
    t = cfg.labware_def("Trough 300ml MCA96")
    self.assertEqual((t.z_max, t.z_start, t.z_dispense, t.z_travel), (2034, 1791, 1857, 1640))
    self.assertEqual(cfg.labware_def("DiTi 200ul MCA96").tip_length, 431)


class GeometryTests(_TipDiameter, unittest.TestCase):
  def test_xy_match_every_captured_position(self):
    _, racks, rack50, dw, _ = capture_deck()
    expected = {1: (11879, -52), 2: (11877, 908), 3: (11873, 1868), 4: (11871, 2828)}
    for site, xy in expected.items():
      self.assertEqual(self.geo.xy(racks[site]), xy, f"site {site}")
    self.assertEqual(self.geo.xy(rack50), (14864, 2831))
    self.assertEqual(self.geo.xy(dw), (5577, 2340))
    _, _, _, _, troughs = capture_deck(cleaning=True)
    self.assertEqual([self.geo.xy(troughs[g]) for g in (9, 15, 21)], [(2561, 2327), (4061, 2327), (5561, 2327)])

  def test_trough_well_resolves_to_trough(self):
    _, _, _, _, troughs = capture_deck(cleaning=True)
    self.assertEqual(self.geo.xy(troughs[9].get_item(0)), (2561, 2327))

  def test_operator_deck_trough_grid9_site1(self):
    deck = EVO200Deck()
    mp = MP_3Pos("mp9")
    deck.assign_child_resource(mp, rails=9)
    trough = Trough_300ml_MCA96("trough")
    mp[2] = trough  # PLR index 2 = EVOware site 1 (rear)
    self.assertEqual(self.geo.xy(trough), (2561, 407))

  def test_offset_mm(self):
    _, racks, _, _, _ = capture_deck()
    self.assertEqual(self.geo.xy(racks[4], Coordinate(1, 2, 0)), (11881, 2808))

  def test_z_heights(self):
    _, racks, _, dw, troughs = capture_deck()
    t = mca.LiquidTarget(self.geo, dw, Coordinate.zero(), TIP200)
    self.assertEqual([t.z(k) for k in ("approach", "bottom", "retract", "dispense")], [1591, 1121, 1511, 1491])
    t50 = mca.LiquidTarget(self.geo, dw, Coordinate.zero(), TIP50)
    self.assertEqual([t50.z(k) for k in ("approach", "bottom", "retract")], [1456, 986, 1376])
    tt = mca.LiquidTarget(self.geo, troughs[9], Coordinate.zero(), TIP200)
    self.assertEqual([tt.z("approach"), tt.z("bottom"), tt.z("dispense")], [1521, 1127, 1304])
    self.assertEqual((self.geo.z(racks[4], "z_start"), self.geo.z(racks[4], "z_max")), (714, 520))

  def test_liquid_height_raises_bottom(self):
    _, _, _, dw, _ = capture_deck()
    t = mca.LiquidTarget(self.geo, dw, Coordinate.zero(), TIP200, liquid_height=2.5)
    self.assertEqual(t.z("bottom"), 1146)

  def test_unknown_labware_rejected(self):
    with self.assertRaises(ValueError):
      self.geo.xy(EVO200Deck())


class GoldenTests(_TipDiameter, unittest.TestCase):
  def assertMatchesCapture(self, plan, name, start=None):
    ours, ours_xy = mca.normalize(wire(plan))
    theirs, theirs_xy = mca.normalize(capture(name))
    self.assertEqual(ours, theirs)
    # EVOware re-sends XY moves to the position the head is already at; we skip them.
    skip = f"PAA{start[0]},{start[1]}" if start else None
    self.assertEqual(ours_xy, [xy for xy in theirs_xy if xy != skip])

  def test_pick_up_tips200(self):
    _, racks, _, _, _ = capture_deck()
    plan = mca.plan_pick_up(self.geo, racks[4], Coordinate.zero(), TRAVEL, mca.PlungerState())
    self.assertMatchesCapture(plan, "get_tips200")

  def test_pick_up_tips50(self):
    _, _, rack50, _, _ = capture_deck()
    plan = mca.plan_pick_up(self.geo, rack50, Coordinate.zero(), TRAVEL, mca.PlungerState())
    self.assertMatchesCapture(plan, "get_tips50")

  def test_drop_tips200(self):
    _, racks, _, _, _ = capture_deck()
    plan = mca.plan_drop(self.geo, racks[4], Coordinate.zero(), TRAVEL, mca.PlungerState(position=50000))
    self.assertMatchesCapture(plan, "drop_tips200")

  def test_drop_tips50(self):
    _, _, rack50, _, _ = capture_deck()
    plan = mca.plan_drop(self.geo, rack50, Coordinate.zero(), TRAVEL, mca.PlungerState(position=50000))
    self.assertMatchesCapture(plan, "drop_tips50")

  def _aspirate(self, volume, tip):
    _, _, _, dw, _ = capture_deck()
    plunger = mca.PlungerState(position=mca.PLUNGER_READY)
    target = mca.LiquidTarget(self.geo, dw, Coordinate.zero(), tip)
    plan = mca.plan_aspirate(target, volume, liquid_class(tip, volume), TRAVEL, plunger)
    return target, plunger, plan

  def test_aspirate_captures(self):
    for name, volume, tip in [
      ("aspirate_1ul_tips200", 1.0, TIP200),
      ("aspirate_100ul_tips200", 100.0, TIP200),
      ("aspirate_175ul_tips200", 175.0, TIP200),
      ("aspirate_1ul_tips50", 1.0, TIP50),
      ("aspirate_50ul_tips50", 50.0, TIP50),
    ]:
      with self.subTest(name):
        _, _, plan = self._aspirate(volume, tip)
        self.assertMatchesCapture(plan, name)

  def test_plunger_checks_match_capture_replies(self):
    _, _, plan = self._aspirate(100.0, TIP200)
    pairs = json.loads((CAPTURES / "aspirate_100ul_tips200.json").read_text())["pairs"]
    self.assertEqual(list(plan.checks.values()), [int(b) for c, _, b in pairs if c == "T40RDP4"])

  def test_dispense_captures(self):
    for name, volume, tip in [
      ("dispense_1ul_tips200", 1.0, TIP200),
      ("dispense_100ul_tips200", 100.0, TIP200),
      ("dispense_1ul_tips50", 1.0, TIP50),
      ("dispense_50ul_tips50", 50.0, TIP50),
    ]:
      with self.subTest(name):
        target, plunger, _ = self._aspirate(volume, tip)
        here = (5577, 2340, target.z("retract"))
        plan = mca.plan_dispense(target, volume, here, plunger)
        self.assertMatchesCapture(plan, name, start=here)
        self.assertEqual(plunger.position, mca.PLUNGER_READY)

  def test_tip_cleaning_replay(self):
    """Whole EVOware tip-washing run: 4 boxes x (pick up, return) then
    4 boxes x (pick up, mix 5 x 175 uL in 3 troughs, return)."""
    _, racks, _, _, troughs = capture_deck(cleaning=True)
    plunger = mca.PlungerState()
    lc = liquid_class(TIP200, 175.0)
    commands, here = [], TRAVEL

    def run(plan):
      nonlocal here
      commands.extend(wire(plan))
      here = mca.track(here, plan)

    for site in (1, 2, 3, 4):
      run(mca.plan_pick_up(self.geo, racks[site], Coordinate.zero(), here, plunger))
      run(mca.plan_drop(self.geo, racks[site], Coordinate.zero(), here, plunger))
    for site in (1, 2, 3, 4):
      run(mca.plan_pick_up(self.geo, racks[site], Coordinate.zero(), here, plunger))
      for grid in (9, 15, 21):
        target = mca.LiquidTarget(self.geo, troughs[grid], Coordinate.zero(), TIP200)
        run(mca.plan_mix(target, 175.0, 5, lc, here, plunger))
      run(mca.plan_drop(self.geo, racks[site], Coordinate.zero(), here, plunger))

    def essential(cmds):
      """Motion/plunger commands, without XY moves to the current XY."""
      out, xy = [], None
      for c in cmds:
        c = c.rstrip(",")
        params = c[3:].split(",")
        if c.startswith("PAA") and len(params) == 2 and all(params):
          if params == xy:
            continue
          xy = params
        if c.startswith(("PAA", "PPR", "PPA", "ATF", "ATD", "T40RDP4")):
          out.append(c)
      return out

    ours, theirs = essential(commands), essential(capture("tip_cleaning"))
    diff = "\n".join(list(difflib.unified_diff(theirs, ours, "evoware", "plr", lineterm=""))[:80])
    self.assertEqual(ours, theirs, diff)


class LiquidClassTests(unittest.TestCase):
  def test_xml_values_in_firmware_units(self):
    lc = liquid_class(TIP200, 100)
    self.assertEqual((lc.single.lead_air_gap, lc.single.lead_speed), (4000, 10000))
    self.assertEqual((lc.single.trail_air_gap, lc.single.trail_speed), (2000, 1000))
    self.assertEqual((lc.single.aspirate_speed, lc.single.dispense_speed), (10000, 80000))
    self.assertEqual(lc.single.aspirate_steps(100), 21080)
    self.assertEqual((lc.aspirate_position, lc.aspirate_retract_position, lc.aspirate_retract_z_speed),
                     ("z_max", "z_start", 50))
    self.assertEqual((lc.dispense_position, lc.dispense_retract_position), ("z_dispense", "z_dispense"))
    self.assertEqual((lc.multi.lead_air_gap, lc.multi.trail_air_gap, lc.multi.excess_ul), (2000, 1000, 10.0))
    self.assertTrue(lc.multi.dispense_tag)
    wet = liquid_class(TIP50, 50)
    self.assertTrue(wet.dispense_contact)
    self.assertEqual((wet.dispense_retract_position, wet.dispense_retract_z_speed), ("z_start", 400))

  def test_subclass_ranges(self):
    self.assertEqual(liquid_class(TIP200, 1).single.aspirate_steps(1), 290)  # offset 0.45, factor 1
    self.assertEqual(liquid_class(TIP200, 20).volume_range, (7.51, 20.01))
    self.assertEqual(liquid_class(TIP200, 20.01).volume_range, (20.01, 200.01))

  def test_unknown_dispense_ramp_refused(self):
    from pylabrobot.legacy.liquid_handling.backends.tecan.evoware_liquid_classes import dispense_slp

    self.assertEqual(dispense_slp(80000), (1200000, 80000, 3200000))
    with self.assertRaises(NotImplementedError):
      dispense_slp(60000)


class PipettingTests(_TipDiameter, unittest.TestCase):
  def setUp(self):
    super().setUp()
    _, _, _, _, self.troughs = capture_deck(cleaning=True)
    self.src = mca.LiquidTarget(self.geo, self.troughs[9], Coordinate.zero(), TIP200)
    self.dst = mca.LiquidTarget(self.geo, self.troughs[15], Coordinate.zero(), TIP200)
    self.plunger = mca.PlungerState(position=mca.PLUNGER_READY)

  def pprs(self, plan):
    return [p[0] for c, p in plan.commands if c == "PPR"]

  def test_two_aspirations_then_full_dispense(self):
    lc = liquid_class(TIP200, 50)
    a1 = mca.plan_aspirate(self.src, 50, lc, TRAVEL, self.plunger)
    a2 = mca.plan_aspirate(self.dst, 50, lc, (0, 0, 2370), self.plunger)
    self.assertEqual(self.pprs(a1), [-4000, -10580, -2000])
    self.assertEqual(self.pprs(a2), [-10580, -2000])  # no second leading air gap
    self.assertEqual(self.plunger.volume_ul, 100)
    d = mca.plan_dispense(self.dst, 100, (0, 0, 2370), self.plunger)
    self.assertEqual(self.pprs(d), [4000 + 10580 + 2000 + 10580 + 2000])
    self.assertTrue(self.plunger.is_ready)

  def test_multi_dispense(self):
    lc = liquid_class(TIP200, 100)
    a = mca.plan_aspirate(self.src, 100, lc, TRAVEL, self.plunger, multi=True)
    # multi: LAG 10 uL, liquid 100 uL (factor 1) + excess 10 uL, TAG 5 uL
    self.assertEqual(self.pprs(a), [-2000, -22000, -1000])
    parts = [mca.plan_dispense(self.dst, 25, (0, 0, 2370), self.plunger) for _ in range(4)]
    self.assertEqual(self.pprs(parts[0]), [1000 + 5000, -1000])  # TAG out + 25 uL, new TAG
    self.assertEqual(self.pprs(parts[3]), [1000 + 5000])  # last portion, no new TAG
    self.assertEqual(self.plunger.volume_ul, 0)
    self.assertEqual(self.plunger.liquid, 2000)  # the excess
    with self.assertRaises(RuntimeError):
      mca.plan_drop(self.geo, capture_deck()[1][4], Coordinate.zero(), TRAVEL, self.plunger)
    e = mca.plan_empty(self.dst, (0, 0, 2370), self.plunger)
    self.assertEqual(self.pprs(e), [2000 + 2000])  # excess + leading air gap
    self.assertTrue(self.plunger.is_ready)

  def test_single_mode_partial_dispense(self):
    lc = liquid_class(TIP200, 100)
    mca.plan_aspirate(self.src, 100, lc, TRAVEL, self.plunger)
    d1 = mca.plan_dispense(self.dst, 40, (0, 0, 2370), self.plunger)
    self.assertEqual(self.pprs(d1), [2000 + 8400])  # TAG + 200 * 1.05 * 40
    d2 = mca.plan_dispense(self.dst, 60, (0, 0, 2370), self.plunger)
    # the rest in single mode empties the tips completely (like EVOware): + leading air gap
    self.assertEqual(self.pprs(d2), [21080 - 8400 + 4000])
    self.assertTrue(self.plunger.is_ready)

  def test_mix_before_aspiration(self):
    lc = liquid_class(TIP200, 100)
    a = mca.plan_aspirate(self.src, 100, lc, TRAVEL, self.plunger, mix=mca.MixSpec(150, 2))
    self.assertEqual(self.pprs(a), [-4000, -30000, 30000, -30000, 30000, -21080, -2000])

  def test_mix_after_dispense(self):
    lc = liquid_class(TIP200, 100)
    mca.plan_aspirate(self.src, 100, lc, TRAVEL, self.plunger)
    d = mca.plan_dispense(self.dst, 100, (0, 0, 2370), self.plunger, mix=mca.MixSpec(100, 2))
    self.assertEqual(self.pprs(d), [27080, -4000, -20000, 20000, -20000, 20000])
    self.assertEqual(d.commands[-1], ("PAA", [None, None, mca.Z_TRAVEL]))

  def test_delays_from_liquid_class(self):
    lc = liquid_class(TIP200, 100)
    a = mca.plan_aspirate(self.src, 100, lc, TRAVEL, self.plunger)
    self.assertIn(("WAIT", [500]), a.commands)

  def test_mixed_modes_refused(self):
    lc = liquid_class(TIP200, 50)
    mca.plan_aspirate(self.src, 50, lc, TRAVEL, self.plunger)
    with self.assertRaises(ValueError):
      mca.plan_aspirate(self.src, 50, lc, TRAVEL, self.plunger, multi=True)

  def test_capacity(self):
    lc = liquid_class(TIP200, 150)
    mca.plan_aspirate(self.src, 150, lc, TRAVEL, self.plunger)
    with self.assertRaises(ValueError):
      mca.plan_aspirate(self.src, 60, liquid_class(TIP200, 60), TRAVEL, self.plunger)


class GripperTests(_TipDiameter, unittest.TestCase):
  def test_grip_and_release_plans(self):
    _, _, _, dw, troughs = capture_deck()
    g = mca.plan_grip(self.geo, dw, TRAVEL, z_grip=1200, grip_width=780, open_width=900)
    self.assertEqual(wire(g), ["SSM1", "PAA5577,2340", "SSM0", "PAA,,,900", "PAA,,1200", "AGR780,100,75", "PAA,,2370"])
    r = mca.plan_release(self.geo, troughs[9], (5577, 2340, 2370), z_release=1200, open_width=900)
    self.assertEqual(wire(r)[-3:], ["PAA,,1200", "PAA,,,900", "PAA,,2370"])
    with self.assertRaises(ValueError):
      mca.plan_grip(self.geo, dw, TRAVEL, z_grip=1200, grip_width=900, open_width=900)


class InitTests(unittest.TestCase):
  def test_head_drive_init_matches_evoware_full_initialization(self):
    # "Tecan EVO full initialization no carousel.pcapng", W1 after PIA/PIA1/PRA/RHP reads
    evoware = ["PAA,,,1284", "PID", "ASH0", "ASH0", "SLP1152000,80000,3200000", "PPA50000"]
    plunger = mca.PlungerState()
    self.assertEqual(wire(mca.plan_initialize_head_drive(plunger)), evoware)
    self.assertTrue(plunger.is_ready)


class StateTests(_TipDiameter, unittest.TestCase):
  def test_aspirate_requires_ready_plunger(self):
    _, _, _, dw, _ = capture_deck()
    target = mca.LiquidTarget(self.geo, dw, Coordinate.zero(), TIP200)
    with self.assertRaises(RuntimeError):
      mca.plan_aspirate(target, 100, liquid_class(TIP200, 100), TRAVEL, mca.PlungerState())

  def test_dispensing_more_than_held_refused(self):
    _, _, _, dw, _ = capture_deck()
    target = mca.LiquidTarget(self.geo, dw, Coordinate.zero(), TIP200)
    lc = liquid_class(TIP200, 100)
    plunger = mca.PlungerState(position=mca.PLUNGER_READY)
    mca.plan_aspirate(target, 100, lc, TRAVEL, plunger)
    with self.assertRaises(ValueError):
      mca.plan_dispense(target, 150, TRAVEL, plunger)

  def test_volume_outside_classes_refused(self):
    with self.assertRaises(ValueError):
      liquid_class(TIP200, 250)

  def test_flow_rate_override(self):
    self.assertEqual(mca.with_flow_rate((1, 10000, 3), 25), (1, 5000, 3))
    with self.assertRaises(ValueError):
      mca.with_flow_rate((1, 10000, 3), 0)

  def test_xy_move_raises_z_first_when_low(self):
    plan = mca.Plan()
    mca.plan_move_xy(plan, 100, 200, (0, 0, 1500))
    self.assertEqual(wire(plan), ["PAA,,2370", "SSM1", "PAA100,200", "SSM0"])
    plan = mca.Plan()
    mca.plan_move_xy(plan, 100, 200, (100, 200, 1500))
    self.assertEqual(plan.commands, [])


if __name__ == "__main__":
  unittest.main()
