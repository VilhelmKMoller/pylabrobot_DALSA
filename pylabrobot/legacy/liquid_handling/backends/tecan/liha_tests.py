"""LiHa command plans vs. EVOware run logs (golden tests, offline).

Geometry comes from the instrument's EVOware configuration (``test_data/evoware``);
expected commands come from ``test_data/liha_logs`` (C5 traffic extracted from
``AuditTrail/log/EVO_*.LOG``).
"""

import json
from pathlib import Path
import unittest

from pylabrobot.legacy.liquid_handling.backends.tecan import evoware_config, liha

DATA = Path(__file__).parent / "test_data"


def geometry():
  return liha.LihaGeometry(evoware_config.load(DATA / "evoware"), z_range=2100)


class LihaMachineTests(unittest.TestCase):
  def test_from_evoware_inf(self):
    m = geometry().machine
    self.assertEqual((m.tip_count, m.tip_offset, m.dilutor_volume, m.diluter_steps, m.wash_max_speed),
                     (8, -500, 1000, 3000, 900))
    self.assertEqual(m.steps_per_ul, 3)
    self.assertEqual(liha.plunger_speed(m, 900), 5400)
    self.assertEqual(liha.plunger_speed(m, 70), 420)


class LihaGeometryTests(unittest.TestCase):
  def test_wash_waste_position(self):
    geo = geometry()
    self.assertEqual(geo.xy(liha.WASH_WASTE), liha.Position(-18, 748, 90))
    self.assertEqual(geo.z(liha.WASH_WASTE, "z_travel"), 690)

  def test_bad_site(self):
    geo = geometry()
    with self.assertRaises(ValueError):
      geo.xy(liha.LihaSite(1, 8, liha.WASH_WASTE.carrier, liha.WASH_WASTE.labware))
    with self.assertRaises(KeyError):
      geo.xy(liha.LihaSite(1, 1, "no such carrier", liha.WASH_WASTE.labware))


class LihaWashTests(unittest.TestCase):
  def test_matches_evoware_flush(self):
    fixture = json.loads((DATA / "liha_logs" / "wash_12ml_8ml_grid1.json").read_text())
    expected = liha.normalize_log([cmd for cmd, *_ in fixture["pairs"]])
    got = liha.plan_strings(liha.plan_wash(geometry()))
    self.assertEqual(got, expected)

  def test_delay_is_a_host_wait_before_the_air_gap(self):
    plan = liha.plan_wash(geometry(), waste_delay_ms=50)
    names = [c for c, _ in plan.commands]
    self.assertEqual(plan.commands[names.index("WAIT")], ("WAIT", [50]))
    self.assertEqual(names[names.index("WAIT") + 1 :], ["SHZ", "SEP", "PPR", "RTS"])

  def test_volume_must_be_whole_strokes(self):
    with self.assertRaises(ValueError):
      liha.plan_wash(geometry(), waste_volume_ml=12.5)
    with self.assertRaises(ValueError):
      liha.plan_wash(geometry(), waste_volume_ml=0)
    plan = liha.plan_wash(geometry(), waste_volume_ml=3)
    self.assertEqual(sum(1 for c, p in plan.commands if c == "PPR" and p[0] == 3000), 3)

  def test_channel_subset(self):
    plan = liha.plan_wash(geometry(), waste_volume_ml=1, channels=[0, 7])
    ppr = next(p for c, p in plan.commands if c == "PPR")
    self.assertEqual(ppr, [3000, None, None, None, None, None, None, 3000])
    self.assertEqual(liha.format_command("PPR", ppr), "PPR3000,,,,,,,3000")
    with self.assertRaises(ValueError):
      liha.plan_wash(geometry(), channels=[8])


if __name__ == "__main__":
  unittest.main()
