"""LiHa positions against the EVOware run logs and the 2026-11-24 deck."""

import json
import unittest
from pathlib import Path
from typing import Dict, List, Tuple

from pylabrobot.legacy.liquid_handling.backends.tecan import evoware_config, liha
from pylabrobot.legacy.liquid_handling.backends.tecan.liha import LihaGeometry, LihaSite

HERE = Path(__file__).parent / "test_data"
LIHA_MACHINE = "[LihaMachine]\nTipCount=8\nTipOffset={}\nDilutorVolume=1000\nDiluterSteps=3000\nWashMaxSpeed=600\n"

# Logs whose steps differ from the computed positions, with the reason and the checks it affects.
# The test fails on a mismatch in any other log or check, and on a listed log that no longer differs.
NOT_THE_RACK = "the last PAA before the action is at the wash/park position, not the labware"
PLATE_1814 = "plate z_start 1814 in use; snapshots by timestamp say 1821 (z +7)"
EXPECTED_MISMATCHES: Dict[str, Tuple[str, Tuple[str, ...]]] = {
  "EVO_20240919_140227.LOG": (
    "the PAA before the dispense is at the wash station (X -15)",
    ("x", "y"),
  ),
  "EVO_20240920_104306.LOG": (
    "Carrier.cfg saved 3 min before the run (10:40); AGT 700 vs 710",
    ("agt",),
  ),
  "EVO_20240920_105654.LOG": ("Carrier.cfg saved at 10:40; AGT 700 vs 710", ("agt",)),
  "EVO_20251028_120510.LOG": (NOT_THE_RACK, ("x", "y")),
  "EVO_20251105_142711.LOG": (NOT_THE_RACK, ("x", "y")),
  "EVO_20251106_114000.LOG": (NOT_THE_RACK, ("x", "y")),
  "EVO_20260409_122520.LOG": (NOT_THE_RACK, ("x", "y")),
  "EVO_20260408_140631.LOG": (
    "trough definition being edited (saved as 1/1/1, then 8/1/1, within minutes); only tip 1 moved",
    ("x", "stl", "sml"),
  ),
  "EVO_20260408_141156.LOG": ("trough definition being edited", ("x",)),
  "EVO_20260408_141842.LOG": ("trough definition being edited", ("x",)),
  # These runs used z_start 1814 for 96 Well TecanExtractionPlate, a value that appears in the
  # AuditTrail Carrier.cfg snapshots only from 2026-06-16 on; the snapshots in force by timestamp
  # (2026-04-08/09) have 1821. Snapshot times do not show when a definition took effect.
  "EVO_20260409_111830.LOG": (PLATE_1814, ("stl", "sml")),
  "EVO_20260409_115221.LOG": (PLATE_1814, ("stl", "sml")),
  "EVO_20260409_120340.LOG": (PLATE_1814, ("stl", "sml")),
}


def case_geometry(case) -> Tuple[LihaGeometry, LihaSite]:
  text = "\n".join(case["config_records"])
  config = evoware_config.parse_carrier_cfg(text)
  evoware_config.parse_evoware_inf(LIHA_MACHINE.format(case["tip_offset"]), config)
  carrier = config.carriers_by_id[case["carrier_id"]].name
  site = LihaSite(case["grid"], case["site"] - 1, carrier, case["labware"])
  return LihaGeometry(config, case["z_range"] or 2100), site


def check_case(case) -> List[Tuple[str, object, object]]:
  """(check, computed, logged) for every check that applies to this case."""
  geo, site = case_geometry(case)
  if case["labware"] in liha.UNSUPPORTED_LABWARE:
    return []
  out: List[Tuple[str, object, object]] = []
  pos = geo.tips(site, case["column"], case["first_row"], case["first_tip"])
  x, y, ys = case["paa"][:3]
  out += [("x", pos.x, x), ("y", pos.y, y)]
  if case["n_tips"] > 1:  # a single tip keeps whatever spacing was set before
    out.append(("ys", pos.y_spacing, ys))
  if case["verb"] == "fetch DiTis" and "agt" in case and case["diti"] is None:
    fetch = geo.diti_fetch(site, case["column"], case["first_row"], case["first_tip"])
    out.append(("agt", (fetch.z_start, fetch.search_dist), tuple(case["agt"][1:3])))
  if case["verb"] in ("aspirate", "mix", "dispense") and case["diti"] is not None:
    tips = range(case["first_tip"] - 1, case["first_tip"] - 1 + case["n_tips"])
    for code, height in (("stl", "z_start"), ("sml", "z_max")):
      if code in case:
        want = geo.z(site, height, case["diti"])
        out.append((code, want, tuple(case[code][t] for t in tips if t < len(case[code]))))
  return out


def matches(check, computed, logged) -> bool:
  if check in ("stl", "sml"):
    return all(v == computed for v in logged)
  return bool(computed == logged)


class GoldenEvowareTests(unittest.TestCase):
  def test_positions_match_evoware_logs(self):
    cases = json.loads((HERE / "liha_logs/geometry_cases.json").read_text())["cases"]
    self.assertGreater(len(cases), 450)
    counts = {"x": 0, "y": 0, "ys": 0, "agt": 0, "stl": 0, "sml": 0}
    agree = dict(counts)
    unexpected, explained = {}, set()
    for case in cases:
      for check, computed, logged in check_case(case):
        counts[check] += 1
        if matches(check, computed, logged):
          agree[check] += 1
          continue
        reason = EXPECTED_MISMATCHES.get(case["log"])
        if reason and check in reason[1]:
          explained.add(case["log"])
        else:
          key = (case["log"], case["verb"], case["labware"], case["column"], check)
          unexpected[key] = (computed, logged)
    self.assertEqual(unexpected, {}, "positions differing from EVOware")
    self.assertEqual(sorted(set(EXPECTED_MISMATCHES) - explained), [], "listed logs that now match")
    # Coverage, and agreement over all cases including the explained ones.
    self.assertGreater(counts["x"], 450)
    self.assertGreater(counts["agt"], 150)
    self.assertGreater(counts["stl"], 80)
    for check in ("x", "y", "ys", "agt"):
      self.assertGreater(agree[check] / counts[check], 0.97, check)

  def test_unsupported_trough_raises(self):
    cases = json.loads((HERE / "liha_logs/geometry_cases.json").read_text())["cases"]
    trough = next(c for c in cases if c["labware"] == "Trough 100ml")
    geo, site = case_geometry(trough)
    with self.assertRaisesRegex(NotImplementedError, "spreads the tips"):
      geo.tips(site)

  def test_chute_drop_matches_every_logged_drop(self):
    drops = json.loads((HERE / "liha_logs/diti_waste_drops.json").read_text())["drops"]
    geo = deck_geometry()
    pos = geo.diti_waste()
    for drop in drops:
      x, y, ys, *z = [int(v) for v in drop["paa"][3:].split(",")]
      mask = int(drop["ast"][3:].split(",")[0])
      if "expected_mismatch" in drop:
        self.assertNotEqual(y, pos.y, drop["log"])
        continue
      self.assertEqual((x, y, ys), (pos.x, pos.y, pos.y_spacing), drop["log"])
      for tip, value in enumerate(z):
        dropping = bool(mask & (1 << tip))
        self.assertEqual(value, liha.DITI_WASTE_Z if dropping else geo.z_range, drop["log"])
    self.assertEqual(sum(d["count"] for d in drops if "expected_mismatch" not in d), 121)


def deck_geometry() -> LihaGeometry:
  config = evoware_config.load(HERE / "evoware")
  return LihaGeometry(config, 2100)


class Deck20261124Tests(unittest.TestCase):
  """The deck for the visual validation: REFINES worktable, plain "DiTi 1000ul LiHa" at grid 46,
  site 1 (EVOware site index 0) of the MCA96 Carrier 4 pos, DiTi waste chute at grid 1."""

  def setUp(self):
    self.geo = deck_geometry()
    self.rack = LihaSite(46, 0, "MCA96 Carrier 4 pos", "DiTi 1000ul LiHa")

  def test_tip_box_positions(self):
    fetch = self.geo.diti_fetch(self.rack)
    self.assertEqual(fetch.position, liha.Position(11347, -348, 90))
    self.assertEqual((fetch.z_start, fetch.search_dist), (783, 210))
    self.assertEqual(self.geo.tips(self.rack, column=12).x, 11347 + 990)
    self.assertEqual(self.geo.tips(self.rack, row=8, tip=8).y, -348)
    self.assertEqual(self.geo.tips(self.rack, row=2, tip=1).y, -348 + 90)

  def test_tip_box_y_is_within_the_liha_y_range(self):
    # LiHa Y limits from the EVOware logs: RPY7 -902 (displacement), RPY4 10 (init offset),
    # RPY5 2825 (range); lower limit = displacement - offset + 10 (manual 3.7.2).
    lower, upper = -902 - 10 + 10, 2825
    for row in (1, 8):
      y = self.geo.tips(self.rack, row=row, tip=1).y
      self.assertTrue(lower <= y <= upper, y)

  def test_troughs_and_chute(self):
    trough = LihaSite(15, 2, "MP 3Pos", "Trough 300ml MCA96")  # grid 15, site 3 (front)
    pos = self.geo.tips(trough, column=1)
    self.assertEqual(pos.y_spacing, 96)
    self.assertEqual(pos.x, 250 * 14 + 55 + 128 - 120)
    self.assertEqual(self.geo.tips(trough, column=12).x - pos.x, 1035)
    self.assertEqual(self.geo.diti_waste(), liha.Position(126, 2810, 90))
    self.assertEqual(self.geo.z(trough, "z_start", "DiTi 1000ul LiHa"), 2100 + 625 - 1791 + 326)

  def test_wash_waste_unchanged(self):
    self.assertEqual(self.geo.xy(liha.WASH_WASTE), liha.Position(-18, 748, 90))
    self.assertEqual(self.geo.z(liha.WASH_WASTE, "z_travel"), 690)

  def test_invalid_wells_and_tips(self):
    with self.assertRaises(ValueError):
      self.geo.tips(self.rack, column=13)
    with self.assertRaises(ValueError):
      self.geo.tips(self.rack, row=9)
    with self.assertRaises(ValueError):
      self.geo.tips(self.rack, tip=9)
    with self.assertRaises(ValueError):
      self.geo.z(self.rack, "z_bottom")
    with self.assertRaises(ValueError):
      self.geo.diti_fetch(LihaSite(15, 2, "MP 3Pos", "Trough 300ml MCA96"))


if __name__ == "__main__":
  unittest.main()
