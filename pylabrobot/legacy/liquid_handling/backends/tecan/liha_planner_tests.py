"""Replay complete EVOware runs through the LiHa planners and compare every LiHa command.

Each step of a logged run is planned with liha.py from the configuration in force, feeding in only
what EVOware itself read from the instrument during the step (liquid heights after MDT/MET, plunger
positions RPP0). Our plunger bookkeeping is checked against every RPP0 EVOware read. Outcomes:

  same            identical to EVOware after normalize_log
  plate model     identical except MDT/MET Zadd, STZ or SSZ by at most 1 in a plate (EVOware's
                  well-shape model is not reproduced; 0.1 mm / 0.1 mm/s)
  drop air gap    a chute drop identical except the speed of the final air gap (rule unknown)
  expected        a listed step that differs for a stated reason
  refused / not planned / instrument error   steps outside what the planners do
"""

import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from pylabrobot.legacy.liquid_handling.backends.tecan import evoware_config, liha
from pylabrobot.legacy.liquid_handling.backends.tecan.evoware_liquid_classes import (
  LiquidClassLibrary,
)
from pylabrobot.legacy.liquid_handling.backends.tecan.liha import (
  LihaGeometry,
  LihaSite,
  normalize_log,
  plan_strings,
)

HERE = Path(__file__).parent / "test_data"
REPORTS = ("RPX", "RPY", "RPZ", "RPP")
PLATE_MAX_AREA = 100.0  # mm2; wells of 96-well plates are ~33 mm2, troughs thousands

# The run starts after an earlier session dropped its DiTis, which leaves the 10 uL DropDITI air gap
# (30 steps) in every adapter. The first Dispense's RPP0 (573 = 30 + 45 + 468 + 30) confirms it.
INITIAL_PLUNGER = 30

END_OF_RUN = "last steps of the run: EVOware raises Z (PAZ2100) or re-reads RTS as the program ends"
CUSTOMIZED = "liquid class customized in the script command ('>> name <<'); not in the class files"
TROUGH_300 = (
  "300 ul from the 300 ml trough: EVOware MDT Zadd/STZ 3, area rule gives 1 (156 ul gives 1 in"
  " both; 779 ul gave 1 in 2025); rule unknown, 0.2 mm"
)

EXPECTED: Dict[str, Dict[int, str]] = {
  "EVO_20260409_115221.json": {
    25: TROUGH_300,
    33: CUSTOMIZED,
    37: CUSTOMIZED,
    41: CUSTOMIZED,
    44: END_OF_RUN,
    45: END_OF_RUN,
  },
  "EVO_20260409_111830.json": {
    25: TROUGH_300,
    75: END_OF_RUN,
  },
}


def run_setup(run) -> Tuple[LihaGeometry, LiquidClassLibrary]:
  config = evoware_config.parse_carrier_cfg("\n".join(run["config_records"]))
  inf = (HERE / "evoware" / "evoware.inf").read_text(encoding="latin-1")
  evoware_config.parse_evoware_inf(inf, config)
  geo = LihaGeometry(config, run["z_range"])
  assert geo.machine.tip_offset == run["tip_offset"]
  classes = {
    name: (ET.fromstring(text), run["log"]) for name, text in run["liquid_classes"].items()
  }
  return geo, LiquidClassLibrary(classes)


def reply_after(commands, code: str, after: str) -> List[int]:
  """Reply data of the first ``code`` after the first command starting with ``after``."""
  seen = False
  for c, _, data in commands:
    if c.startswith(after):
      seen = True
    elif seen and c == code:
      return [int(v) for v in data.split(",")]
  raise AssertionError(f"no {code} after {after}")


def first_reply(commands, code: str) -> List[int]:
  return next([int(v) for v in d.split(",")] for c, _, d in commands if c == code)


def mask_tips(mask: int) -> List[int]:
  return [t for t in range(1, 9) if mask & (1 << (t - 1))]


def _values(command: str) -> List[Optional[int]]:
  return [None if v == "" else int(v) for v in command[3:].split(",")]


def within_one(a: str, b: str, skip: int = 0) -> bool:
  """Same command, per-tip values differing by at most 1 (after ``skip`` leading parameters)."""
  if a[:3] != b[:3]:
    return False
  va, vb = _values(a), _values(b)
  if len(va) != len(vb) or va[:skip] != vb[:skip]:
    return False
  return all(
    x == y or (x is not None and y is not None and abs(x - y) <= 1) for x, y in zip(va, vb)
  )


def classify(ours: List[str], logged: List[str], plate: bool, drop: bool) -> str:
  if ours == logged:
    return "same"
  if len(ours) != len(logged):
    return "different"
  diffs = [(i, a, b) for i, (a, b) in enumerate(zip(ours, logged)) if a != b]
  if plate and all(
    (a[:3] in ("MDT", "MET") and within_one(a, b, skip=4))
    or (a[:3] in ("STZ", "SSZ") and within_one(a, b))
    for _, a, b in diffs
  ):
    return "plate model"
  if drop and len(diffs) == 1 and diffs[0][1][:3] == "SEP" and diffs[0][0] == len(ours) - 2:
    return "drop air gap"
  return "different"


class Replay:
  def __init__(self, run):
    self.run = run
    self.geo, self.lib = run_setup(run)
    self.n = self.geo.machine.tip_count
    self.plunger: List[Optional[int]] = [INITIAL_PLUNGER] * self.n
    self.diti: Optional[str] = None
    self.results: List[Tuple[int, str, str, object]] = []  # (index, step, outcome, detail)

  def site(self, note) -> LihaSite:
    carrier_id = self.run["layout"][str(note["grid"])]
    carrier = self.geo.config.carriers_by_id[carrier_id].name
    return LihaSite(note["grid"], note["site"] - 1, carrier, note["labware"])

  def step(self, index: int, step) -> None:
    try:
      self._step(index, step)
    except NotImplementedError as exc:
      self.results.append((index, step["step"], "refused", str(exc)))
      self._unknown_after(step)

  def _unknown_after(self, step) -> None:
    """State after a step EVOware did but we did not plan."""
    for note in step["tips"]:
      if note["action"] == "fetch DiTis":
        self.diti = note["labware"]
      elif note["action"] in ("aspirate", "dispense", "mix"):
        self.plunger[note["tip"] - 1] = None

  def _step(self, index: int, step) -> None:
    name = step["step"]
    cmds = step["commands"]
    errors = [(c, s) for c, s, _ in cmds if s not in (0, -1)]
    if errors:  # the instrument reported an error; EVOware's recovery is not planned here
      self.results.append((index, name, "instrument error", errors))
      self._unknown_after(step)
      return
    logged = normalize_log([c for c, _, _ in cmds if not c.startswith(REPORTS)])
    notes = step["tips"]
    plate = drop = False
    if name.startswith(("Drop DiTis", "DiTi already mounted")):
      ast = next(c for c, _, _ in cmds if c.startswith("AST"))
      tips = mask_tips(int(ast[3:].split(",")[0]))
      plan = plan_strings(liha.plan_drop_ditis(self.geo, tips))
      for t in tips:
        self.plunger[t - 1] = round(liha.DROP_AIR_GAP_UL * self.geo.machine.steps_per_ul)
      self.diti = None
      drop = True
    elif name.startswith("Get DiTis"):
      first = notes[0]
      tips = [t["tip"] for t in notes]
      slow = any(c.startswith("SFX4999") for c, _, _ in cmds)
      site = self.site(first)
      plan = plan_strings(
        liha.plan_get_ditis(self.geo, site, first["column"], first["row"], tips, slow)
      )
      self.diti = first["labware"]
    elif name.startswith("Aspirate"):
      first, lcn = notes[0], step["liquid_class"]
      tips = [t["tip"] for t in notes]
      lc = self.lib.liha(lcn["name"], lcn["volume"])
      before = list(self.plunger)
      if any(before[t - 1] is None for t in tips):
        self.results.append((index, name, "not planned", "plunger unknown after an unplanned step"))
        return
      site = self.site(first)
      assert self.diti is not None, "aspirate without DiTis"
      asp = liha.plan_aspirate(
        self.geo,
        lc,
        site,
        first["column"],
        first["row"],
        tips,
        lcn["volume"],
        self.diti,
        [p if p is not None else 0 for p in before],
      )
      detected = reply_after(cmds, "RPZ0", "MDT")
      plan = plan_strings(asp.search) + plan_strings(asp.finish(detected))
      for t in tips:
        b = before[t - 1]
        assert b is not None
        self.plunger[t - 1] = liha.aspirated_steps(lc, self.geo, lcn["volume"], b)
      plate = self.geo.config.labware_def(first["labware"]).area <= PLATE_MAX_AREA
    elif name.startswith("Dispense"):
      first, lcn = notes[0], step["liquid_class"]
      tips = [t["tip"] for t in notes]
      lc = self.lib.liha(lcn["name"], lcn["volume"])
      read = first_reply(cmds, "RPP0")
      tracked = [self.plunger[t - 1] for t in tips]
      if all(p is not None for p in tracked):
        self.results.append((index, name, "plunger", (tracked, [read[t - 1] for t in tips])))
      site = self.site(first)
      assert self.diti is not None, "dispense without DiTis"
      disp = liha.plan_dispense(
        self.geo, lc, site, first["column"], first["row"], tips, self.diti, read
      )
      plan = plan_strings(disp.first)
      if disp.finish_args is not None:
        plan += plan_strings(disp.finish(reply_after(cmds, "RPZ0", "MET")))
      steps = liha.dispense_steps(lc, self.geo, read, tips)
      for t in tips:
        self.plunger[t - 1] = read[t - 1] - steps[t - 1]
      plate = self.geo.config.labware_def(first["labware"]).area <= PLATE_MAX_AREA
    else:
      self.results.append((index, name, "not planned", name))
      self._unknown_after(step)
      return
    ours = normalize_log(plan)
    self.results.append((index, name, classify(ours, logged, plate, drop), (ours, logged)))

  def replay(self):
    for i, step in enumerate(self.run["steps"]):
      self.step(i, step)
    return self.results


def load(name: str):
  return json.loads((HERE / "liha_logs" / "runs" / name).read_text())


def first_difference(ours: List[str], logged: List[str]) -> str:
  for i, (a, b) in enumerate(zip(ours, logged)):
    if a != b:
      return f"#{i}: ours {a!r} vs EVOware {b!r}"
  return f"length {len(ours)} vs {len(logged)}"


class ReplayTests(unittest.TestCase):
  def replay(self, name: str) -> Dict[str, int]:
    run = load(name)
    results = Replay(run).replay()
    expected = EXPECTED[name]
    unexplained = {}
    for i, step, outcome, detail in results:
      if outcome == "different" and i not in expected:
        unexplained[(i, step)] = first_difference(*detail)  # type: ignore[misc]
    stale = sorted(
      i for i in expected if not any(r[0] == i and r[2] == "different" for r in results)
    )
    plunger = [(i, d) for i, _, outcome, d in results if outcome == "plunger" and d[0] != d[1]]
    self.assertEqual(unexplained, {})
    self.assertEqual(stale, [], "listed steps that no longer differ")
    self.assertEqual(plunger, [], "plunger bookkeeping vs EVOware's RPP0")
    counts: Dict[str, int] = {}
    for _, _, outcome, _ in results:
      counts[outcome] = counts.get(outcome, 0) + 1
    return counts

  def test_free_dispense_run(self):
    """REFINES_fortynding 2026-04-09 11:52: 1000 ul DiTis, 150/300 ul 'Water free dispense' from the
    300 ml trough into a TecanExtractionPlate, 'Water wet contact' dispenses, drops into the chute."""
    counts = self.replay("EVO_20260409_115221.json")
    self.assertGreaterEqual(counts.get("same", 0), 34)
    self.assertEqual(counts.get("plunger"), 16)  # every Dispense's RPP0 matched our bookkeeping

  def test_wet_contact_run(self):
    """REFINES_fortynding 2026-04-09 11:18: as above plus Mix steps and instrument errors."""
    counts = self.replay("EVO_20260409_111830.json")
    self.assertGreaterEqual(counts.get("same", 0), 25)
    self.assertGreater(counts.get("plunger", 0), 10)

  def test_planners_for_the_2026_11_24_deck(self):
    """The deck for the visual validation: plain 1000 ul DiTis at grid 46 site 1 (MCA carrier),
    300 ml trough at grid 15 site 3, chute at grid 1."""
    config = evoware_config.load(HERE / "evoware")
    geo = LihaGeometry(config, 2100)
    lib = LiquidClassLibrary.from_database(HERE / "evoware")
    rack = LihaSite(46, 0, "MCA96 Carrier 4 pos", "DiTi 1000ul LiHa")
    trough = LihaSite(15, 2, "MP 3Pos", "Trough 300ml MCA96")
    tips = list(range(1, 9))
    z8 = ",".join(["2100"] * 8)
    self.assertEqual(
      plan_strings(liha.plan_get_ditis(geo, rack, 1, 1, tips)),
      [
        "SSM1",
        "SFX4999,550",
        "SHZ" + z8,
        "SSM1",
        "PAA11347,-348,90," + z8,
        "SSM0",
        "SFX10000,1600",
        "AGT255,783,210,0",
        "RTS",
      ],
    )
    lc = lib.liha("Water free dispense", 150)
    asp = liha.plan_aspirate(geo, lc, trough, 1, 1, tips, 150, "DiTi 1000ul LiHa", [30] * 8)
    search = plan_strings(asp.search)
    self.assertIn("PAA3563,2022,96," + ",".join(["1411"] * 8), search)
    self.assertIn("PPR" + ",".join(["45"] * 8), search)  # (20 - 10) + 5 uL
    self.assertIn("STL" + ",".join(["1260"] * 8), search)
    self.assertIn("MDT255,,,," + ",".join(["1"] * 8), search)
    finish = plan_strings(asp.finish([1034] * 8))
    self.assertIn("MTR" + ",".join(["468"] * 8), finish)
    self.assertIn("MAZ" + ",".join(["1104"] * 8), finish)
    drop = plan_strings(liha.plan_drop_ditis(geo, tips))
    self.assertIn("PAA126,2810,90," + ",".join(["1075"] * 8), drop)
    self.assertIn("AST255,0", drop)

  def test_refusals(self):
    config = evoware_config.load(HERE / "evoware")
    geo = LihaGeometry(config, 2100)
    rack = LihaSite(46, 0, "MCA96 Carrier 4 pos", "DiTi 1000ul LiHa")
    with self.assertRaises(NotImplementedError):
      liha.plan_get_ditis(geo, rack, 1, 1, [1, 3])
    with self.assertRaises(ValueError):
      liha.plan_get_ditis(geo, rack, 1, 1, [0])
    lc = LiquidClassLibrary.from_database(HERE / "evoware").liha("Water free dispense", 150)
    with self.assertRaisesRegex(ValueError, "nothing to dispense"):
      liha.dispense_steps(lc, geo, [60] * 8, [1])


if __name__ == "__main__":
  unittest.main()
