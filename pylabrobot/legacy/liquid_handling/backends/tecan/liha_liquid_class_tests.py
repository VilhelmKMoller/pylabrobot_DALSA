"""LiHa liquid classes against EVOware run logs and the instrument's DefaultLCs.XML."""

import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from pylabrobot.legacy.liquid_handling.backends.tecan.evoware_liquid_classes import (
  LihaFirmwareValues,
  LiquidClassLibrary,
  subclass_to_liha,
)

HERE = Path(__file__).parent / "test_data"
STEPS_PER_UL = 3  # [LihaMachine] DiluterSteps 3000 / DilutorVolume 1000

Command = Tuple[str, List[Optional[int]]]


def parse(commands: List[str]) -> List[Command]:
  out = []
  for text in commands:
    code, rest = text[:3], text[3:]
    if not code.isalpha():  # T pass-through to a subdevice, e.g. T30AFI1,38,12
      continue
    out.append((code, [None if v == "" else int(v) for v in rest.split(",")] if rest else []))
  return out


def first(cmds: List[Command], code: str, start: int = 0, sign: int = 0) -> Optional[int]:
  """Index of the first ``code`` at or after ``start`` (with values of ``sign`` if given)."""
  for i in range(start, len(cmds)):
    c, v = cmds[i]
    if c == code and (sign == 0 or any(x is not None and x * sign > 0 for x in v)):
      return i
  return None


def before(cmds: List[Command], code: str, index: int) -> Optional[int]:
  for i in range(index - 1, -1, -1):
    if cmds[i][0] == code:
      return i
  return None


def tip_values(values: List[Optional[int]], tips: List[int]) -> List[Optional[int]]:
  """Values of the noted tips that this command addresses (EVOware may split a step's tips over
  several commands, leaving the others empty)."""
  return [values[t - 1] for t in tips if t - 1 < len(values) and values[t - 1] is not None]


class Missing(Exception):
  pass


def values_before(cmds: List[Command], code: str, index: Optional[int], tips: List[int]) -> set:
  i = None if index is None else before(cmds, code, index)
  if i is None:
    raise Missing(f"no {code} before the action")
  return set(tip_values(cmds[i][1], tips))


def load_cases():
  data = json.loads((HERE / "liha_logs/liquid_class_cases.json").read_text())
  return data["xml"], data["cases"]


def case_values(xml: Dict[str, str], case) -> LihaFirmwareValues:
  glob = ET.fromstring(xml[case["global"]])
  sc = ET.fromstring(xml[case["subclass"]])
  return LihaFirmwareValues(subclass_to_liha(case["class"], glob, sc), STEPS_PER_UL)


def check_aspirate(
  fw: LihaFirmwareValues, case, cmds: List[Command]
) -> List[Tuple[str, object, object]]:
  tips, nominal = case["tips"], case["nominal_ul"]
  multi = case["mode"] == "Multi"
  m = fw.lc.mode(multi)
  out: List[Tuple[str, object, object]] = []
  i_mdt = first(cmds, "MDT")
  i_mtr = first(cmds, "MTR", sign=1)
  i_maz = first(cmds, "MAZ", start=i_mtr or 0)
  if i_mdt is None or i_mtr is None or i_maz is None:
    raise Missing("no MDT / MTR / MAZ")
  # liquid detection settings, sent before MDT
  i_sdm = before(cmds, "SDM", i_mdt)
  if i_sdm is None:
    raise Missing("no SDM before MDT")
  out.append(("SDM", fw.detection_mode, tuple(cmds[i_sdm][1])))
  for code, want in (
    ("SSL", fw.search_speed),
    ("SDL", fw.retract_distance),
    ("SBL", fw.submerge()),
  ):
    out.append((code, {want}, values_before(cmds, code, i_mdt, tips)))
  # leading air gap speed (its amount depends on the preceding steps: step 6)
  i_lag = before(cmds, "PPR", i_mdt)
  if i_lag is not None:
    out.append(("LAG speed", {fw.speed(m.lead_speed)}, values_before(cmds, "SEP", i_lag, tips)))
  # aspiration: speed and calibrated volume
  out.append(
    ("aspirate speed", {fw.speed(m.aspirate_speed)}, values_before(cmds, "SEP", i_mtr, tips))
  )
  out.append(
    ("aspirate steps", {fw.aspirate_steps(nominal, multi)}, set(tip_values(cmds[i_mtr][1], tips)))
  )
  out.append(("notes volume", round(m.calibrated(nominal), 2), case["calibrated_ul"]))
  # retract: speed, and target = detected Z (RPZ0 after MDT) + submerge + 5 mm
  out.append(("retract speed", {fw.retract_speed()}, values_before(cmds, "SSZ", i_maz, tips)))
  detected = [int(v) for v in case["first_rpz0"].split(",")]
  maz = cmds[i_maz][1]
  addressed = [t for t in tips if t - 1 < len(maz) and maz[t - 1] is not None]
  want_maz = [detected[t - 1] + fw.submerge() + fw.retract_z_above_surface() for t in addressed]
  out.append(("retract Z", want_maz, [maz[t - 1] for t in addressed]))
  # trailing air gap after the retract
  i_tag = first(cmds, "PPR", start=i_maz)
  if i_tag is not None:
    out.append(("TAG", {fw.steps(m.trail_air_gap)}, set(tip_values(cmds[i_tag][1], tips))))
    out.append(("TAG speed", {fw.speed(m.trail_speed)}, values_before(cmds, "SEP", i_tag, tips)))
  return out


def check_dispense(
  fw: LihaFirmwareValues, case, cmds: List[Command]
) -> List[Tuple[str, object, object]]:
  tips = case["tips"]
  m = fw.lc.single
  out: List[Tuple[str, object, object]] = []
  i_mtr = first(cmds, "MTR", sign=-1)
  if i_mtr is None:
    raise Missing("no dispensing MTR")
  out.append(
    ("dispense speed", {fw.speed(m.dispense_speed)}, values_before(cmds, "SEP", i_mtr, tips))
  )
  out.append(
    ("breakoff speed", {fw.speed(m.breakoff_speed)}, values_before(cmds, "SPP", i_mtr, tips))
  )
  # EVOware dispenses the volume its notes give (what the tip holds; see test_simple_dispense_volume)
  out.append(
    ("dispense steps", {-fw.steps(case["calibrated_ul"])}, set(tip_values(cmds[i_mtr][1], tips)))
  )
  if fw.lc.dispense_lld:
    i_det = first(cmds, "MET")
    if i_det is None:
      i_det = first(cmds, "MDT")
    if i_det is None:
      raise Missing("no MET/MDT for a wet dispense")
    i_sdm = before(cmds, "SDM", i_det)
    if i_sdm is None:
      raise Missing("no SDM before MET")
    out.append(("SDM", fw.detection_mode, tuple(cmds[i_sdm][1])))
    out.append(("SBL", {fw.submerge(dispense=True)}, values_before(cmds, "SBL", i_det, tips)))
    i_maz = first(cmds, "MAZ", start=i_mtr)
    if i_maz is not None:
      out.append(
        (
          "retract speed",
          {fw.retract_speed(dispense=True)},
          values_before(cmds, "SSZ", i_maz, tips),
        )
      )
  return out


# Dispenses whose noted volume is not "calibrated + leading + trailing air gap" because the tips
# held more from the preceding steps (per-tip aspirations and a mix before, or a repeated air gap).
EXTRA_TIP_CONTENT_LOGS = {
  "EVO_20260205_133842.LOG": "checker pattern: +5 uL from an earlier air gap",
  "EVO_20260205_134547.LOG": "checker pattern: +5 uL",
  "EVO_20260205_135057.LOG": "checker pattern: +5 uL",
  "EVO_20260408_144323.LOG": "+10 uL: second aspiration of the DiTi air gap",
}


class GoldenLiquidClassTests(unittest.TestCase):
  def test_liquid_class_values_match_evoware_logs(self):
    xml, cases = load_cases()
    self.assertGreater(len(cases), 250)
    checked, refused = 0, set()
    failures: List[tuple] = []
    for case in cases:
      try:
        fw = case_values(xml, case)
      except NotImplementedError:
        refused.add(case["class"])
        continue
      cmds = parse(case["commands"])
      check = check_aspirate if case["verb"] == "aspirate" else check_dispense
      try:
        results = check(fw, case, cmds)
      except Missing as exc:
        failures.append(
          (case["log"], case["class"], case["range"], case["verb"], "missing", str(exc), None)
        )
        continue
      for name, want, got in results:
        checked += 1
        if want != got:
          failures.append(
            (case["log"], case["class"], case["range"], case["verb"], name, want, got)
          )
    self.assertEqual(failures, [])
    self.assertGreater(checked, 2500)
    # logged only aspirating; its dispense position code 6 is not verified
    self.assertEqual(refused, {"Water dry contact High density plate"})

  def test_simple_dispense_volume(self):
    """A full single dispense is calibrated volume + LAG + TAG, unless the tip held more."""
    xml, cases = load_cases()
    simple, exceptions = 0, set()
    for case in cases:
      if case["verb"] != "dispense" or case["class"] == "Water dry contact High density plate":
        continue
      fw = case_values(xml, case)
      steps = fw.dispense_steps(case["nominal_ul"])
      if steps == -fw.steps(case["calibrated_ul"]):
        simple += 1
      else:
        exceptions.add(case["log"])
    self.assertEqual(sorted(exceptions), sorted(EXTRA_TIP_CONTENT_LOGS))
    self.assertGreater(simple, 55)


class LibraryTests(unittest.TestCase):
  def setUp(self):
    self.lib = LiquidClassLibrary.from_database(HERE / "evoware")

  def test_water_free_dispense_150ul(self):
    lc = self.lib.liha("Water free dispense", 150)
    fw = LihaFirmwareValues(lc, STEPS_PER_UL)
    self.assertEqual(lc.volume_range, (15.01, 200.01))
    self.assertEqual(fw.aspirate_steps(150), 468)  # 150 * 1.034 + 0.9 = 156.0 uL
    self.assertEqual(fw.dispense_steps(150), -513)  # + 5 LAG + 10 TAG
    self.assertEqual(
      (fw.speed(lc.single.aspirate_speed), fw.speed(lc.single.dispense_speed)), (600, 3600)
    )
    self.assertEqual(fw.speed(lc.single.breakoff_speed), 2400)
    self.assertEqual((fw.detection_mode, fw.search_speed, fw.retract_distance), ((7, 1), 600, 40))
    self.assertEqual(
      (fw.submerge(), fw.retract_speed(), fw.retract_z_above_surface()), (20, 200, 50)
    )
    self.assertEqual((lc.dispense_lld, lc.dispense_position), (False, "z_dispense"))

  def test_water_wet_contact_150ul(self):
    lc = self.lib.liha("Water wet contact", 150)
    fw = LihaFirmwareValues(lc, STEPS_PER_UL)
    self.assertEqual(fw.aspirate_steps(150), 471)  # 150 * 1.0347 + 1.93 = 157.135 uL
    self.assertEqual(fw.dispense_steps(150), -516)
    self.assertEqual(
      (lc.dispense_lld, lc.dispense_position, fw.submerge(dispense=True)),
      (True, "liquid_level", -10),
    )
    self.assertEqual(fw.retract_speed(dispense=True), 500)

  def test_subclass_selection_and_refusals(self):
    self.assertEqual(self.lib.liha("Water free dispense", 15).volume_range, (3.0, 15.01))
    self.assertEqual(self.lib.liha("Water free dispense", 500).volume_range, (200.01, 1000.01))
    with self.assertRaises(ValueError):
      self.lib.liha("Water free dispense", 1500)
    with self.assertRaises(KeyError):
      self.lib.liha("No such class", 100)
    with self.assertRaises(NotImplementedError):
      LihaFirmwareValues(self.lib.liha("Water free dispense", 150), STEPS_PER_UL).dispense_steps(
        150, multi=True
      )

  def test_unverified_position_codes_are_refused(self):
    xml, cases = load_cases()
    case = next(c for c in cases if c["class"] == "Water free dispense")
    sc = ET.fromstring(xml[case["subclass"]])
    node = sc.find("Dispense/Retract")
    assert node is not None
    node.set("position", "0")
    with self.assertRaisesRegex(NotImplementedError, "dispense retract position 0"):
      subclass_to_liha("x", ET.fromstring(xml[case["global"]]), sc)
    sc = ET.fromstring(xml[case["subclass"]])
    node = sc.find("Aspirate/Mix")
    assert node is not None
    node.set("enabled", "True")
    with self.assertRaisesRegex(NotImplementedError, "Mix"):
      subclass_to_liha("x", ET.fromstring(xml[case["global"]]), sc)


if __name__ == "__main__":
  unittest.main()
