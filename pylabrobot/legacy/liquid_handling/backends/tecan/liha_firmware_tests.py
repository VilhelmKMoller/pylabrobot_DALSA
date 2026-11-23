"""Offline tests of the LiHa firmware layer against the spec table and every EVOware LiHa command."""

import asyncio
import inspect
import json
import re
import unittest
from pathlib import Path
from typing import Any, List

from pylabrobot.legacy.liquid_handling.backends.tecan import liha_firmware
from pylabrobot.legacy.liquid_handling.backends.tecan.liha_firmware import (
  COMMANDS,
  HARDWARE_OPTIONS,
  LihaFirmware,
  LihaFirmwareError,
  LihaOptionNotInstalled,
)

HERE = Path(__file__).parent
SPEC_DIR = HERE / "test_data" / "liha_spec"


def load_spec():
  commands = {}
  for path in sorted(SPEC_DIR.glob("4.*.json")):
    for c in json.loads(path.read_text())["commands"]:
      commands[c["code"]] = c
  return commands


SPEC = load_spec()
EXCLUDED = sorted(c for c, s in SPEC.items() if s["status"] == "excluded")


class Recorder:
  def __init__(self):
    self.calls = []

  async def __call__(self, **kwargs):
    self.calls.append(kwargs)
    return {"module": kwargs["module"], "data": []}


def wire_text(command, params):
  """The command text as EVOBackend._assemble_command writes it, without module and framing."""
  return command + ",".join("" if p is None else str(p) for p in params)


def evoware_normal_form(text):
  """EVOware's text with its two value-neutral quirks removed: trailing empty fields
  ('MAZ,,,,,,1019,' = 'MAZ,,,,,,1019', 748 strings) and negative zero ('STZ-0' = 'STZ0', 8 strings)."""
  code, rest = text[:3], text[3:].rstrip(",")
  return code + ",".join("0" if f == "-0" else f for f in rest.split(",")) if rest else code


def decode(fw, text):
  """Split an EVOware command string into (code, method values) using the command table."""
  code, rest = text[:3], text[3:]
  raw = rest.split(",") if rest else []
  values: List[Any] = []
  i = 0
  for p in [p for p in COMMANDS[code]["params"] if not p["excluded"]]:
    n = min(p["count"], fw.num_tips) if p["count"] > 1 else 1
    chunk = [None if x == "" else int(x) for x in raw[i : i + n]]
    i += n
    if p["count"] > 1:
      values.append(chunk or None)
    else:
      values.append(chunk[0] if chunk else None)
  if i < len(raw):
    raise AssertionError(f"{text}: {len(raw) - i} parameters left over")
  return code, values


class SpecTableTests(unittest.TestCase):
  def test_commands_match_the_spec_table(self):
    expected = {c for c, s in SPEC.items() if s["status"] != "excluded" and c != "T"}
    self.assertEqual(set(COMMANDS), expected)
    for code, entry in COMMANDS.items():
      spec = SPEC[code]
      self.assertEqual(entry["status"], spec["status"], code)
      self.assertEqual(entry["option"], spec.get("option"), code)
      self.assertEqual(entry["moves_x"], bool(spec.get("moves_x")), code)
      self.assertEqual(entry["untested"], bool(spec.get("untested")), code)
      self.assertEqual(len(entry["params"]), len(spec["params"]), code)
      for mine, theirs in zip(entry["params"], spec["params"]):
        where = f"{code} {theirs['name']}"
        self.assertEqual(mine["name"], theirs["name"], where)
        self.assertEqual(mine["count"], theirs.get("count", 1), where)
        for key in ("min", "max"):
          raw = theirs.get(key)
          self.assertEqual(mine[key], raw if type(raw) is int else None, where)
        self.assertEqual(mine["allowed"], theirs.get("allowed"), where)
        self.assertEqual(mine["allowed_range"], theirs.get("allowed_range"), where)
        self.assertEqual(mine["excluded_values"], theirs.get("excluded_values"), where)
        self.assertEqual(mine["optional"], bool(theirs.get("optional")), where)
        self.assertEqual(mine["excluded"], bool(theirs.get("excluded")), where)

  def test_every_command_has_its_method(self):
    names = [entry["method"] for entry in COMMANDS.values()]
    self.assertEqual(len(names), len(set(names)))
    for code, entry in COMMANDS.items():
      method = getattr(LihaFirmware, entry["method"])
      self.assertTrue(inspect.iscoroutinefunction(method), code)
      self.assertIn(f"{code}: ", method.__doc__ or "")

  def test_msd_is_marked_untested(self):
    self.assertTrue(COMMANDS["MSD"]["untested"])
    self.assertIn("NOT TESTED", LihaFirmware.spiral_move.__doc__ or "")


class ExcludedCommandTests(unittest.TestCase):
  """Operator rule 2026-11-21: no memory-write or calibration command on any arm."""

  def test_excluded_commands_are_not_available(self):
    self.assertIn("AWE", EXCLUDED)
    fw = LihaFirmware(Recorder(), options=HARDWARE_OPTIONS)
    for code in EXCLUDED:
      self.assertNotIn(code, COMMANDS)
      with self.assertRaises(LihaFirmwareError, msg=code):
        fw.encode(code)

  def test_no_arm_source_sends_an_excluded_command(self):
    sources = (
      [HERE / "EVO_backend.py"] + sorted(HERE.glob("liha*.py")) + sorted(HERE.glob("mca*.py"))
    )
    sources = [p for p in sources if not p.name.endswith("_tests.py")]
    pattern = re.compile(r"""["'](%s)["'0-9,]""" % "|".join(EXCLUDED))
    found = []
    for path in sources:
      for n, line in enumerate(path.read_text().splitlines(), 1):
        if pattern.search(line):
          found.append(f"{path.name}:{n}: {line.strip()}")
    self.assertEqual(found, [])

  def test_excluded_parameters_are_never_sent(self):
    async def run():
      rec = Recorder()
      fw = LihaFirmware(rec)
      await fw.initialize_x(init_speed=400)
      await fw.initialize_y(init_speed=350)
      await fw.initialize_z(tip_select=255, init_speed=270)
      return rec.calls

    calls = asyncio.run(run())
    self.assertEqual([len(c["params"]) for c in calls], [1, 1, 2])
    for name in ("initialize_x", "initialize_y", "initialize_z"):
      params = inspect.signature(getattr(LihaFirmware, name)).parameters
      self.assertNotIn("range", params)
      self.assertNotIn("offset", params)

  def test_tip_guard_calibration_values_refused(self):
    fw = LihaFirmware(Recorder(), options={"tip_guard"})
    self.assertEqual(fw.encode("PTG", 1), [1])
    for value in (99, 100, 101):
      with self.assertRaisesRegex(LihaFirmwareError, "calibration"):
        fw.encode("PTG", value)


class EvowareRoundTripTests(unittest.TestCase):
  def test_every_evoware_liha_command_round_trips(self):
    """Each distinct C5 command EVOware sent decodes into a valid call that re-encodes identically."""
    fixture = json.loads((SPEC_DIR / "evoware_c5_commands.json").read_text())["commands"]
    self.assertGreater(len(fixture), 2000)
    fw = LihaFirmware(Recorder(), options=HARDWARE_OPTIONS)
    refused_t, checked = [], 0
    for text in fixture:
      if re.match(r"T[0-9A-F]{2}[A-Z]{3}", text):
        layer, device, report = int(text[1]), int(text[2], 16), text[3:6]
        params = [int(x) for x in text[6:].split(",")] if text[6:] else []
        rec = Recorder()
        try:
          asyncio.run(LihaFirmware(rec).report_subdevice(layer, device, report, *params))
        except LihaFirmwareError:
          refused_t.append(text)
          continue
        self.assertEqual(wire_text(rec.calls[0]["command"], rec.calls[0]["params"]), text)
        continue
      code, values = decode(fw, text)
      self.assertEqual(wire_text(code, fw.encode(code, *values)), evoware_normal_form(text), text)
      checked += 1
    self.assertGreater(checked, 2000)
    self.assertTrue(refused_t and all(t.startswith("T30AFI") for t in refused_t), refused_t)


class EncodingTests(unittest.TestCase):
  def test_sender_receives_module_command_params(self):
    async def run():
      rec = Recorder()
      reply = await LihaFirmware(rec).report_tip_status()
      return rec.calls, reply

    calls, reply = asyncio.run(run())
    self.assertEqual(calls, [{"module": "C5", "command": "RTS", "params": []}])
    self.assertEqual(reply, {"module": "C5", "data": []})

  def test_per_tip_groups_keep_later_parameters_in_place(self):
    fw = LihaFirmware(Recorder())
    self.assertEqual(fw.encode("MAZ", [1000], 200), [1000] + [None] * 7 + [200])
    self.assertEqual(fw.encode("SHZ", [None, 2000]), [None, 2000])
    self.assertEqual(fw.encode("PAA", 100, None, None, None), [100])
    four = LihaFirmware(Recorder(), num_tips=4)
    self.assertEqual(four.encode("MAZ", [1000], 200), [1000, None, None, None, 200])
    with self.assertRaises(LihaFirmwareError):
      four.encode("SHZ", [1, 2, 3, 4, 5])

  def test_invalid_values_refused_before_sending(self):
    fw = LihaFirmware(Recorder())
    bad = [
      ("SHZ", ([-1],)),  # below documented bound
      ("SSL", ([1501],)),  # above
      ("SSL", ([True],)),  # bool is not int
      ("SSL", ("400",)),  # string is not a list
      ("SSL", ([400] * 9,)),  # too many tips
      ("RPX", (12,)),  # selector out of range
      ("RDX", (3,)),  # not in allowed set 0,1,2,5
      ("PSY", (50,)),  # neither 0 nor 90..380
      ("AGT", (255, None, 0, 0, None)),  # Zstart required
      ("RGD", (None,)),  # Selector required
      ("PPR", ([3151],)),
      ("SEP", ([4],)),
    ]
    for code, values in bad:
      with self.assertRaises(LihaFirmwareError, msg=f"{code}{values}"):
        fw.encode(code, *values)
    with self.assertRaises(LihaFirmwareError):
      fw.encode("RPX")  # wrong number of method values

  def test_cross_parameter_rules(self):
    fw = LihaFirmware(Recorder(), options={"piercing"})
    with self.assertRaisesRegex(LihaFirmwareError, "Zmax"):
      fw.encode("MDT", 255, 0, 1000, 1500, None)
    with self.assertRaisesRegex(LihaFirmwareError, "Zadd"):
      fw.encode("MDT", 255, 0, 1500, 1000, [600])
    self.assertEqual(fw.encode("MDT", 255, 0, 1500, 1000, [500]), [255, 0, 1500, 1000, 500])
    with self.assertRaisesRegex(LihaFirmwareError, "Limit"):
      fw.encode("MCT", 255, 50, 200, 50)
    with self.assertRaisesRegex(LihaFirmwareError, "SearchDist"):
      fw.encode("AGT", 255, 700, 800, 0, None)
    with self.assertRaisesRegex(LihaFirmwareError, "both"):
      fw.encode("APT", 255, 1000, None)
    with self.assertRaisesRegex(LihaFirmwareError, "same sign"):
      fw.encode("MSD", 10, -10, 2, 255, 0, 0, 100, 1000)

  def test_hardware_options(self):
    fw = LihaFirmware(Recorder())
    for code, values in [("SDE", (1,)), ("RBK", ()), ("RAP", ()), ("AXT", (0,)), ("APS", (1,))]:
      with self.assertRaises(LihaOptionNotInstalled, msg=code):
        fw.encode(code, *values)
    self.assertEqual(LihaFirmware(Recorder(), options={"lower_diti_eject"}).encode("SDE", 1), [1])
    with self.assertRaises(LihaOptionNotInstalled):
      fw.encode("SDM", 10, 1)
    self.assertEqual(fw.encode("SDM", 1, 0), [1, 0])
    pmp = LihaFirmware(Recorder(), options={"pmp"})
    self.assertEqual(pmp.encode("SDM", 10, 1), [10, 1])
    with self.assertRaisesRegex(LihaFirmwareError, "pLLD"):
      pmp.encode("SDM", 10, 0)
    with self.assertRaises(ValueError):
      LihaFirmware(Recorder(), options={"turbo"})

  def test_subdevice_pass_through_whitelist(self):
    async def run():
      rec = Recorder()
      fw = LihaFirmware(rec)
      await fw.report_subdevice(3, 0, "RFV", 15)
      for report in ("AFI", "PPA", "SPV"):
        with self.assertRaises(LihaFirmwareError):
          await fw.report_subdevice(3, 0, report, 1)
      with self.assertRaises(LihaFirmwareError):
        await fw.report_subdevice(8, 0, "RFV")
      return rec.calls

    self.assertEqual(asyncio.run(run()), [{"module": "C5", "command": "T30RFV", "params": [15]}])

  def test_untested_command_logs_a_warning(self):
    async def run():
      fw = LihaFirmware(Recorder())
      with self.assertLogs(liha_firmware.logger, "WARNING") as logs:
        await fw.spiral_move(
          x_delta_distance=10,
          y_delta_distance=10,
          nr_of_half_spirals=2,
          tip_select=255,
          dilutor_distance=0,
          z_tracking_distance=0,
          xy_speed=100,
          xy_acceleration=1000,
        )
      return logs.output

    self.assertIn("never been tested", asyncio.run(run())[0])

  def test_x_moving_commands_are_marked(self):
    moving = {c for c, e in COMMANDS.items() if e["moves_x"]}
    self.assertEqual(
      moving,
      {"PIA", "PIX", "PAA", "PXZ", "PAX", "PRX", "PCX", "MAX", "MAA", "MRX", "MSD"},
    )


if __name__ == "__main__":
  unittest.main()
