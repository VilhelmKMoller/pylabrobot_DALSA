"""LiHa (C5) error mapping and the follow-up reports, replayed against real EVOware error cases."""

import asyncio
import contextlib
import json
import unittest
from pathlib import Path
from typing import Any, Dict, List, Union

from pylabrobot.legacy.liquid_handling.backends.tecan import liha_firmware
from pylabrobot.legacy.liquid_handling.backends.tecan.errors import (
  LIHA_ERROR_CLASSES,
  LIHA_ERRORS,
  TecanError,
  TecanLihaCrashError,
  TecanLihaDoorLockError,
  TecanLihaError,
  TecanLihaNoLiquidError,
  TecanLihaNotEnoughLiquidError,
  TecanLihaSubdeviceError,
  TecanLihaTipMountedError,
  TecanLihaTipNotFetchedError,
  TecanLihaTipNotMountedError,
  decode_extended_errors,
  error_code_to_exception,
)
from pylabrobot.legacy.liquid_handling.backends.tecan.liha_firmware import (
  HARDWARE_OPTIONS,
  LihaFirmware,
)
from pylabrobot.legacy.liquid_handling.backends.tecan.liha_firmware_tests import (
  decode,
  evoware_normal_form,
  wire_text,
)
from pylabrobot.resources.errors import HasTipError, NoTipError, TooLittleLiquidError

SPEC_DIR = Path(__file__).parent / "test_data" / "liha_spec"
FOLLOW_UP_CODES = (
  liha_firmware.REPORT_AXES_AFTER
  | liha_firmware.REPORT_TIPS_AFTER
  | liha_firmware.REPORT_DILUTORS_AFTER
)


def parse_data(text: str) -> List[Union[int, str]]:
  """Reply data as EVOBackend.parse_response returns it."""
  return [int(x) if x.lstrip("-").isdigit() else x for x in text.split(",") if x]


class ErrorTableTests(unittest.TestCase):
  def test_table_matches_spec_section_5(self):
    spec = json.loads((SPEC_DIR / "5_return_codes.json").read_text())["codes"]
    for code, entry in spec.items():
      if code == "0":
        continue
      self.assertEqual(LIHA_ERRORS[int(code)], entry["description"], code)
    self.assertEqual(set(LIHA_ERRORS) - {int(c) for c in spec}, {250})

  def test_codes_map_to_typed_errors(self):
    for code in LIHA_ERRORS:
      exc = error_code_to_exception("C5", code)
      self.assertIsInstance(exc, TecanLihaError)
      self.assertIsInstance(exc, TecanError)
      self.assertIs(type(exc), LIHA_ERROR_CLASSES.get(code, TecanLihaError))
      self.assertEqual((exc.module, exc.error_code, exc.message), ("C5", code, LIHA_ERRORS[code]))
    self.assertIsInstance(error_code_to_exception("C5", 11), TooLittleLiquidError)
    self.assertIsInstance(error_code_to_exception("C5", 12), TecanLihaNotEnoughLiquidError)
    self.assertIsInstance(error_code_to_exception("C5", 25), NoTipError)
    self.assertIsInstance(error_code_to_exception("C5", 26), NoTipError)
    self.assertIsInstance(error_code_to_exception("C5", 27), HasTipError)
    self.assertIsInstance(error_code_to_exception("C5", 10), TecanLihaCrashError)
    self.assertIsInstance(error_code_to_exception("C5", 250), TecanLihaDoorLockError)
    unknown = error_code_to_exception("C5", 99)
    self.assertIs(type(unknown), TecanLihaError)
    self.assertEqual(unknown.message, "Unknown error code 99")

  def test_other_modules_unchanged(self):
    self.assertIs(type(error_code_to_exception("C1", 9)), TecanError)
    self.assertNotIsInstance(error_code_to_exception("W1", 25), TecanLihaError)

  def test_decode_extended_errors_examples_from_logs(self):
    axes = decode_extended_errors("@@@IIWWWWIJ")
    self.assertEqual(list(axes)[:4], ["X", "Y", "Ys", "Z1"])
    exc = error_code_to_exception("C5", 10)
    assert isinstance(exc, TecanLihaError)
    exc.axis_errors = axes
    self.assertEqual(exc.tip_errors, {0: 9, 1: 9, 2: 23, 3: 23, 4: 23, 5: 23, 6: 9, 7: 10})
    self.assertIn("tip errors", str(exc))
    self.assertEqual(decode_extended_errors("@@@@@@@@@@@"), {a: 0 for a in axes})
    for bad in ("@@@", "@@@I@@@@@@@@", "000I0000000"):
      with self.assertRaises(ValueError):
        decode_extended_errors(bad)


class Replay:
  """Fake sender: raises ``error`` on the first command, answers one follow-up report, then fails."""

  def __init__(self, error: int, follow_up: Union[str, None] = None, reply: str = ""):
    self.error, self.follow_up, self.reply = error, follow_up, reply
    self.sent: List[str] = []

  async def __call__(self, module: str, command: str, params: List[Any]) -> Dict[str, Any]:
    self.sent.append(wire_text(command, params))
    if len(self.sent) == 1:
      raise error_code_to_exception(module, self.error)
    if command == self.follow_up:
      return {"module": module, "data": parse_data(self.reply)}
    raise TecanError("no reply recorded for this follow-up", module, 6)


class FollowUpReportTests(unittest.TestCase):
  def run_case(self, sender, code, *values, options=HARDWARE_OPTIONS) -> TecanLihaError:
    fw = LihaFirmware(sender, options=options)
    with self.assertRaises(TecanLihaError) as ctx:
      asyncio.run(fw.execute(code, *values))
    return ctx.exception

  def test_every_evoware_error_case(self):
    """Replay all 106 distinct LiHa errors from the EVOware logs with the instrument's answers."""
    cases = json.loads((SPEC_DIR / "evoware_c5_errors.json").read_text())["cases"]
    self.assertGreater(len(cases), 100)
    fw = LihaFirmware(Replay(0), options=HARDWARE_OPTIONS)  # only used to decode the commands
    for case in cases:
      if case["follow_up_status"] not in (None, 0):
        continue
      with self.subTest(command=case["command"], error=case["error"]):
        code, values = decode(fw, case["command"])
        sender = Replay(case["error"], case["follow_up"], case["follow_up_reply"] or "")
        # A follow-up the log has no answer for fails in the fake sender and is logged as a warning.
        asks = case["error"] in FOLLOW_UP_CODES
        expect_warning = asks and case["follow_up"] is None
        with (
          self.assertLogs(liha_firmware.logger, "WARNING")
          if expect_warning
          else contextlib.nullcontext()
        ):
          exc = self.run_case(sender, code, *values)
        self.assertIs(type(exc), LIHA_ERROR_CLASSES.get(case["error"], TecanLihaError))
        self.assertEqual(sender.sent[0], evoware_normal_form(case["command"]))
        if case["follow_up"] == "REE":
          self.assertEqual(sender.sent[1:], ["REE"])
          self.assertEqual(exc.axis_errors, decode_extended_errors(case["follow_up_reply"]))
          assert exc.axis_errors is not None
          self.assertTrue(exc.tip_errors or exc.axis_errors["X"] or exc.axis_errors["Y"])
        elif case["follow_up"] == "RTS":
          self.assertEqual(sender.sent[1:], ["RTS"])
          self.assertEqual(exc.mounted_tips, int(case["follow_up_reply"]))
        elif case["error"] in (3, 250):
          self.assertEqual(len(sender.sent), 1)  # nothing moved; no follow-up report
          self.assertIsNone(exc.axis_errors)

  def test_ree_codes_name_the_failing_tip(self):
    exc = self.run_case(Replay(9, "REE", "@@@I@@@@@@@"), "MDT", 1, None, None, None, [3])
    self.assertIsInstance(exc, TecanLihaNoLiquidError)
    self.assertEqual(exc.tip_errors, {0: 9})

  def test_rts_after_tip_not_fetched(self):
    exc = self.run_case(Replay(25, "RTS", "127"), "AGT", 255, 783, 210, 0, None)
    self.assertIsInstance(exc, TecanLihaTipNotFetchedError)
    self.assertIsInstance(exc, NoTipError)
    self.assertEqual(exc.mounted_tips, 127)  # tip 8 not fetched
    self.assertIn("mounted tips 01111111", str(exc))

  def test_rts_after_tip_mounted_and_not_mounted(self):
    self.assertEqual(self.run_case(Replay(27, "RTS", "255"), "AST", 255, 0).mounted_tips, 255)
    exc = self.run_case(Replay(26, "RTS", "0"), "AGT", 15, 724, 210, 0, None)
    self.assertIsInstance(exc, TecanLihaTipNotMountedError)
    self.assertEqual(exc.mounted_tips, 0)
    self.assertIsInstance(self.run_case(Replay(27, "RTS", "1"), "ADT", 1), TecanLihaTipMountedError)

  def test_rds_after_subdevice_error(self):
    exc = self.run_case(Replay(28, "RDS", "5,7,7,7,7,7,0,0"), "PPR", [3000])
    self.assertIsInstance(exc, TecanLihaSubdeviceError)
    self.assertEqual(exc.dilutor_errors, [5, 7, 7, 7, 7, 7, 0, 0])
    self.assertEqual(exc.mounted_tips, None)
    self.assertIn("dilutor errors", str(exc))

  def test_failing_follow_up_keeps_the_original_error(self):
    sender = Replay(10, None)
    with self.assertLogs(liha_firmware.logger, "WARNING") as logs:
      exc = self.run_case(sender, "PAZ", [514])
    self.assertIsInstance(exc, TecanLihaCrashError)
    self.assertIsNone(exc.axis_errors)
    self.assertEqual(sender.sent, ["PAZ514", "REE"])
    self.assertIn("REE after PAZ error 10 failed", logs.output[0])

  def test_malformed_follow_up_reply_is_ignored(self):
    with self.assertLogs(liha_firmware.logger, "WARNING"):
      exc = self.run_case(Replay(9, "REE", "12"), "MDT", 1, None, None, None, None)
    self.assertIsNone(exc.axis_errors)
    with self.assertLogs(liha_firmware.logger, "WARNING"):
      exc = self.run_case(Replay(25, "RTS", "x"), "AGT", 1, 700, 210, 0, None)
    self.assertIsNone(exc.mounted_tips)

  def test_command_errors_get_no_follow_up(self):
    for code in (2, 3, 5, 250):
      sender = Replay(code, "REE", "@@@@@@@@@@@")
      self.run_case(sender, "PAZ", [2100])
      self.assertEqual(len(sender.sent), 1, code)

  def test_other_modules_errors_pass_through(self):
    async def send(module, command, params):
      raise error_code_to_exception("W1", 25)

    with self.assertRaises(TecanError) as ctx:
      asyncio.run(LihaFirmware(send).report_tip_status())
    self.assertNotIsInstance(ctx.exception, TecanLihaError)


if __name__ == "__main__":
  unittest.main()
