"""Tecan backend errors"""

from typing import Dict, List, Optional, Type

from pylabrobot.resources.errors import HasTipError, NoTipError, TooLittleLiquidError


class TecanError(Exception):
  """Tecan backend errors, raised by a single module."""

  def __init__(
    self,
    message: str,
    module: str,
    error_code: int,
  ):
    self.message = message
    self.module = module
    self.error_code = error_code

  def __repr__(self) -> str:
    return f"{self.__class__.__name__}('{self.message}')"


class TecanTipsNotFetchedError(TecanError, NoTipError):
  """MCA tip fetch (ATF) reached Zend without stalling on tips (W1 error 25)."""


# MCA (W1) error codes, firmware manual 394562 V1.0 (command error lists, pp. 27-39).
MCA_ERRORS = {
  1: "Initialization failed",
  2: "Invalid command",
  3: "Invalid operand",
  4: "CAN acknowledge problems",
  5: "Device not implemented",
  6: "CAN answer timeout",
  7: "Device not initialized",
  9: "Plate not fetched",
  10: "Drive no load",
  16: "Sub-device error",
  25: "Tips not fetched",
  26: "Plunger blocked",
}


# LiHa (C5) return codes, firmware manual 392736 V1.80 section 5 (p. 77). Code 16 is only on p. 30
# (RDX). Code 250 is not in the manual: EVOware logs show it on C5 commands sent with the door
# locks open (reply text "Command blocked due open doorlocks"), so it comes from the instrument's
# door interlock, not from the LiHa firmware.
LIHA_ERRORS = {
  1: "Initialization failed",
  2: "Invalid command",
  3: "Invalid operand",
  4: "CAN acknowledge problems",
  5: "Device not implemented",
  6: "CAN answer timeout",
  7: "Device not initialized",
  8: "Command overflow of TeCU",
  9: "No liquid detected",
  10: "Drive no load",
  11: "Not enough liquid",
  12: "Not enough liquid",
  13: "No Flash access",
  15: "Command overflow of subdevice",
  16: "Power fail circuit error",
  17: "Measurement failed",
  18: "Clot limit passed",
  19: "No clot exit detected",
  20: "No liquid exit detected",
  21: "Delta pressure overrun (pLLD)",
  22: "Tip Guard in wrong position",
  23: "Not yet moved or move aborted",
  24: "llid pulse error or reed crosstalk error",
  25: "Tip not fetched",
  26: "Tip not mounted",
  27: "Tip mounted",
  28: "Subdevice error",
  29: "Application switch and axes mismatch",
  30: "Wrong DC-Servo type",
  31: "Virtual Drive",
  250: "Command blocked due open doorlocks",
}

LIHA_AXES = ("X", "Y", "Ys") + tuple(f"Z{i}" for i in range(1, 9))
"""Axis order of the REE strings, 'XYSZZZZZZZZ' (REE1) on an 8-tip LiHa with Y-spacing."""


def decode_extended_errors(text: str, axes=LIHA_AXES) -> Dict[str, int]:
  """Decode an REE0 reply: one character per axis, error code + 0x40 ('@' = 0, 'I' = 9, ...).

  E.g. ``'@@@IIWWWWIJ'`` -> Z1, Z2 and Z7 error 9, Z3..Z6 error 23, Z8 error 10, the rest 0.
  """
  if len(text) != len(axes) or any(not 0x40 <= ord(c) <= 0x7F for c in text):
    raise ValueError(f"unexpected REE reply {text!r} for axes {axes}")
  return {axis: ord(c) - 0x40 for axis, c in zip(axes, text)}


class TecanLihaError(TecanError):
  """LiHa (C5) error. The firmware layer adds what the instrument reports after the error:

  - ``axis_errors``: per-axis codes from REE (X, Y, Ys, Z1..Z8), after liquid/move errors;
  - ``mounted_tips``: RTS bit mask (bit 0 = tip 1), after DiTi errors 25/26/27;
  - ``dilutor_errors``: RDS codes of dilutors 1..8, after error 28 (meaning per the XP2000/XP6000
    dilutor manual, which we do not have; the spec example reads 5 = not connected,
    7 = not initialized, 0 = ok).

  Each stays None when it was not read or the follow-up report failed.
  """

  def __init__(self, message: str, module: str, error_code: int):
    super().__init__(message, module, error_code)
    self.axis_errors: Optional[Dict[str, int]] = None
    self.mounted_tips: Optional[int] = None
    self.dilutor_errors: Optional[List[int]] = None

  @property
  def tip_errors(self) -> Optional[Dict[int, int]]:
    """Non-zero Z-axis codes from ``axis_errors`` by 0-based channel, or None if not read."""
    if self.axis_errors is None:
      return None
    return {
      int(axis[1:]) - 1: code
      for axis, code in self.axis_errors.items()
      if axis.startswith("Z") and code != 0
    }

  def __str__(self) -> str:
    extra = []
    if self.tip_errors:
      extra.append(f"tip errors {self.tip_errors}")
    if self.mounted_tips is not None:
      extra.append(f"mounted tips {self.mounted_tips:08b}")
    if self.dilutor_errors is not None:
      extra.append(f"dilutor errors {self.dilutor_errors}")
    detail = f" ({'; '.join(extra)})" if extra else ""
    return f"{self.module} error {self.error_code}: {self.message}{detail}"


class TecanLihaNotInitializedError(TecanLihaError):
  """Error 7: a move was sent before the axes were initialized."""


class TecanLihaNoLiquidError(TecanLihaError):
  """Error 9: MDT/MET found no liquid down to Z-max."""


class TecanLihaCrashError(TecanLihaError):
  """Error 10 'Drive no load': crash detected."""


class TecanLihaNotEnoughLiquidError(TecanLihaError, TooLittleLiquidError):
  """Errors 11 (MDT) / 12 (MET): liquid found, but not enough below it before Z-max."""


class TecanLihaClotError(TecanLihaError):
  """Errors 18 (clot limit passed) / 19 (no clot exit detected), MCT."""


class TecanLihaLiquidDetectionError(TecanLihaError):
  """Errors 20, 21, 23, 24: liquid detection failed (exit, pLLD overrun, aborted, lld pulse)."""


class TecanLihaTipNotFetchedError(TecanLihaError, NoTipError):
  """Error 25: AGT did not fetch the tip(s)."""


class TecanLihaTipNotMountedError(TecanLihaError, NoTipError):
  """Error 26: no tip mounted where one is needed."""


class TecanLihaTipMountedError(TecanLihaError, HasTipError):
  """Error 27: a tip is (still) mounted."""


class TecanLihaSubdeviceError(TecanLihaError):
  """Error 28: a dilutor or other subdevice failed; see ``dilutor_errors`` (RDS)."""


class TecanLihaDoorLockError(TecanLihaError):
  """Error 250 (not in the LiHa manual): command blocked because the door locks are open."""


LIHA_ERROR_CLASSES: Dict[int, Type[TecanLihaError]] = {
  7: TecanLihaNotInitializedError,
  9: TecanLihaNoLiquidError,
  10: TecanLihaCrashError,
  11: TecanLihaNotEnoughLiquidError,
  12: TecanLihaNotEnoughLiquidError,
  18: TecanLihaClotError,
  19: TecanLihaClotError,
  20: TecanLihaLiquidDetectionError,
  21: TecanLihaLiquidDetectionError,
  23: TecanLihaLiquidDetectionError,
  24: TecanLihaLiquidDetectionError,
  25: TecanLihaTipNotFetchedError,
  26: TecanLihaTipNotMountedError,
  27: TecanLihaTipMountedError,
  28: TecanLihaSubdeviceError,
  250: TecanLihaDoorLockError,
}


def error_code_to_exception(module: str, error_code: int) -> TecanError:
  """Convert an error code to an exception"""
  table = None

  if module == "C5":
    cls = LIHA_ERROR_CLASSES.get(error_code, TecanLihaError)
    message = LIHA_ERRORS.get(error_code, f"Unknown error code {error_code}")
    return cls(message, module, error_code)
  elif module == "C1":
    table = {
      1: "Initialization failed",
      2: "Invalid command",
      3: "Invalid operand",
      4: "CAN acknowledge problems",
      5: "Device not implemented",
      6: "CAN answer timeout",
      7: "Device not initialized",
      8: "Command overflow of TeCU",
      9: "Plate not fetched",
      10: "Drive no load",
      11: "Sub device not ready yet",
      13: "No access to Flash-EPROM",
      14: "Hardware not defined",
      15: "Command overflow of this device",
      17: "Verification failed",
      21: "BCS communication error",
      25: "Download Error",
      28: "Sub device error",
      30: "Invalid servo version",
    }

  elif module == "W1":
    if error_code == 25:
      return TecanTipsNotFetchedError(MCA_ERRORS[25], module, error_code)
    table = MCA_ERRORS

  if table is not None and error_code in table:
    return TecanError(table[error_code], module, error_code)

  return TecanError(f"Unknown error code {error_code}", module, error_code)
