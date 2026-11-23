"""LiHa (C5) firmware layer for Tecan spec 392736 V1.80 ("Firmware Command Set LIHA").

One method per command (generated into ``liha_firmware_commands.py`` from
``test_data/liha_spec``). Every call goes through :meth:`LihaFirmware.encode`, which checks:

- that the command is available: excluded commands (flash writes, calibration, machine configuration,
  service) do not exist here at all, by operator decision of 2026-11-21;
- hardware options (lower DiTi eject, 8+1, PMP, piercing, tip guard, piezo, pinch valve) are refused
  unless declared in ``options``;
- parameter types (int, never bool), required/omissible parameters, per-tip list lengths;
- the fixed bounds documented in the spec, excluded parameter values, and a few cross-parameter rules;
- the firmware limit of 12 parameters per command.

It does NOT check machine-dependent travel limits (bounds given as e.g. ``0..z_range+20``), deck
geometry, X collisions with the other arms, or instrument state. Commands that move X are marked with
``COMMANDS[code]["moves_x"]``; the caller must clear the X path first (``x_coordinator``).

``EVOBackend`` sends every LiHa plan through this layer (``EVOBackend._liha_run``), with a sender
that bypasses its set-command cache: ``EVOBackend.send_command`` skips a set command repeated with
equal parameters, and the planners rely on every command being sent. A real sender moves hardware.

Parameters marked with an asterisk in the spec "may be omitted and retain the current setting";
parameters without an asterisk use their default when omitted. Both are passed as ``None`` here.
"""

import logging
from typing import Any, Awaitable, Callable, Iterable, List, Optional, Sequence

from pylabrobot.legacy.liquid_handling.backends.tecan.errors import (
  LIHA_AXES,
  TecanLihaError,
  decode_extended_errors,
)
from pylabrobot.legacy.liquid_handling.backends.tecan.liha_firmware_commands import (
  COMMANDS,
  LihaCommandsMixin,
)

logger = logging.getLogger(__name__)

MAX_PARAMS = 12
"""The LiHa firmware parses at most 12 parameters (11 commas), spec section 3.4."""

HARDWARE_OPTIONS = frozenset(
  {"lower_diti_eject", "8plus1", "pmp", "piercing", "tip_guard", "piezo", "pinch_valve"}
)

SUBDEVICE_REPORTS = ("RFV", "RPP", "RSD", "RIS")
"""Read-only subdevice reports allowed through the T pass-through: exactly those EVOware sends."""

PLLD_DETECTION_MODES = range(10, 17)

# Read-only reports sent after a LiHa error (see LihaFirmware.execute). EVOware sends REE after
# 9/10/11/12/24 and RTS after 25/26/27 (AuditTrail logs); REE is extended here to the other
# axis-specific codes. RDS after 28 follows the spec (EVOware never got a 28 in its logs).
REPORT_AXES_AFTER = frozenset({1, 7, 9, 10, 11, 12, 18, 19, 20, 21, 22, 23, 24, 29, 30, 31})
REPORT_TIPS_AFTER = frozenset({25, 26, 27})
REPORT_DILUTORS_AFTER = frozenset({28})

Sender = Callable[..., Awaitable[Any]]


class LihaFirmwareError(ValueError):
  """A LiHa command was refused before anything was sent."""


class LihaOptionNotInstalled(LihaFirmwareError):
  """The command needs a hardware option that was not declared in ``options``."""


def _is_int(value: Any) -> bool:
  return type(value) is int


def _check_value(code: str, p: dict, value: int) -> None:
  where = f"{code} {p['name']}={value}"
  if p.get("excluded_values") and value in p["excluded_values"]:
    raise LihaFirmwareError(f"{where}: calibration value, never sent")
  if p.get("allowed") is not None:
    if value not in p["allowed"]:
      raise LihaFirmwareError(f"{where}: must be {p['bound']}")
    return
  if p.get("allowed_range") is not None:
    if not any(lo <= value <= hi for lo, hi in p["allowed_range"]):
      raise LihaFirmwareError(f"{where}: must be {p['bound']}")
    return
  if p["min"] is not None and value < p["min"]:
    raise LihaFirmwareError(f"{where}: below {p['bound']}")
  if p["max"] is not None and value > p["max"]:
    raise LihaFirmwareError(f"{where}: above {p['bound']}")


class LihaFirmware(LihaCommandsMixin):
  """Send validated LiHa commands through an injected async sender.

  Args:
    sender: ``async sender(module=..., command=..., params=[...])``, e.g. a cache-bypassing wrapper
      around ``EVOBackend.send_command``. Its return value is passed back unchanged.
    module: CAN address of the LiHa (``"C5"``).
    options: installed hardware options, a subset of :data:`HARDWARE_OPTIONS`. Unknown until probed
      on the instrument (RSD, RTG0, RDA, RNT, REE1), so the default is none.
    num_tips: tips on the arm. Per-tip parameter groups (Z1..Z8, Plunger1..8, ...) have this many
      positions, which fixes where the parameters after them go on the wire.
  """

  def __init__(
    self,
    sender: Sender,
    module: str = "C5",
    options: Iterable[str] = (),
    num_tips: int = 8,
  ):
    unknown = set(options) - HARDWARE_OPTIONS
    if unknown:
      raise ValueError(f"unknown LiHa hardware options {sorted(unknown)}")
    if not 1 <= num_tips <= 8:
      raise ValueError("num_tips must be 1..8")
    self._sender = sender
    self.module = module
    self.options = frozenset(options)
    self.num_tips = num_tips

  # -- encoding ---------------------------------------------------------------------------------

  def encode(self, code: str, *values: Any) -> List[Optional[int]]:
    """Validate a command and return its wire parameters (trailing omitted ones trimmed).

    ``values`` are the method arguments in order: an int or None per scalar parameter, a sequence
    (or None) per per-tip parameter group. Raises :class:`LihaFirmwareError` on any problem.
    """
    spec = COMMANDS.get(code)
    if spec is None:
      raise LihaFirmwareError(f"LiHa command {code!r} is not available (unknown or excluded)")
    if spec["status"] == "hardware_option" and spec["option"] not in self.options:
      raise LihaOptionNotInstalled(
        f"{code} needs the hardware option {spec['option']!r}, not declared in options"
      )
    exposed = [p for p in spec["params"] if not p["excluded"]]
    if len(values) != len(exposed):
      raise LihaFirmwareError(f"{code} takes {len(exposed)} parameters, got {len(values)}")

    wire: List[Optional[int]] = []
    named = {}
    for p, value in zip(exposed, values):
      if p["count"] > 1:
        slots = min(p["count"], self.num_tips)
        if value is None:
          if p["required"]:
            raise LihaFirmwareError(f"{code} {p['name']} is required")
          items: List[Optional[int]] = []
        else:
          if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
            raise LihaFirmwareError(f"{code} {p['name']} must be a list of up to {slots} values")
          items = list(value)
          if len(items) > slots:
            raise LihaFirmwareError(f"{code} {p['name']} has {len(items)} values, max {slots}")
        for item in items:
          if item is None:
            if p["required"]:
              raise LihaFirmwareError(f"{code} {p['name']} values are required")
            continue
          if not _is_int(item):
            raise LihaFirmwareError(f"{code} {p['name']} values must be int, got {item!r}")
          _check_value(code, p, item)
        named[p["arg"]] = items
        wire.extend(items + [None] * (slots - len(items)))
      else:
        if value is None:
          if p["required"]:
            raise LihaFirmwareError(f"{code} {p['name']} is required")
        elif not _is_int(value):
          raise LihaFirmwareError(f"{code} {p['name']} must be int, got {value!r}")
        else:
          _check_value(code, p, value)
        named[p["arg"]] = value
        wire.append(value)

    self._check_rules(code, named)
    while wire and wire[-1] is None:
      wire.pop()
    if len(wire) > MAX_PARAMS:
      raise LihaFirmwareError(f"{code} has {len(wire)} parameters; the firmware parses at most 12")
    return wire

  def split_wire(self, code: str, params: Sequence[Optional[int]]) -> List[Any]:
    """The method values for wire parameters ``params`` (the inverse of :meth:`encode`): one
    int/None per scalar parameter, a list (or None) per per-tip group. Refuses leftovers."""
    spec = COMMANDS.get(code)
    if spec is None:
      raise LihaFirmwareError(f"LiHa command {code!r} is not available (unknown or excluded)")
    raw = list(params)
    values: List[Any] = []
    i = 0
    for p in [p for p in spec["params"] if not p["excluded"]]:
      if p["count"] > 1:
        n = min(p["count"], self.num_tips)
        chunk = raw[i : i + n]
        i += n
        values.append(chunk if any(v is not None for v in chunk) else None)
      else:
        values.append(raw[i] if i < len(raw) else None)
        i += 1
    if i < len(raw) and any(v is not None for v in raw[i:]):
      raise LihaFirmwareError(f"{code}: {len(raw) - i} parameters left over")
    return values

  def check_wire(self, code: str, params: Sequence[Optional[int]]) -> List[Optional[int]]:
    """Validate a command given as wire parameters; return what :meth:`encode` would send.
    Refuses parameters that would not go out unchanged."""
    wire = self.encode(code, *self.split_wire(code, params))
    trimmed = list(params)
    while trimmed and trimmed[-1] is None:
      trimmed.pop()
    if wire != trimmed:
      raise LihaFirmwareError(f"{code}{params}: would be sent as {wire}")
    return wire

  async def execute_wire(self, code: str, params: Sequence[Optional[int]]):
    """:meth:`execute` for a command given as wire parameters (a planner's ``(code, params)``)."""
    return await self.execute(code, *self.split_wire(code, params))

  def _check_rules(self, code: str, v: dict) -> None:
    """Cross-parameter rules stated in the spec text."""
    if code == "SDM":
      proc, sense = v.get("det_proc"), v.get("sense")
      if proc in PLLD_DETECTION_MODES:
        if "pmp" not in self.options:
          raise LihaOptionNotInstalled("SDM detection modes 10..16 need the PMP option")
        if sense == 0:
          raise LihaFirmwareError("SDM Sense 0 (standard) is not valid for pLLD modes")
    elif code in ("MDT", "MET"):
      zstart, zmax = v.get("zstart"), v.get("zmax")
      if zstart is not None and zmax is not None:
        if zmax > zstart:
          raise LihaFirmwareError(
            f"{code}: Zmax {zmax} is above Zstart {zstart} (search goes down)"
          )
        for add in v.get("zadd") or []:
          if add is not None and add > zstart - zmax:
            raise LihaFirmwareError(f"{code}: Zadd {add} exceeds Zstart - Zmax = {zstart - zmax}")
    elif code == "MCT":
      dist, limit = v.get("retract_dist"), v.get("limit")
      if dist is not None and limit is not None and not limit < dist:
        raise LihaFirmwareError(f"MCT: Limit {limit} must be below RetractDist {dist}")
    elif code == "AGT":
      if v.get("search_dist") is not None and v["search_dist"] > v["zstart"]:
        raise LihaFirmwareError("AGT: SearchDist must be within 0..Zstart")
    elif code == "APT":
      if v.get("zstart") is None or v.get("zmax") is None:
        raise LihaFirmwareError("APT: Zstart and Zmax must both be given, otherwise nothing moves")
    elif code == "MSD":
      dx, dy = v["x_delta_distance"], v["y_delta_distance"]
      if dx == 0 or dy == 0 or (dx > 0) != (dy > 0):
        raise LihaFirmwareError("MSD: X/YDeltaDistance must be non-zero with the same sign")

  # -- sending ----------------------------------------------------------------------------------

  async def execute(self, code: str, *values: Any):
    """Validate, then send ``code`` with its wire parameters; return the sender's reply unchanged.

    On a LiHa error the instrument is asked what happened, with read-only reports only, as EVOware
    does (AuditTrail logs), and the answer is attached to the raised :class:`TecanLihaError`:
    REE after liquid/move errors (``axis_errors``, ``tip_errors``), RTS after DiTi errors 25/26/27
    (``mounted_tips``), RDS after a subdevice error 28 (``dilutor_errors``). The original error is
    always re-raised; a failing follow-up report is logged and leaves its attribute None.
    """
    params = self.encode(code, *values)
    if COMMANDS[code]["untested"]:
      logger.warning("LiHa %s has never been tested on hardware", code)
    try:
      return await self._sender(module=self.module, command=code, params=params)
    except TecanLihaError as exc:
      if exc.module == self.module:
        await self._report_after_error(code, exc)
      raise

  async def _report_after_error(self, code: str, exc: TecanLihaError) -> None:
    report = (
      "REE"
      if exc.error_code in REPORT_AXES_AFTER
      else "RTS"
      if exc.error_code in REPORT_TIPS_AFTER
      else "RDS"
      if exc.error_code in REPORT_DILUTORS_AFTER
      else None
    )
    if report is None:
      return
    try:
      reply = await self._sender(module=self.module, command=report, params=[])
      data = reply["data"]
      if report == "REE":
        if len(data) != 1 or not isinstance(data[0], str):
          raise ValueError(f"unexpected REE reply {data!r}")
        exc.axis_errors = decode_extended_errors(data[0], LIHA_AXES[: 3 + self.num_tips])
      elif report == "RTS":
        if len(data) != 1 or not _is_int(data[0]):
          raise ValueError(f"unexpected RTS reply {data!r}")
        exc.mounted_tips = data[0]
      else:
        if not data or not all(_is_int(x) for x in data):
          raise ValueError(f"unexpected RDS reply {data!r}")
        exc.dilutor_errors = list(data)
    except Exception as follow_up:  # never hide the original error
      logger.warning(
        "LiHa %s after %s error %s failed: %r", report, code, exc.error_code, follow_up
      )

  async def report_subdevice(self, layer: int, device: int, report: str, *params: int):
    """T: Pipeline to Slave Device (spec p. 54), restricted to read-only subdevice reports.

    Sends ``T<layer><device><report><params>``, e.g. ``T26RPP7`` = RPP7 to dilutor 6 (layer 2).
    Only :data:`SUBDEVICE_REPORTS` (what EVOware sends) are allowed: the pass-through bypasses every
    check of this layer, so actions and set commands to subdevices are refused.

    Args:
      layer: 0..7 (EVOware writes it as a digit).
      device: 0..15 (hex digit on the wire).
      report: one of RFV, RPP, RSD, RIS.
      params: non-negative ints.
    """
    if not (_is_int(layer) and 0 <= layer <= 7 and _is_int(device) and 0 <= device <= 15):
      raise LihaFirmwareError("T: layer must be 0..7 and device 0..15")
    if report not in SUBDEVICE_REPORTS:
      raise LihaFirmwareError(
        f"T: only read-only subdevice reports {SUBDEVICE_REPORTS} are allowed"
      )
    if not all(_is_int(x) and x >= 0 for x in params) or len(params) > MAX_PARAMS:
      raise LihaFirmwareError("T: parameters must be up to 12 non-negative ints")
    command = f"T{layer}{device:X}{report}"
    return await self._sender(module=self.module, command=command, params=list(params))
