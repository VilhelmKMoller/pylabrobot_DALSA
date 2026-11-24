"""Tecan EVO LiHa (liquid handling arm) command planning.

Pure functions that turn EVOware configuration and parameters into LiHa (module
C5) firmware command sequences, reproducing what EVOware sends for the same
operation (run logs ``AuditTrail/log/EVO_*.LOG``, see ``test_data/liha_logs``).
Nothing here talks to hardware; ``EVOBackend`` executes the commands.

Positions follow from the instrument's EVOware configuration like the MCA's
(``mca.py``), with LiHa arm offsets 0 and the tip offset of ``[LihaMachine]``:
  X = 250 * (grid - 1) + site_x + labware_x1 - carrier_x
  Y = site_y + labware_y1 - carrier_y          (tip 1; spacing (y2 - y1) / (rows - 1))
  Z = z_range + carrier_z + TipOffset - labware_z
Verified for the wash station waste: X -18, Y 748, spacing 90, Z(travel) 690.

Firmware references (LiHa firmware manual 392736 V1.80): PAA (X, Y, Y spacing,
Z per tip), SHZ (Z travel height per tip), SSM (X smooth move mode), PVL (valve:
0 outlet = towards the tip, 1 inlet = system liquid), PPA/PPR (plunger, full
steps), SEP/SPP (plunger end/stop speed, half steps per second), RTS (mounted
DiTis, binary; 0 = none).
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from pylabrobot.legacy.liquid_handling.backends.tecan.evoware_config import (
  EvowareCarrier,
  EvowareConfig,
  EvowareLabware,
  EvowareSite,
)

Command = Tuple[str, List[Optional[int]]]

GRID_PITCH = 250  # 1/10 mm per grid


@dataclass(frozen=True)
class LihaMachine:
  """``evoware.inf [LihaMachine]`` values used for planning (first tip's value
  where EVOware stores one per tip; all tips must agree)."""

  tip_count: int
  tip_offset: int  # 1/10 mm
  dilutor_volume: int  # uL per full plunger stroke
  diluter_steps: int  # full steps per stroke
  wash_max_speed: int  # uL/s

  @property
  def steps_per_ul(self) -> float:
    return self.diluter_steps / self.dilutor_volume

  @classmethod
  def from_config(cls, config: EvowareConfig) -> "LihaMachine":
    try:
      values = config.sections["LihaMachine"]
    except KeyError:
      raise KeyError("evoware.inf has no [LihaMachine] section") from None

    def one(key: str) -> int:
      per_tip = {v.strip() for v in values[key].split(";") if v.strip() != ""}
      if len(per_tip) != 1:
        raise ValueError(f"[LihaMachine] {key} differs between tips: {values[key]}")
      return int(float(per_tip.pop()))

    return cls(
      tip_count=int(values["TipCount"]),
      tip_offset=one("TipOffset"),
      dilutor_volume=one("DilutorVolume"),
      diluter_steps=one("DiluterSteps"),
      wash_max_speed=one("WashMaxSpeed"),
    )


@dataclass(frozen=True)
class LihaSite:
  """An EVOware worktable position: carrier on ``grid`` (first grid = 1), site
  index ``site`` (0-based, as in EVOware's Wash command) and the labware there."""

  grid: int
  site: int
  carrier: str
  labware: str


# The instrument's wash station: carrier "DiTi Waste 3 Trough + Wash" at grid 1
# (flushLiHa.esc: Wash(255,1,1,1,0,...) = waste grid 1 site 1, cleaner grid 1 site 0).
WASH_WASTE = LihaSite(1, 1, "DiTi Waste 3 Trough + Wash", "Wash Station Waste")


# The DiTi waste chute of the wash carrier: DropDITI(255,1,6,...) in every LiHa script on this
# instrument = grid 1, site index 6 ("Washstation 2Grid DiTi Waste").
DITI_WASTE = LihaSite(1, 6, "DiTi Waste 3 Trough + Wash", "Washstation 2Grid DiTi Waste")

# Where EVOware puts the tips to drop DiTis into the chute (every "Drop DiTis" step in the logs:
# PAA126,2810,90,1075x8 then AST255,0). X follows the position formula; Y and Z do not: the chute
# labware lies beyond the Y range (Y 3125) and its heights would give Z 2926, above the Z range.
# EVOware uses Y 2810 (also the LiHa's home Y in the logs) and Z 1075 instead. Observed, not
# derived: verified against the logs only.
DITI_WASTE_Y = 2810
DITI_WASTE_Z = 1075
# EVOware ejects DiTis at this one Z everywhere: into the chute and when setting them back into a
# rack ("Set DiTis Back": all 252 error-free logged steps, 4 DiTi labware types on grids 10-46,
# fetch starts 700-783). It follows no labware height.
DITI_EJECT_Z = DITI_WASTE_Z

MIN_Y_SPACING, MAX_Y_SPACING = 90, 380  # PAA/PSY Ys range, LiHa manual p. 56-57
DEFAULT_Y_SPACING = 90

UNSUPPORTED_LABWARE = {
  "Trough 100ml": (
    "EVOware spreads the tips over this trough with Y spacing 180 around its centre and adds 10 mm"
    " to every Z (logs 2025-10-01, 2026-04-09); the rule behind that is not known"
  ),
}

HEIGHTS = ("z_travel", "z_start", "z_dispense", "z_max")


def _round(value: float) -> int:
  """Round half up (EVOware's rounding of fractional well pitches is not documented; no logged
  position needed a half)."""
  return int(value + 0.5) if value >= 0 else -int(-value + 0.5)


@dataclass(frozen=True)
class Position:
  """Arm position for PAA: X, Y of tip 1, and the Y spacing between tips (1/10 mm)."""

  x: int
  y: int
  y_spacing: int


@dataclass(frozen=True)
class DitiFetch:
  """Where and how deep to fetch DiTis: PAA position, then AGT Zstart and SearchDist."""

  position: Position
  z_start: int
  search_dist: int


class LihaGeometry:
  """LiHa positions from the EVOware configuration, as EVOware computes them.

  Verified against 490 logged LiHa steps (``test_data/liha_logs/geometry_cases.json``):
    X = 250 (grid - 1) + site_x + x1 - carrier_x + (column - 1) * column pitch
    Y(tip 1) = site_y + y1 - carrier_y + (row - 1) * row pitch - (tip - 1) * Y spacing
    Y spacing = row pitch (rounded) for 8-row labware, 90 for single-position labware
    Z = z_range + carrier_z - labware height + offset
  where the offset is ``[LihaMachine] TipOffset`` without DiTis (-500 here) and the mounted DiTi's
  length (``tip_length`` of its labware, 326 for "DiTi 1000ul LiHa") with DiTis. LiHa arm offsets
  are 0. A labware with one column but more positions than rows (field 8 of the record, e.g. 96 for
  "Trough 300ml MCA96") is addressed in positions/rows columns between x1 and x2.

  Positions only: no reachability check (X/Y/Z limits come from RPX/RPY/RPZ on the instrument), no
  collision check (``x_coordinator``).
  """

  def __init__(
    self,
    config: EvowareConfig,
    z_range: int,
    offsets: Optional[Dict[LihaSite, Tuple[int, int]]] = None,
    z_offsets: Optional[Dict[LihaSite, int]] = None,
  ):
    """``z_range``: the LiHa Z range (``RPZ5``; 2100 on this instrument).
    ``offsets``: operator-measured (dX, dY) corrections in 1/10 mm per worktable site, added to every
    tip position on that site (e.g. +10 X on the DiTi box at grid 46, 2026-11-23).
    ``z_offsets``: operator-measured Z corrections in 1/10 mm per site, added to every Z on that
    site (negative = lower; -10 = 1 mm lower)."""
    self.config = config
    self.machine = LihaMachine.from_config(config)
    self.z_range = z_range
    self.offsets = dict(offsets or {})
    self.z_offsets = dict(z_offsets or {})

  def _parts(self, where: LihaSite) -> Tuple[EvowareCarrier, EvowareSite, EvowareLabware]:
    if where.labware in UNSUPPORTED_LABWARE:
      raise NotImplementedError(f"{where.labware}: {UNSUPPORTED_LABWARE[where.labware]}")
    carrier = self.config.carrier(where.carrier)
    if not 0 <= where.site < len(carrier.sites):
      raise ValueError(f"{where.carrier} has no site index {where.site}")
    return carrier, carrier.sites[where.site], self.config.labware_def(where.labware)

  @staticmethod
  def columns(lw: EvowareLabware) -> int:
    """Addressable columns: the labware's columns, or positions/rows for a one-column trough
    with more positions than rows (EVOware addresses "Trough 300ml MCA96" in 12 columns)."""
    if lw.columns == 1 and lw.positions > lw.rows and lw.x2 != lw.x1:
      return lw.positions // lw.rows
    return lw.columns

  def _pitches(self, lw: EvowareLabware) -> Tuple[float, float]:
    n = self.columns(lw)
    col = (lw.x2 - lw.x1) / (n - 1) if n > 1 else 0.0
    row = (lw.y2 - lw.y1) / (lw.rows - 1) if lw.rows > 1 else 0.0
    return col, row

  def y_spacing(self, where: LihaSite, row_stride: int = 1) -> int:
    """Tip spacing EVOware uses on this labware: the row pitch (times ``row_stride`` when the tips
    go over every ``row_stride``-th row), or 90 for one-position labware."""
    _, _, lw = self._parts(where)
    _, row = self._pitches(lw)
    if row == 0:
      if row_stride != 1:
        raise ValueError(f"{where.labware} has one row; row_stride must be 1")
      return DEFAULT_Y_SPACING
    spacing = _round(row_stride * row)
    if not MIN_Y_SPACING <= spacing <= MAX_Y_SPACING:
      raise NotImplementedError(
        f"{where.labware}: row pitch {row:.1f} is outside the LiHa Y spacing {MIN_Y_SPACING}..{MAX_Y_SPACING};"
        " EVOware's tip-to-well assignment for that is not known"
      )
    return spacing

  def tips(
    self, where: LihaSite, column: int = 1, row: int = 1, tip: int = 1, row_stride: int = 1
  ) -> Position:
    """PAA position that puts ``tip`` (1-based) over well (``column``, ``row``) (1-based) and the
    following tips over the following rows, or over every ``row_stride``-th row (EVOware: tips 1-4
    over rows 2, 4, 6, 8 with PAA spacing 180 on a 9 mm plate)."""
    if row_stride < 1:
      raise ValueError("row_stride must be at least 1")
    carrier, site, lw = self._parts(where)
    n_cols = self.columns(lw)
    if not 1 <= column <= n_cols or not 1 <= row <= lw.rows:
      raise ValueError(f"{where.labware} has no well ({column}, {row}); {n_cols} x {lw.rows}")
    if not 1 <= tip <= self.machine.tip_count:
      raise ValueError(f"tip must be 1..{self.machine.tip_count}")
    col_pitch, row_pitch = self._pitches(lw)
    spacing = self.y_spacing(where, row_stride)
    x = GRID_PITCH * (where.grid - 1) + site.x + lw.x1 - carrier.x + (column - 1) * col_pitch
    y = site.y + lw.y1 - carrier.y + (row - 1) * row_pitch - (tip - 1) * spacing
    dx, dy = self.offsets.get(where, (0, 0))
    return Position(_round(x) + dx, _round(y) + dy, spacing)

  def xy(self, where: LihaSite) -> Position:
    """Tip 1 over the first well (as the wash uses it)."""
    return self.tips(where)

  def z(self, where: LihaSite, which: str, diti: Optional[str] = None) -> int:
    """Z for the labware height ``which`` (z_travel, z_start, z_dispense or z_max), with the DiTi
    labware ``diti`` mounted, or bare DiTi adapters if None."""
    carrier, _, lw = self._parts(where)
    if which not in HEIGHTS:
      raise ValueError(f"unknown labware height {which!r}")
    offset = self.machine.tip_offset if diti is None else self.config.labware_def(diti).tip_length
    height: int = getattr(lw, which)
    return self.z_range + carrier.z - height + offset + self.z_offsets.get(where, 0)

  def diti_fetch(self, rack: LihaSite, column: int = 1, row: int = 1, tip: int = 1) -> DitiFetch:
    """Fetching DiTis from ``rack`` (no DiTis mounted): AGT starts at the rack's z_start and searches
    down to its z_max (logged e.g. AGT255,768,210,0)."""
    _, _, lw = self._parts(rack)
    if lw.tip_length <= 0:
      raise ValueError(f"{rack.labware} is not a DiTi rack")
    return DitiFetch(
      self.tips(rack, column, row, tip),
      self.z(rack, "z_start"),
      lw.z_max - lw.z_start,
    )

  def diti_waste(self) -> Position:
    """Tip position for dropping DiTis into the chute (see DITI_WASTE_Y / DITI_WASTE_Z)."""
    carrier, site, lw = self._parts(DITI_WASTE)
    x = GRID_PITCH * (DITI_WASTE.grid - 1) + site.x + lw.x1 - carrier.x
    return Position(_round(x), DITI_WASTE_Y, DEFAULT_Y_SPACING)


@dataclass
class Plan:
  """Commands to send in order; ``WAIT`` entries are host delays in ms."""

  commands: List[Command] = field(default_factory=list)

  def add(self, command: str, *params: Optional[int]) -> None:
    self.commands.append((command, list(params)))

  def wait(self, ms: int) -> None:
    if ms > 0:
      self.add("WAIT", ms)


def plunger_speed(machine: LihaMachine, ul_per_s: float) -> int:
  """SEP/SPP value (half steps per second) for a flow in uL/s."""
  return round(ul_per_s * machine.steps_per_ul * 2)


def plan_wash(
  geo: LihaGeometry,
  waste: LihaSite = WASH_WASTE,
  waste_volume_ml: float = 12.0,
  waste_delay_ms: int = 50,
  air_gap_ul: float = 10,
  air_gap_speed: float = 70,
  channels: Sequence[int] = tuple(range(8)),
) -> Plan:
  """EVOware "Wash Tips" for bare DiTi adapters (flushLiHa): flush system liquid
  through the tips over the waste, then aspirate a system trailing air gap.

  For DiTi adapters EVOware moves to the waste at Z-travel and skips the cleaner
  (manual 393804 sec. 15.4), so the cleaner arguments do not exist here. The
  waste volume is per tip and is pumped in full dilutor strokes: valve to system
  liquid, aspirate one stroke, valve to the tip, dispense. Only whole strokes are
  accepted (all EVOware logs show 12 x 1 mL); a remainder would be a guess.
  The caller must make sure no DiTis are mounted (RTS 0).
  """
  m = geo.machine
  n = m.tip_count
  if not channels or any(not 0 <= c < n for c in channels):
    raise ValueError(f"channels must be within 0..{n - 1}")
  strokes = waste_volume_ml * 1000 / m.dilutor_volume
  if strokes <= 0 or abs(strokes - round(strokes)) > 1e-9:
    raise ValueError(
      f"waste volume must be a whole number of {m.dilutor_volume} uL strokes, got {waste_volume_ml} mL"
    )
  if not 0 <= air_gap_ul * m.steps_per_ul <= m.diluter_steps:
    raise ValueError("air gap outside the plunger travel")

  def each(value: int) -> List[Optional[int]]:
    return [value if c in channels else None for c in range(n)]

  pos = geo.xy(waste)
  z_travel = geo.z(waste, "z_travel")
  stroke = m.diluter_steps
  wash_speed = plunger_speed(m, m.wash_max_speed)

  plan = Plan()
  plan.add("SSM", 1)
  plan.add("SHZ", *[geo.z_range] * n)
  plan.add("PAA", pos.x, pos.y, pos.y_spacing, *[z_travel] * n)
  plan.add("SSM", 0)
  # empty the plunger towards the tips first
  plan.add("PVL", *each(0))
  plan.add("SEP", *each(wash_speed))
  plan.add("SPP", *each(wash_speed // 2))
  plan.add("PPA", *each(0))
  for _ in range(round(strokes)):
    plan.add("PVL", *each(1))
    plan.add("SEP", *each(wash_speed))
    plan.add("SPP", *each(wash_speed // 2))
    plan.add("PPR", *each(stroke))
    plan.add("PVL", *each(0))
    plan.add("PPA", *each(0))
  plan.wait(waste_delay_ms)  # valves stay open to relieve the tubing pressure
  plan.add("SHZ", *[geo.z_range] * n)
  plan.add("SEP", *each(plunger_speed(m, air_gap_speed)))
  plan.add("PPR", *each(round(air_gap_ul * m.steps_per_ul)))
  plan.add("RTS")
  return plan


def format_command(command: str, params: Sequence[Optional[int]]) -> str:
  """Wire text of a command, as ``TecanLiquidHandler._assemble_command`` builds it."""
  return command + ",".join("" if p is None else str(p) for p in params)


def normalize_log(commands: Sequence[str]) -> List[str]:
  """Reduce EVOware's C5 commands for comparison with a plan: drop position
  reads (RPX/RPY) and repeats of a command already sent before the first PAA or
  RTS (EVOware re-sends SSM1/SHZ while planning the move; a step without a move,
  such as an aspirate right after a Mix in the same wells, starts with RTS)."""
  out: List[str] = []
  moved = False
  for c in commands:
    c = c.rstrip(",")
    if c.startswith(("RPX", "RPY", "RPZ")):
      continue
    if not moved and c in out:
      continue
    moved = moved or c.startswith(("PAA", "RTS"))
    out.append(c)
  return out


def plan_strings(plan: Plan) -> List[str]:
  return [format_command(c, p) for c, p in plan.commands if c != "WAIT"]


# --- DiTi handling and pipetting ------------------------------------------------------------------
#
# Reproduce EVOware's LiHa command sequences for "Get DiTis", "Drop DiTis" (into the chute),
# "Aspirate" (liquid level detection, single mode) and "Dispense" (free or wet contact), as replayed
# against complete EVOware runs in liha_planner_tests.py. Values that EVOware reads back from the
# instrument during a step are inputs here: the liquid heights after MDT/MET (``detected``, RPZ0)
# and the plunger positions (``plunger``, RPP0). ``tips`` are 1-based channels in consecutive rows
# of one column (EVOware's normal case; anything else is refused).

# X move settings EVOware uses around DiTi fetching (logged SFX4999,550 ... SFX10000,1600).
DITI_X_MOVE = (4999, 550)
DEFAULT_X_MOVE = (10000, 1600)
# SDT before dropping DiTis: no aspiration while discarding, plunger speed 2700, delay 200 ms (logged).
DROP_SDT = (0, 2700, 200)
# DropDITI(...,10,70,...): the air gap taken into the bare adapters after dropping DiTis.
DROP_AIR_GAP_UL = 10.0
DROP_AIR_GAP_SPEED = 70.0


def tip_mask(tips: Sequence[int]) -> int:
  """TipSelect bit mask of 1-based channels."""
  return sum(1 << (t - 1) for t in set(tips))


def _check_tips(tips: Sequence[int], n: int, gaps: bool = False) -> List[int]:
  """Sorted 1-based tips. ``gaps``: tips may be left out (e.g. 1, 2, 3, 5); the others keep their
  places, tip t over row ``row + (t - first tip)`` (logged for DiTi steps only: 16 Pick Up DiTis and
  24 Set DiTis Back with mask 247 or 23, EVO_20260616_174054 and _175039)."""
  tips = sorted(tips)
  if not tips or tips[0] < 1 or tips[-1] > n or len(set(tips)) != len(tips):
    raise ValueError(f"tips must be distinct and within 1..{n}")
  if not gaps and tips != list(range(tips[0], tips[0] + len(tips))):
    raise NotImplementedError(
      "only consecutive tips (one column, consecutive rows) are supported for this step"
    )
  return tips


def _each(n: int, tips: Sequence[int], value, other=None) -> List[Optional[int]]:
  return [value if c + 1 in tips else other for c in range(n)]


def _per_tip(n: int, tips: Sequence[int], values: Sequence[int], other=None) -> List[Optional[int]]:
  return [values[c] if c + 1 in tips else other for c in range(n)]


def _move(plan: Plan, geo: LihaGeometry, pos: Position, z: List[Optional[int]], ssm_first: bool):
  """SHZ to the Z range, then PAA with X smooth move on, as EVOware does before every LiHa move."""
  n = geo.machine.tip_count
  if ssm_first:
    plan.add("SSM", 1)
  plan.add("SHZ", *[geo.z_range] * n)
  plan.add("SSM", 1)
  plan.add("PAA", pos.x, pos.y, pos.y_spacing, *z)
  plan.add("SSM", 0)


def _require(lc, what: str, value, allowed: Sequence, planner: str) -> None:
  """Refuse a liquid-class position this planner does not reproduce."""
  if value not in allowed:
    raise NotImplementedError(
      f"{lc.name} ({lc.tip_type} {lc.volume_range[0]}-{lc.volume_range[1]}): {what} {value!r} is"
      f" not supported by {planner} (supported: {', '.join(map(repr, allowed))})"
    )


def _ceil(value: float) -> int:
  return int(-(-value // 1))


def plan_get_ditis(
  geo: LihaGeometry,
  rack: LihaSite,
  column: int,
  row: int,
  tips: Sequence[int],
  slow_x: bool = True,
) -> Plan:
  """EVOware "Get DiTis": tips over the rack at the Z range, then AGT (logged: SSM1, SFX4999,550,
  SHZ, PAA x,y,90,2100x8, SSM0, SFX10000,1600, AGT255,768,210,0, RTS). The caller checks the RTS
  reply against the tips it asked for.

  ``slow_x``: move X with the slow settings (SFX4999,550, restored to SFX10000,1600 after the move).
  EVOware did so in 44 of 45 logged fetches in sessions where it polled the RoMa, and never
  otherwise; this instrument has a RoMa, so it is the default."""
  n = geo.machine.tip_count
  tips = _check_tips(tips, n, gaps=True)
  fetch = geo.diti_fetch(rack, column, row, tips[0])
  plan = Plan()
  plan.add("SSM", 1)
  if slow_x:
    plan.add("SFX", *DITI_X_MOVE)
  plan.add("SHZ", *[geo.z_range] * n)
  plan.add("SSM", 1)
  plan.add("PAA", fetch.position.x, fetch.position.y, fetch.position.y_spacing, *[geo.z_range] * n)
  plan.add("SSM", 0)
  if slow_x:
    plan.add("SFX", *DEFAULT_X_MOVE)
  plan.add("AGT", tip_mask(tips), fetch.z_start, fetch.search_dist, 0)
  plan.add("RTS")
  return plan


def plan_set_ditis_back(
  geo: LihaGeometry,
  rack: LihaSite,
  column: int,
  row: int,
  tips: Sequence[int],
  slow_x: bool = True,
) -> Plan:
  """EVOware "Set DiTis Back": the mounted DiTis back into a rack (logged: SSM1, [SFX4999,550,] SHZ,
  SSM1, PAA x,y,ys,1075 on the tips, SSM0, [SFX10000,1600,] RTS, AST mask,0, RTS).

  X, Y and spacing are those of fetching from that rack position (``diti_fetch``; all 252
  error-free logged steps), Z is ``DITI_EJECT_Z``. The plungers are not touched. ``slow_x`` as in
  ``plan_get_ditis`` (EVOware: 75 of 254 steps, the sessions that polled the RoMa). The caller checks
  the last RTS: the tips must be gone. Pick the DiTis up again with ``plan_get_ditis`` (EVOware's
  "Pick Up DiTis" sends the same commands: 255 of 257 logged steps)."""
  n = geo.machine.tip_count
  tips = _check_tips(tips, n, gaps=True)
  pos = geo.diti_fetch(rack, column, row, tips[0]).position
  plan = Plan()
  plan.add("SSM", 1)
  if slow_x:
    plan.add("SFX", *DITI_X_MOVE)
  plan.add("SHZ", *[geo.z_range] * n)
  plan.add("SSM", 1)
  plan.add("PAA", pos.x, pos.y, pos.y_spacing, *_each(n, tips, DITI_EJECT_Z, geo.z_range))
  plan.add("SSM", 0)
  if slow_x:
    plan.add("SFX", *DEFAULT_X_MOVE)
  plan.add("RTS")
  plan.add("AST", tip_mask(tips), 0)
  plan.add("RTS")
  return plan


def plan_drop_ditis(
  geo: LihaGeometry,
  tips: Sequence[int],
  air_gap_ul: float = DROP_AIR_GAP_UL,
  air_gap_speed: float = DROP_AIR_GAP_SPEED,
) -> Plan:
  """EVOware "Drop DiTis" into the chute (DropDITI(mask,1,6,10,70,0)): tips to the chute, plungers
  emptied at wash speed, AST, then an air gap into the bare adapters.

  ``air_gap_ul`` / ``air_gap_speed``: DropDITI's air gap (10 uL at 70 uL/s in every script here).
  EVOware usually sends exactly that (SEP420, PPR30), but in a loop of identical steps it sometimes
  used 30 uL/s (SEP180) instead; the rule is unknown (EVO_20260409_115221, drops 35 vs 39)."""
  m, n = geo.machine, geo.machine.tip_count
  tips = _check_tips(tips, n)
  pos = geo.diti_waste()
  wash = plunger_speed(m, m.wash_max_speed)
  plan = Plan()
  _move(plan, geo, pos, _each(n, tips, DITI_WASTE_Z, geo.z_range), ssm_first=True)
  plan.add("PVL", *_each(n, tips, 0))
  plan.add("SEP", *_each(n, tips, wash))
  plan.add("SPP", *_each(n, tips, wash // 2))
  plan.add("PPA", *_each(n, tips, 0))
  plan.add("SDT", *DROP_SDT)
  plan.add("RTS")
  plan.add("AST", tip_mask(tips), 0)
  plan.add("RTS")
  plan.add("SEP", *_each(n, tips, plunger_speed(m, air_gap_speed)))
  plan.add("PPR", *_each(n, tips, round(air_gap_ul * m.steps_per_ul)))
  return plan


def tracking(volume_ul: float, area_mm2: float) -> int:
  """Z tracking distance (1/10 mm) for ``volume_ul`` in labware of cross-section ``area_mm2``,
  rounded up (EVOware: STZ -1 for 156 uL in the 300 ml trough, -48 for 157 uL in a plate well).
  In plates EVOware's value is sometimes 1 lower (its well-shape model is not reproduced)."""
  return _ceil(10 * volume_ul / area_mm2) if area_mm2 > 0 else 0


def tracking_speed(distance: float, steps: int, plunger_speed_half_steps: int) -> int:
  """SSZ (1/10 mm/s) so the Z tracking lasts as long as the plunger move, rounded up."""
  if distance == 0 or steps == 0:
    return 1
  seconds = abs(steps) / (plunger_speed_half_steps / 2)
  return max(1, _ceil(abs(distance) / seconds))


def _lc_values(lc, geo: LihaGeometry):
  from pylabrobot.legacy.liquid_handling.backends.tecan.evoware_liquid_classes import (
    LihaFirmwareValues,
  )

  return LihaFirmwareValues(lc, geo.machine.steps_per_ul)


@dataclass
class AspiratePlan:
  """An aspiration in two parts: ``search`` up to and including the liquid detection (MDT); then
  ``finish(detected)`` with the Z positions the instrument reports after MDT (RPZ0)."""

  search: Plan
  finish_args: dict

  def finish(self, detected: Sequence[int]) -> Plan:
    return _aspirate_finish(detected=detected, **self.finish_args)


def plan_aspirate(
  geo: LihaGeometry,
  lc,
  site: LihaSite,
  column: int,
  row: int,
  tips: Sequence[int],
  volume: float,
  diti: str,
  plunger: Sequence[int],
  after_mix_z: Optional[int] = None,
  multi: bool = False,
) -> AspiratePlan:
  """EVOware "Aspirate" with liquid level detection, single mode, ``volume`` uL per tip.

  ``multi``: multi-pipetting mode (the class's Multi sub-class), for dispensing ``volume`` in
  portions (``plan_dispense(multi_volume=...)``). As logged (EVO_20251105_141010/141257, worklist,
  "Water free dispense" 15-200 uL): the tips take ``volume`` + excess + conditioning, and after the
  retract dispense the conditioning back into the source (dispense and breakoff speed); the excess
  stays in the tip until the DiTi is dropped. Only classes whose Multi sub-class has no air gaps
  and no calibration are verified.

  ``lc``: the ``LihaLiquidClass`` for ``volume``; ``diti``: the mounted DiTi labware; ``plunger``:
  current plunger positions (steps) of all channels = the air already in the tips. Before the leading
  air gap EVOware tops this up to the sub-class's system trailing air gap (STAG): logged PPR45 =
  (20 - 10) + 5 uL with 10 uL already in the tip, PPR75 = 20 + 5 with an empty tip, PPR15 = 5 with
  20 uL in the tip.

  ``after_mix_z``: the same tips just mixed in the same wells (``plan_mix``) and stand there at this Z.
  EVOware then neither moves nor takes an air gap (the mix's leading air gap is still in the tip),
  and returns to this Z instead of the Z range after the detection (logged, 9 of 9 Mix steps of
  EVO_20260409_111830).
  """
  n = geo.machine.tip_count
  tips = _check_tips(tips, n)
  if len(plunger) != n:
    raise ValueError(f"plunger must have {n} values")
  _require(lc, "aspirate position", lc.aspirate_position, ("liquid_level",), "plan_aspirate")
  _require(
    lc,
    "aspirate retract position",
    lc.aspirate_retract_position,
    ("liquid_level",),
    "plan_aspirate",
  )
  fw = _lc_values(lc, geo)
  mode = lc.multi if multi else lc.single
  if multi:
    _require_plain_multi(lc)
    if lc.aspirate_mix is not None or after_mix_z is not None:
      raise NotImplementedError(
        f"{lc.name}: mixing before a multi-pipetting aspirate is not verified"
      )
  pos = geo.tips(site, column, row, tips[0])
  _, _, lw = geo._parts(site)
  z_travel = geo.z(site, "z_travel", diti)
  if multi:
    calibrated = volume + mode.excess + mode.conditioning
    steps = fw.steps(calibrated)
  else:
    calibrated = mode.calibrated(volume)
    steps = fw.aspirate_steps(volume)
  lead = [lead_air_steps(fw, mode, plunger[c]) for c in range(n)]
  depth = tracking(calibrated, lw.area)

  plan = Plan()
  if after_mix_z is None:
    _move(plan, geo, pos, _each(n, tips, z_travel, geo.z_range), ssm_first=False)
    plan.add("RTS")
    plan.add("PVL", *_each(n, tips, 0))
    plan.add("SEP", *_each(n, tips, fw.speed(mode.lead_speed)))
    plan.add("PPR", *_per_tip(n, tips, lead))
    z_after: List[Optional[int]] = [geo.z_range] * n
  else:
    plan.add("SSM", 1)
    plan.add("SSM", 0)
    plan.add("SHZ", *_each(n, tips, after_mix_z, geo.z_range))
    plan.add("RTS")
    z_after = _each(n, tips, after_mix_z, geo.z_range)
  plan.add("SDM", *fw.detection_mode)
  plan.add("SSL", *_each(n, tips, fw.search_speed))
  plan.add("SDL", *_each(n, tips, fw.retract_distance))
  plan.add("STL", *_each(n, tips, geo.z(site, "z_start", diti)))
  plan.add("SML", *_each(n, tips, geo.z(site, "z_max", diti), geo.z_range))
  plan.add("SBL", *_each(n, tips, fw.submerge()))
  plan.add("SHZ", *_each(n, tips, z_travel, geo.z_range))
  plan.add("MDT", tip_mask(tips), None, None, None, *_each(n, tips, depth, 0))
  plan.add("SHZ", *z_after)
  return AspiratePlan(
    plan,
    dict(geo=geo, fw=fw, tips=tips, steps=steps, depth=depth, mode=mode, multi=multi),
  )


def _require_plain_multi(lc) -> None:
  m = lc.multi
  if m.lead_air_gap or m.trail_air_gap or (m.calibration_factor, m.calibration_offset) != (1, 0):
    raise NotImplementedError(
      f"{lc.name}: multi-pipetting with air gaps or a calibration is not verified (logged only"
      " without: LAG 0, TAG 0, factor 1, offset 0)"
    )


def _aspirate_finish(
  geo, fw, tips, steps, depth, mode, detected: Sequence[int], multi: bool = False
) -> Plan:
  n = geo.machine.tip_count
  if len(detected) != n:
    raise ValueError(f"detected must have {n} values")
  speed = fw.speed(mode.aspirate_speed)
  plan = Plan()
  if fw.lc.aspirate_mix is not None:
    # the class's Aspirate/Mix: RTS, then the strokes where the tips detected (uncalibrated, no
    # tracking), RTS (EVO_20260409_115221 steps 33/37/41: 3 x 100 uL -> MTR+-300). EVOware's MDT
    # Zadd was 55 there instead of the plain aspirate's 48 (rule unknown from one class/volume;
    # Zadd only sets how deep the liquid must be for MDT to accept the surface).
    volume, cycles = fw.lc.aspirate_mix
    mix_steps = fw.steps(volume)
    plan.add("RTS")
    for _ in range(cycles):
      plan.add("SEP", *_each(n, tips, speed))
      plan.add("STZ", *_each(n, tips, 0))
      plan.add("MTR", *_each(n, tips, mix_steps))
      plan.wait(mode.aspirate_delay_ms)
      plan.add("SEP", *_each(n, tips, fw.speed(mode.dispense_speed)))
      plan.add("SPP", *_each(n, tips, fw.speed(mode.breakoff_speed)))
      plan.add("STZ", *_each(n, tips, 0))
      plan.add("MTR", *_each(n, tips, -mix_steps))
      plan.wait(mode.dispense_delay_ms)
    plan.add("RTS")
  plan.add("SSZ", *_each(n, tips, tracking_speed(depth, steps, speed)))
  plan.add("SEP", *_each(n, tips, speed))
  plan.add("STZ", *_each(n, tips, -depth))
  plan.add("MTR", *_each(n, tips, steps))
  plan.wait(mode.aspirate_delay_ms)
  plan.add("SSZ", *_each(n, tips, fw.retract_speed()))
  above = fw.submerge() + fw.retract_z_above_surface()
  plan.add("MAZ", *_per_tip(n, tips, [z + above for z in detected]))
  if multi:
    # the conditioning volume back into the source, above it; no trailing air gap (TAG 0)
    plan.add("PVL", *_each(n, tips, 0))
    plan.add("SEP", *_each(n, tips, fw.speed(mode.dispense_speed)))
    plan.add("SPP", *_each(n, tips, fw.speed(mode.breakoff_speed)))
    plan.add("PPR", *_each(n, tips, -fw.steps(mode.conditioning)))
    plan.add("RTS")
    return plan
  plan.add("PVL", *_each(n, tips, 0))
  plan.add("SEP", *_each(n, tips, fw.speed(mode.trail_speed)))
  plan.add("PPR", *_each(n, tips, fw.steps(mode.trail_air_gap)))
  plan.add("RTS")
  return plan


def aspirated_steps(
  lc,
  geo: LihaGeometry,
  volume: float,
  plunger_before: int,
  after_mix: bool = False,
  multi: bool = False,
) -> int:
  """Plunger position of one tip after ``plan_aspirate``: top-up + LAG + liquid + TAG (no top-up
  and LAG right after a Mix in the same wells). ``multi``: top-up + volume + excess (the
  conditioning went back)."""
  fw = _lc_values(lc, geo)
  if multi:
    mm = lc.multi
    return int(
      plunger_before + lead_air_steps(fw, mm, plunger_before) + fw.steps(volume + mm.excess)
    )
  m = lc.single
  air = 0 if after_mix else lead_air_steps(fw, m, plunger_before)
  return int(plunger_before + air + fw.aspirate_steps(volume) + fw.steps(m.trail_air_gap))


def lead_air_steps(fw, mode, plunger_before: int) -> int:
  """Air EVOware takes before aspirating or mixing: the top-up to the STAG plus the LAG (steps)."""
  stag = fw.steps(mode.system_trail_air_gap)
  return int(max(0, stag - plunger_before) + fw.steps(mode.lead_air_gap))


@dataclass
class DispensePlan:
  """A dispense: ``first`` up to the move (free dispense: the whole dispense), and for wet contact
  ``finish(detected)`` after the liquid detection (MET, RPZ0)."""

  first: Plan
  finish_args: Optional[dict] = None

  def finish(self, detected: Sequence[int]) -> Plan:
    if self.finish_args is None:
      return Plan()
    return _dispense_finish(detected=detected, **self.finish_args)


def dispense_steps(lc, geo: LihaGeometry, plunger: Sequence[int], tips: Sequence[int]) -> List[int]:
  """Steps EVOware dispenses per tip: everything above the system trailing air gap (logged: RPP0 573
  -> MTR-513 with STAG 20 uL = 60 steps; RPP0 140 -> MTR-80)."""
  fw = _lc_values(lc, geo)
  stag = fw.steps(lc.single.system_trail_air_gap)
  out = []
  for c in range(geo.machine.tip_count):
    if c + 1 in tips:
      if plunger[c] <= stag:
        raise ValueError(
          f"tip {c + 1} holds nothing to dispense (plunger {plunger[c]}, STAG {stag})"
        )
      out.append(plunger[c] - stag)
    else:
      out.append(0)
  return out


def plan_dispense(
  geo: LihaGeometry,
  lc,
  site: LihaSite,
  column: int,
  row: int,
  tips: Sequence[int],
  diti: str,
  plunger: Sequence[int],
  row_stride: int = 1,
  multi_volume: Optional[float] = None,
) -> DispensePlan:
  """EVOware "Dispense", single mode: free dispense at z_dispense, or wet contact (liquid detection,
  MET, then tracking up). Dispenses everything above the STAG (``dispense_steps``); ``plunger`` is the
  RPP0 reading EVOware takes before dispensing. ``row_stride``: the tips go over every
  ``row_stride``-th row (``LihaGeometry.tips``; logged: 39 dispenses with tips 1-4 over rows
  2, 4, 6, 8, checker pattern).

  ``multi_volume``: a portion of a multi-pipetting aspirate (``plan_aspirate(multi=True)``): that
  many uL per tip, uncalibrated, free dispense only, the rest stays in the tip (logged: 50 uL ->
  MTR-150, otherwise as the single free dispense)."""
  n = geo.machine.tip_count
  tips = _check_tips(tips, n)
  if len(plunger) != n:
    raise ValueError(f"plunger must have {n} values")
  _require(
    lc,
    "dispense position",
    (lc.dispense_lld, lc.dispense_position),
    ((False, "z_dispense"), (True, "liquid_level")),
    "plan_dispense",
  )
  _require(
    lc, "dispense retract position", lc.dispense_retract_position, ("z_dispense",), "plan_dispense"
  )
  fw = _lc_values(lc, geo)
  m = lc.multi if multi_volume is not None else lc.single
  pos = geo.tips(site, column, row, tips[0], row_stride)
  _, _, lw = geo._parts(site)
  if multi_volume is not None:
    _require_plain_multi(lc)
    if lc.dispense_lld:
      raise NotImplementedError(
        f"{lc.name}: a multi-pipetting wet-contact dispense is not verified"
      )
    if not multi_volume > 0:
      raise ValueError("multi_volume must be positive")
    portion = fw.steps(multi_volume)
    stag = fw.steps(m.system_trail_air_gap)
    for t in tips:
      if plunger[t - 1] - portion < stag:
        raise ValueError(f"tip {t} holds less than {multi_volume} uL (plunger {plunger[t - 1]})")
    steps: List[int] = [portion if c + 1 in tips else 0 for c in range(n)]
  else:
    steps = dispense_steps(lc, geo, plunger, tips)
  speed, breakoff = fw.speed(m.dispense_speed), fw.speed(m.breakoff_speed)
  plan = Plan()
  if not lc.dispense_lld:
    z = geo.z(site, "z_dispense", diti)
    _move(plan, geo, pos, _each(n, tips, z, geo.z_range), ssm_first=False)
    plan.add("RTS")
    plan.add("SEP", *_each(n, tips, speed))
    plan.add("SPP", *_each(n, tips, breakoff))
    plan.add("STZ", *_each(n, tips, 0))
    plan.add("MTR", *_per_tip(n, tips, [-s for s in steps]))
    plan.wait(m.dispense_delay_ms)
    plan.add("RTS")
    return DispensePlan(plan)
  z_start = geo.z(site, "z_start", diti)
  _move(plan, geo, pos, _each(n, tips, z_start, geo.z_range), ssm_first=False)
  plan.add("RTS")
  plan.add("SDM", *fw.detection_mode)
  plan.add("SSL", *_each(n, tips, fw.search_speed))
  plan.add("SDL", *_each(n, tips, fw.retract_distance))
  plan.add("STL", *_each(n, tips, z_start))
  plan.add("SML", *_each(n, tips, geo.z(site, "z_max", diti), geo.z_range))
  plan.add("SBL", *_each(n, tips, fw.submerge(dispense=True)))
  plan.add("MET", tip_mask(tips), None, None, None, *[0] * n)
  return DispensePlan(
    plan,
    dict(
      geo=geo,
      fw=fw,
      tips=tips,
      steps=steps,
      speed=speed,
      breakoff=breakoff,
      area=lw.area,
      mode=m,
      z_dispense=geo.z(site, "z_dispense", diti),
    ),
  )


def _dispense_finish(
  geo, fw, tips, steps, speed, breakoff, area, mode, z_dispense, detected
) -> Plan:
  n = geo.machine.tip_count
  if len(detected) != n:
    raise ValueError(f"detected must have {n} values")
  rise = [tracking(s / geo.machine.steps_per_ul, area) for s in steps]
  plan = Plan()
  plan.add("SSZ", *_per_tip(n, tips, [tracking_speed(r, s, speed) for r, s in zip(rise, steps)]))
  plan.add("SEP", *_each(n, tips, speed))
  plan.add("SPP", *_each(n, tips, breakoff))
  plan.add("STZ", *_per_tip(n, tips, rise))
  plan.add("MTR", *_per_tip(n, tips, [-s for s in steps]))
  plan.wait(mode.dispense_delay_ms)
  plan.add("SSZ", *_each(n, tips, fw.retract_speed(dispense=True)))
  plan.add("MAZ", *_each(n, tips, z_dispense))
  plan.add("RTS")
  return plan


@dataclass
class MixPlan:
  """A mix in two parts: ``search`` up to and including the liquid detection (MET); then
  ``finish(detected)`` with the Z positions the instrument reports after MET (RPZ0). EVOware reads
  them; with the mix at z_max the strokes do not depend on them."""

  search: Plan
  finish_args: dict

  def finish(self, detected: Sequence[int]) -> Plan:
    return _mix_finish(detected=detected, **self.finish_args)

  @property
  def retract_z(self) -> int:
    """Z of the tips after the mix (Dispense/Retract position)."""
    return int(self.finish_args["z_retract"])


def plan_mix(
  geo: LihaGeometry,
  lc,
  site: LihaSite,
  column: int,
  row: int,
  tips: Sequence[int],
  volume: float,
  cycles: int,
  diti: str,
  plunger: Sequence[int],
) -> MixPlan:
  """EVOware "Mix", single mode: ``cycles`` times aspirate and dispense ``volume`` uL per tip.

  ``lc``: the ``LihaLiquidClass`` for ``volume``; ``diti``: the mounted DiTi labware; ``plunger``:
  current plunger positions (steps) of all channels. As logged (EVO_20260409_111830, 9 steps with
  ">> Water Mix Gentle <<\\t11"):

  - move at z_travel, then the leading air gap as before an aspirate (STAG top-up + LAG; no PVL0);
  - liquid level detection with MET (Zadd 0), then PAZ to the labware's z_max (Aspirate/LLD
    position 0);
  - each cycle: aspirate ``volume`` *uncalibrated* (150 uL -> MTR450) at the aspirate speed, then
    dispense it at the dispense and breakoff speeds; STZ0 (no tracking);
  - retract with Dispense/Retract: its speed (SSZ) to z_start or z_dispense (MAZ).

  The class delays after each stroke are host waits here. The log does not show them, but the step
  time fits (about 9 s for 3 cycles of 150 uL, 6.75 s of plunger moves). The mix leaves the
  leading air gap in the tips: plunger after = before + ``lead_air_steps``.

  Mixing at the liquid level (Aspirate/LLD position "liquid_level"; logged 12 times with
  "Water free dispense", 30 uL, 1 cycle, single tip in a 96 Well DeepWell, EVO_20260205_140608 and
  _141912): the tips stay where MET left them (no PAZ), MET Zadd = submerge + tracking, and each
  stroke tracks: SSZ, SEP, STZ -d, MTR +V; SSZ, SEP, SPP, STZ +d, MTR -V. The tracking d is
  round(volume / area) (uncalibrated volume, as the stroke), and 0 for a tip that would track below
  z_max (then no SSZ is sent); SSZ = ``tracking_speed`` of the unrounded distance. All logged
  values agree, except the dispense SSZ for a surface at 1020 (EVOware 185, ours 181; at 1021 both
  are 181): EVOware's well-shape model, not reproduced (2 %).
  """
  n = geo.machine.tip_count
  tips = _check_tips(tips, n)
  if len(plunger) != n:
    raise ValueError(f"plunger must have {n} values")
  if cycles < 1:
    raise ValueError("cycles must be at least 1")
  if volume <= 0:
    raise ValueError("volume must be positive")
  _require(lc, "aspirate position", lc.aspirate_position, ("z_max", "liquid_level"), "plan_mix")
  _require(
    lc,
    "dispense retract position",
    lc.dispense_retract_position,
    ("z_start", "z_dispense"),
    "plan_mix",
  )
  # at the liquid level the aspirate LLD offset is the submerge (SBL), as for an aspirate
  checks = [("dispense retract offset", lc.dispense_retract_offset)]
  if lc.aspirate_position == "z_max":
    checks.append(("aspirate LLD offset", lc.aspirate_offset))
  for what, value in checks:
    if value != 0:
      raise NotImplementedError(f"{lc.name}: a {what} ({value} mm) is not verified for Mix")
  fw = _lc_values(lc, geo)
  mode = lc.single
  pos = geo.tips(site, column, row, tips[0])
  z_max = geo.z(site, "z_max", diti)
  at_level = lc.aspirate_position == "liquid_level"
  _, _, lw = geo._parts(site)
  distance = 10 * volume / lw.area if at_level and lw.area > 0 else 0.0  # 1/10 mm, unrounded
  depth = round(distance)
  zadd = fw.submerge() + depth if at_level else 0

  plan = Plan()
  _move(plan, geo, pos, _each(n, tips, geo.z(site, "z_travel", diti), geo.z_range), ssm_first=False)
  plan.add("RTS")
  plan.add("SEP", *_each(n, tips, fw.speed(mode.lead_speed)))
  plan.add("PPR", *_per_tip(n, tips, [lead_air_steps(fw, mode, plunger[c]) for c in range(n)]))
  plan.add("SDM", *fw.detection_mode)
  plan.add("SSL", *_each(n, tips, fw.search_speed))
  plan.add("SDL", *_each(n, tips, fw.retract_distance))
  plan.add("STL", *_each(n, tips, geo.z(site, "z_start", diti)))
  plan.add("SML", *_each(n, tips, z_max, geo.z_range))
  plan.add("SBL", *_each(n, tips, fw.submerge()))
  plan.add("MET", tip_mask(tips), None, None, None, *_each(n, tips, zadd, 0))
  return MixPlan(
    plan,
    dict(
      geo=geo,
      fw=fw,
      tips=tips,
      steps=fw.steps(volume),
      cycles=cycles,
      mode=mode,
      z_max=z_max,
      at_level=at_level,
      depth=depth,
      distance=distance,
      z_retract=geo.z(site, lc.dispense_retract_position, diti),
    ),
  )


def _mix_finish(
  geo, fw, tips, steps, cycles, mode, z_max, at_level, depth, distance, z_retract, detected
) -> Plan:
  n = geo.machine.tip_count
  if len(detected) != n:
    raise ValueError(f"detected must have {n} values")
  plan = Plan()
  if not at_level:
    plan.add("PAZ", *_each(n, tips, z_max))
  # per tip: track only where the tip stays above z_max
  d = [depth if c + 1 in tips and detected[c] - depth >= z_max else 0 for c in range(n)]
  tracked = [t for t in tips if d[t - 1]]
  asp, disp = fw.speed(mode.aspirate_speed), fw.speed(mode.dispense_speed)

  def ssz(speed: int):
    if tracked:
      plan.add("SSZ", *_each(n, tracked, tracking_speed(distance, steps, speed)))

  for _ in range(cycles):
    ssz(asp)
    plan.add("SEP", *_each(n, tips, asp))
    plan.add("STZ", *_per_tip(n, tips, [-v for v in d]))
    plan.add("MTR", *_each(n, tips, steps))
    plan.wait(mode.aspirate_delay_ms)
    ssz(disp)
    plan.add("SEP", *_each(n, tips, disp))
    plan.add("SPP", *_each(n, tips, fw.speed(mode.breakoff_speed)))
    plan.add("STZ", *_per_tip(n, tips, d))
    plan.add("MTR", *_each(n, tips, -steps))
    plan.wait(mode.dispense_delay_ms)
  plan.add("SSZ", *_each(n, tips, fw.retract_speed(dispense=True)))
  plan.add("MAZ", *_each(n, tips, z_retract))
  plan.add("RTS")
  return plan


def mixed_steps(lc, geo: LihaGeometry, plunger_before: int) -> int:
  """Plunger position of one tip after ``plan_mix``: the leading air gap stays in the tip."""
  return plunger_before + lead_air_steps(_lc_values(lc, geo), lc.single, plunger_before)


def plan_detect_liquid(
  geo: LihaGeometry,
  lc,
  site: LihaSite,
  column: int,
  row: int,
  tips: Sequence[int],
  diti: str,
) -> Tuple[Plan, Plan]:
  """Liquid level detection only: (search, retract). Read the surface with RPZ0 between the two.

  NOT a reproduction of EVOware's "Detect Liquid" command: that was never logged on this instrument.
  This is the verified search part of ``plan_aspirate`` (move at z_travel, the liquid class's
  detection settings, MDT with the Z range as travel height after it) without the air gap and the
  plunger, with no submerge (SBL0: the tips stop at the surface) and no tracking (Zadd 0); then the
  tips go straight up to z_travel. The liquid class must detect at the liquid level."""
  n = geo.machine.tip_count
  tips = _check_tips(tips, n)
  _require(lc, "aspirate position", lc.aspirate_position, ("liquid_level",), "plan_detect_liquid")
  fw = _lc_values(lc, geo)
  pos = geo.tips(site, column, row, tips[0])
  z_travel = geo.z(site, "z_travel", diti)
  search = Plan()
  _move(search, geo, pos, _each(n, tips, z_travel, geo.z_range), ssm_first=False)
  search.add("RTS")
  search.add("SDM", *fw.detection_mode)
  search.add("SSL", *_each(n, tips, fw.search_speed))
  search.add("SDL", *_each(n, tips, fw.retract_distance))
  search.add("STL", *_each(n, tips, geo.z(site, "z_start", diti)))
  search.add("SML", *_each(n, tips, geo.z(site, "z_max", diti), geo.z_range))
  search.add("SBL", *_each(n, tips, 0))
  search.add("SHZ", *_each(n, tips, z_travel, geo.z_range))
  search.add("MDT", tip_mask(tips), None, None, None, *_each(n, tips, 0, 0))
  search.add("SHZ", *[geo.z_range] * n)
  retract = Plan()
  retract.add("PAZ", *_each(n, tips, z_travel))
  retract.add("RTS")
  return search, retract
