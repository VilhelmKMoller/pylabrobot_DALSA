"""Tecan EVO MCA96 (multi channel arm) command planning.

Pure functions that turn PLR resources and parameters into MCA firmware command
sequences. The sequences reproduce what EVOware sends for the same operation
(recorded USB captures, see ``test_data/mca_captures``). Nothing here talks to
hardware; ``EVOBackend`` executes the commands.

Firmware references (MCA firmware manual 394562 V1.0): PAA p.35 (omitted axis =
no move), PPA/PPR p.37 (plunger, 0.0005 mm/step), ATF p.38 (tip fetch: Zstart,
Zend, Zretract, TipLength, PWM limit), ATD p.39 (tip drop), SLP p.30 (plunger
start ramp, speed, stop ramp), SPA p.27 (axis parameters, selector 2 = fast
speed). ``T40RDP4`` (pipeline to sub-device 40) is not documented in that manual;
EVOware sends it before every plunger move and it answers ``-(PPA position + 1000)``.

Positions come from the instrument's own EVOware configuration (Carrier.cfg and
evoware.inf, see ``evoware_config``), exactly as EVOware computes them:
  X = 250 * (grid - 1) + site_x + labware_x1 - carrier_x + XOfs
  Y = site_y + labware_y1 - carrier_y + YOfs
  Z = ZTransform + carrier_z + tip_length - labware_z
PLR resources are matched to EVOware definitions by name (``EVOWARE_NAMES`` or a
resource attribute ``evoware_name``); the grid is the carrier's rail.
"""

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional, Sequence, Tuple

from pylabrobot.legacy.liquid_handling.backends.tecan.evoware_liquid_classes import (
  McaLiquidClass,
  Slp,
  aspirate_slp,
  dispense_slp,
)
from pylabrobot.legacy.liquid_handling.backends.tecan.evoware_config import (
  EvowareCarrier,
  EvowareConfig,
  EvowareLabware,
  EvowareSite,
)
from pylabrobot.resources import Coordinate, Resource, TipRack

Command = Tuple[str, List[Optional[int]]]  # (firmware command, params)

Z_TRAVEL = 2370  # MCA Z range; head fully raised
PLUNGER_READY = 50000  # PPA position EVOware uses before tip fetch / after drop
STEPS_PER_UL = 200  # plunger steps per uL (nominal; 0.0005 mm/step)
SLP_TIP_HANDLING = (1152000, 80000, 3200000)
DITI_PWM_LIMIT = 90  # evoware.opt [MCA] DITI_PWM_LIMIT
PLUNGER_TOLERANCE = 10  # steps; T40RDP4 reads a few steps off at rest (-50998, -47001)


def rdp_expected(position: int) -> int:
  """Value ``T40RDP4`` reports for a given PPA plunger position (from captures)."""
  return -(position + 1000)


# ----------------------------------------------------------------- geometry

# PLR model name -> EVOware name, where they differ beyond "_" vs " ".
EVOWARE_NAMES: Dict[str, str] = {
  "DeepWell_96_Well": "96 Well DeepWell",
  "MCA96_Carrier_4_pos": "MCA96 Carrier 4 pos",
  "DiTi_Waste_3_Trough_Wash": "DiTi Waste 3 Trough + Wash",
}

# Plate gripper (G) opening EVOware sends over labware (captures); default 900.
G_OPEN: Dict[str, int] = {"Trough 300ml MCA96": 950}

TIP_MAX_VOLUME: Dict[str, float] = {"DiTi 200ul MCA96": 200.0, "DiTi 50ul MCA96": 50.0}


def evoware_name(resource: Resource) -> str:
  explicit = getattr(resource, "evoware_name", None)
  if explicit:
    return explicit
  model = getattr(resource, "model", None) or ""
  return EVOWARE_NAMES.get(model, model.replace("_", " "))


def tip_rack_name(tip_model: str) -> str:
  """EVOware rack name for a PLR tip model (``DiTi_200ul_MCA96_tip`` -> ``DiTi 200ul MCA96``)."""
  base = tip_model[:-4] if tip_model.endswith("_tip") else tip_model
  return EVOWARE_NAMES.get(base, base.replace("_", " "))


@dataclass(frozen=True)
class Placement:
  grid: int
  site_number: int  # EVOware numbering (1 = rear)
  carrier: EvowareCarrier
  site: EvowareSite
  labware: EvowareLabware


@dataclass
class McaGeometry:
  config: EvowareConfig
  arm: str = "MCA"

  def labware_resource(self, resource: Resource) -> Resource:
    """The resource itself if EVOware defines it, else the nearest parent that is
    (a well of a trough resolves to the trough)."""
    r: Optional[Resource] = resource
    while r is not None and evoware_name(r) not in self.config.labware:
      r = r.parent
    if r is None:
      raise ValueError(f"{resource.name} has no EVOware labware definition")
    return r

  def placement(self, resource: Resource) -> Placement:
    labware = self.labware_resource(resource)
    holder = labware.parent
    carrier = holder.parent if holder is not None else None
    sites = getattr(carrier, "sites", None)
    if carrier is None or not sites:
      raise ValueError(f"{labware.name} is not on a carrier site")
    index = next((i for i, s in sites.items() if s is holder), None)
    if index is None:
      raise ValueError(f"{labware.name}: carrier site not found")
    cdef = self.config.carrier(evoware_name(carrier))
    evo_sites = len(cdef.sites)
    while evo_sites and (cdef.sites[evo_sites - 1].width, cdef.sites[evo_sites - 1].depth) == (0, 0):
      evo_sites -= 1  # empty placeholder sites (e.g. the 8th of "DiTi Waste 3 Trough + Wash")
    if evo_sites != len(sites):
      raise ValueError(f"{carrier.name}: {len(sites)} PLR sites vs {evo_sites} in EVOware")
    site_number = len(sites) - index  # PLR Tecan carriers list sites front first
    loc = carrier.location
    if loc is None:
      raise ValueError(f"{carrier.name} has no deck location")
    grid_f = (loc.x + carrier.off_x - 100) / 25 + 1  # inverse of TecanDeck rails placement
    grid = round(grid_f)
    if abs(grid - grid_f) > 1e-6:
      raise ValueError(f"{carrier.name} is not on a rail (grid {grid_f})")
    return Placement(grid, site_number, cdef, cdef.sites[site_number - 1],
                     self.config.labware_def(evoware_name(labware)))

  @property
  def offsets(self):
    return self.config.arms[self.arm]

  def xy(self, resource: Resource, offset: Coordinate = Coordinate.zero()) -> Tuple[int, int]:
    p = self.placement(resource)
    o = self.offsets
    x = 250 * (p.grid - 1) + p.site.x + p.labware.x1 - p.carrier.x + o.x
    y = p.site.y + p.labware.y1 - p.carrier.y + o.y
    return round(x + 10 * offset.x), round(y - 10 * offset.y)

  def z(self, resource: Resource, which: str, tip_length: int = 0) -> int:
    """Firmware Z for labware height ``which`` (z_travel/z_start/z_dispense/z_max)."""
    p = self.placement(resource)
    return self.offsets.z + p.carrier.z + tip_length - getattr(p.labware, which)

  def g_open(self, resource: Resource) -> int:
    return G_OPEN.get(evoware_name(self.labware_resource(resource)), 900)

  def tip_length(self, tip_model: str) -> int:
    return self.config.labware_def(tip_rack_name(tip_model)).tip_length


# ------------------------------------------------------------- liquid classes

DEFAULT_CLASS = {"DiTi 200ul MCA96": "Water free dispense", "DiTi 50ul MCA96": "Water wet contact"}


def with_flow_rate(slp: Slp, flow_rate: Optional[float]) -> Slp:
  """Override the SLP speed with a PLR flow rate (uL/s): speed = 200 * flow."""
  if flow_rate is None:
    return slp
  if not (math.isfinite(flow_rate) and flow_rate > 0):
    raise ValueError("flow rate must be positive")
  return (slp[0], round(STEPS_PER_UL * flow_rate), slp[2])


def dispense_slp_for(speed: int, flow_rate: Optional[float]) -> Slp:
  return dispense_slp(speed if flow_rate is None else with_flow_rate((0, speed, 0), flow_rate)[1])


# ------------------------------------------------------------------- plunger


@dataclass
class PlungerState:
  """Backend model of the MCA plunger (all 96 channels move together).

  From the tip end upwards: trailing air gap, liquid (incl. air gaps that separate
  consecutive aspirations), leading air gap. ``excess`` is liquid that is not
  meant to be dispensed (multi pipetting)."""

  position: Optional[int] = None  # PPA units; None = unknown
  lead_air_gap: int = 0
  liquid: int = 0  # plunger steps holding liquid (and separating air gaps)
  trail_air_gap: int = 0
  volume_ul: float = 0.0  # dispensable liquid volume
  excess: int = 0
  mode: Optional[str] = None  # "single" / "multi"
  liquid_class: Optional[McaLiquidClass] = None

  @property
  def is_ready(self) -> bool:
    return self.position == PLUNGER_READY and not (
      self.lead_air_gap or self.liquid or self.trail_air_gap
    )

  @property
  def holds_liquid(self) -> bool:
    return self.liquid > 0

  def reset(self, position: Optional[int]) -> None:
    self.position = position
    self.lead_air_gap = self.liquid = self.trail_air_gap = self.excess = 0
    self.volume_ul = 0.0
    self.mode = self.liquid_class = None


# ------------------------------------------------------------- command plans


@dataclass
class Plan:
  """Commands to send in order. Plunger moves are preceded by a ``T40RDP4``
  check (expected value in ``checks``); ``WAIT`` entries are delays in ms."""

  commands: List[Command] = field(default_factory=list)
  checks: Dict[int, int] = field(default_factory=dict)  # command index -> expected RDP reply

  def add(self, command: str, *params: Optional[int]) -> None:
    self.commands.append((command, list(params)))

  def wait(self, ms: int) -> None:
    if ms > 0:
      self.add("WAIT", ms)

  def plunger_move(self, slp: Slp, steps: int, plunger: PlungerState) -> None:
    if plunger.position is None:
      raise RuntimeError("MCA plunger position unknown; pick up tips (PPA ready) first")
    if not 0 <= plunger.position + steps <= PLUNGER_READY:
      raise ValueError("plunger move outside its travel")
    self.add("SLP", *slp)
    self.checks[len(self.commands)] = rdp_expected(plunger.position)
    self.add("T40RDP4")
    self.add("PPR", steps)
    plunger.position += steps


G_INIT_OPEN = 1284  # gripper fully open (G range) before head drive init, as EVOware does


def plan_initialize_head_drive(plunger: PlungerState) -> Plan:
  """Plunger (head drive) initialization as in EVOware's full initialization.

  ``PIA`` initializes X/Y/Z/G but not the plunger (manual p.34); without this,
  ``PPA``/``PPR`` fail with error 7. EVOware: open G fully, ``PID`` (plunger to
  its upper limit), ``ASH0`` twice (default TeServo parameters), plunger to ready.
  Run only with the head raised and no tips loaded.
  """
  plan = Plan()
  plan.add("PAA", None, None, None, G_INIT_OPEN)
  plan.add("PID")
  plan.add("ASH", 0)
  plan.add("ASH", 0)
  plan.add("SLP", *SLP_TIP_HANDLING)
  plan.add("PPA", PLUNGER_READY)
  plunger.reset(PLUNGER_READY)
  return plan


def plan_move_xy(plan: Plan, x: int, y: int, current: Optional[Tuple[int, int, int]]) -> None:
  """XY move; raises Z to travel height first unless already there. ``current``
  is (x, y, z) from the last readback, or None if unknown (then always raise)."""
  if current is not None and (current[0], current[1]) == (x, y):
    return
  if current is None or current[2] < Z_TRAVEL:
    plan.add("PAA", None, None, Z_TRAVEL)
  plan.add("SSM", 1)
  plan.add("PAA", x, y)
  plan.add("SSM", 0)


def plan_pick_up(geo: McaGeometry, rack: TipRack, offset: Coordinate, current, plunger: PlungerState) -> Plan:
  z_start, z_end = geo.z(rack, "z_start"), geo.z(rack, "z_max")  # cone, no tip yet
  x, y = geo.xy(rack, offset)
  plan = Plan()
  plan.add("SLP", *SLP_TIP_HANDLING)
  plan.add("PPA", PLUNGER_READY)  # ATF must not start with the plunger in the drop position
  plan_move_xy(plan, x, y, current)
  plan.add("PAA", None, None, None, geo.g_open(rack))
  plan.add("PAA", None, None, z_start)
  plan.add("SLP", *SLP_TIP_HANDLING)
  plan.add("ATF", z_start, z_end, Z_TRAVEL, 0, DITI_PWM_LIMIT)
  plunger.reset(PLUNGER_READY)
  return plan


def plan_drop(geo: McaGeometry, rack: TipRack, offset: Coordinate, current, plunger: PlungerState,
              allow_liquid: bool = False) -> Plan:
  if plunger.holds_liquid and not allow_liquid:
    raise RuntimeError("MCA tips still hold liquid; dropping would push it into the tip box")
  z_start = geo.z(rack, "z_start")
  x, y = geo.xy(rack, offset)
  plan = Plan()
  plan_move_xy(plan, x, y, current)
  plan.add("PAA", None, None, None, geo.g_open(rack))
  plan.add("PAA", None, None, z_start)
  plan.add("PAA", None, None, z_start + 10)
  plan.add("SLP", *SLP_TIP_HANDLING)
  plan.add("ATD")
  plan.add("PAA", None, None, Z_TRAVEL)
  plan.add("SLP", *SLP_TIP_HANDLING)
  plan.add("PPA", PLUNGER_READY)
  plunger.reset(PLUNGER_READY)
  return plan


@dataclass(frozen=True)
class LiquidTarget:
  geometry: McaGeometry
  resource: Resource  # plate / trough (or a well of it)
  offset: Coordinate
  tip_model: str
  liquid_height: Optional[float] = None

  def labware(self) -> Resource:
    return self.geometry.labware_resource(self.resource)

  def z_of(self, which: str, offset: int = 0) -> int:
    """Firmware Z of labware height ``which``; ``offset`` (1/10 mm) positive lowers."""
    return self.geometry.z(self.resource, which, self.geometry.tip_length(self.tip_model)) - offset

  def z(self, kind: str) -> int:
    which = {"approach": "z_travel", "retract": "z_start", "dispense": "z_dispense", "bottom": "z_max"}[kind]
    z = self.z_of(which)
    if kind == "bottom" and self.liquid_height is not None:
      if not (math.isfinite(self.liquid_height) and self.liquid_height >= 0):
        raise ValueError("liquid height must be finite and >= 0")
      z += round(10 * self.liquid_height)
    return z

  def aspirate_z(self, lc: McaLiquidClass) -> int:
    z = self.z_of(lc.aspirate_position, lc.aspirate_offset)
    if self.liquid_height is not None:
      z = self.z("bottom")  # PLR liquid_height: height above the bottom
    return z


def _approach(plan: Plan, target: LiquidTarget, current, z_target: int) -> None:
  x, y = target.geometry.xy(target.resource, target.offset)
  plan_move_xy(plan, x, y, current)
  plan.add("PAA", None, None, None, target.geometry.g_open(target.resource))
  plan.add("PAA", None, None, z_target)


@dataclass(frozen=True)
class MixSpec:
  volume: float
  repetitions: int
  flow_rate: Optional[float] = None


def _mix_cycles(plan: Plan, mix: MixSpec, lc: McaLiquidClass, plunger: PlungerState) -> None:
  if not (math.isfinite(mix.volume) and mix.volume > 0) or type(mix.repetitions) is not int or mix.repetitions < 1:
    raise ValueError("mix needs a positive volume and repetitions >= 1")
  steps = round(STEPS_PER_UL * mix.volume)  # EVOware mixes the nominal volume (no calibration)
  if plunger.position - steps < 0:  # type: ignore[operator]
    raise ValueError("mix volume exceeds plunger travel")
  for _ in range(mix.repetitions):
    plan.plunger_move(with_flow_rate(lc.mix_aspirate_slp, mix.flow_rate), -steps, plunger)
    plan.plunger_move(
      dispense_slp_for(lc.single.dispense_speed, mix.flow_rate) if mix.flow_rate else lc.mix_dispense_slp,
      steps, plunger,
    )


def _retract(plan: Plan, z: Optional[int], z_now: int, speed: int, current_z_speed: int) -> int:
  if z is None or z == z_now:
    return z_now
  plan.add("SPA", 2, None, None, speed)
  plan.add("PAA", None, None, z)
  plan.add("SPA", 2, None, None, current_z_speed)
  return z


def plan_aspirate(
  target: LiquidTarget,
  volume: float,
  lc: McaLiquidClass,
  current,
  plunger: PlungerState,
  flow_rate: Optional[float] = None,
  blow_out_air_volume: Optional[float] = None,
  current_z_speed: int = 1200,
  mix: Optional[MixSpec] = None,
  multi: bool = False,
) -> Plan:
  """Aspirate ``volume`` uL into all 96 tips.

  First aspiration of a tip load: leading air gap, (mix), liquid, retract, trailing
  air gap. Further aspirations (``plunger`` already holds liquid): the previous
  trailing air gap stays in the tip as a separator; no new leading air gap.
  ``multi``: EVOware multi-pipetting parameters (excess volume, multi air gaps and
  calibration) so the liquid can be dispensed in several portions.
  """
  if plunger.position is None:
    raise RuntimeError("MCA plunger position unknown; pick up tips first")
  if not (math.isfinite(volume) and volume > 0):
    raise ValueError("volume must be positive")
  first = plunger.is_ready
  if not first:
    if not plunger.holds_liquid:
      raise RuntimeError("MCA tips hold only air but the plunger is not ready; pick up tips again")
    if plunger.mode != ("multi" if multi else "single"):
      raise ValueError("all aspirations of one tip load must use the same pipetting mode")
  mode = lc.multi if multi else lc.single
  max_volume = TIP_MAX_VOLUME.get(tip_rack_name(target.tip_model), 0)
  if plunger.volume_ul + volume + (mode.excess_ul if first and multi else 0) > max_volume:
    raise ValueError("volume exceeds tip capacity")
  lead = 0
  if first:
    lead = mode.lead_air_gap if blow_out_air_volume is None else round(STEPS_PER_UL * blow_out_air_volume)
  steps = mode.aspirate_steps(volume)
  excess = round(STEPS_PER_UL * mode.excess_ul) if (first and multi) else 0
  if plunger.position - lead - steps - excess - mode.trail_air_gap < 0:
    raise ValueError("aspiration exceeds plunger travel")

  plan = Plan()
  _approach(plan, target, current, target.z("approach"))
  if lead:
    plan.plunger_move(aspirate_slp(mode.lead_speed), -lead, plunger)
  z = target.aspirate_z(lc)
  plan.add("PAA", None, None, z)
  if mix is not None:
    if plunger.holds_liquid:
      raise ValueError("mixing before aspiration needs tips without liquid")
    _mix_cycles(plan, mix, lc, plunger)
  plan.plunger_move(with_flow_rate(aspirate_slp(mode.aspirate_speed), flow_rate), -(steps + excess), plunger)
  plan.wait(mode.aspirate_delay_ms)
  _retract(plan, target.z_of(lc.aspirate_retract_position, lc.aspirate_retract_offset), z,
           lc.aspirate_retract_z_speed, current_z_speed)
  if mode.trail_air_gap:
    plan.plunger_move(aspirate_slp(mode.trail_speed), -mode.trail_air_gap, plunger)

  plunger.lead_air_gap += lead
  plunger.liquid += steps + excess + plunger.trail_air_gap  # old trailing gap is now a separator
  plunger.trail_air_gap = mode.trail_air_gap
  plunger.excess += excess
  plunger.volume_ul += volume
  plunger.mode = "multi" if multi else "single"
  plunger.liquid_class = lc
  return plan


def plan_dispense(
  target: LiquidTarget,
  volume: float,
  current,
  plunger: PlungerState,
  flow_rate: Optional[float] = None,
  current_z_speed: int = 1200,
  mix: Optional[MixSpec] = None,
) -> Plan:
  """Dispense ``volume`` uL from all 96 tips, with the liquid class of the aspiration.

  Dispensing all remaining liquid in single mode empties the tips completely
  (liquid and all air gaps), as EVOware does. Otherwise a portion is dispensed:
  trailing air gap + 200 * factor * volume steps, after which (multi mode,
  ``tag``) a new trailing air gap is aspirated. In multi mode the excess stays in
  the tips; empty it with ``plan_empty``.
  """
  lc = plunger.liquid_class
  if plunger.position is None or not plunger.holds_liquid or lc is None:
    raise RuntimeError("MCA has no aspirated liquid to dispense")
  if not (math.isfinite(volume) and volume > 0):
    raise ValueError("volume must be positive")
  if volume > plunger.volume_ul + 1e-6:
    raise ValueError(f"cannot dispense {volume} uL; tips hold {plunger.volume_ul:g} uL")
  mode = lc.multi if plunger.mode == "multi" else lc.single
  everything = plunger.mode == "single" and math.isclose(volume, plunger.volume_ul, abs_tol=1e-6)

  plan = Plan()
  z = target.z_of(lc.dispense_position, lc.dispense_offset)
  if lc.dispense_contact and target.liquid_height is not None:
    z = target.z("bottom")
  _approach(plan, target, current, z)
  slp = dispense_slp_for(mode.dispense_speed, flow_rate)
  if everything:
    plan.plunger_move(slp, PLUNGER_READY - plunger.position, plunger)  # type: ignore[operator]
  else:
    portion = round(STEPS_PER_UL * mode.calibration_factor * volume)
    remaining_volume = plunger.volume_ul - volume
    if math.isclose(remaining_volume, 0, abs_tol=1e-6):
      portion = plunger.liquid - plunger.excess  # last portion: all dispensable liquid
    plan.plunger_move(slp, plunger.trail_air_gap + portion, plunger)
    plunger.liquid -= portion
    plunger.trail_air_gap = 0
    plunger.volume_ul = max(remaining_volume, 0.0)
  plan.wait(mode.dispense_delay_ms)
  z = _retract(plan, target.z_of(lc.dispense_retract_position, lc.dispense_retract_offset)
               if lc.dispense_retract_position else None, z, lc.dispense_retract_z_speed, current_z_speed)
  if everything:
    plunger.reset(PLUNGER_READY)
    if mix is not None:
      plunger.liquid_class = lc
      plan.add("PAA", None, None, target.z("bottom"))
      plan.plunger_move(aspirate_slp(lc.single.lead_speed), -lc.single.lead_air_gap, plunger)
      plunger.lead_air_gap = lc.single.lead_air_gap
      _mix_cycles(plan, mix, lc, plunger)
      plan.add("PAA", None, None, Z_TRAVEL)
      plunger.liquid_class = None
  else:
    if mix is not None:
      raise ValueError("mixing after a partial dispense would mix the remaining liquid")
    if plunger.mode == "multi" and mode.dispense_tag and plunger.volume_ul > 1e-6 and mode.trail_air_gap:
      plan.plunger_move(aspirate_slp(mode.trail_speed), -mode.trail_air_gap, plunger)
      plunger.trail_air_gap = mode.trail_air_gap
  return plan


def plan_empty(target: LiquidTarget, current, plunger: PlungerState) -> Plan:
  """Expel everything left in the tips (e.g. the multi-pipetting excess) at
  ``target``'s dispense height; the plunger returns to ready."""
  lc = plunger.liquid_class
  if plunger.position is None or lc is None:
    raise RuntimeError("MCA plunger state unknown")
  plan = Plan()
  _approach(plan, target, current, target.z_of(lc.dispense_position, lc.dispense_offset))
  plan.plunger_move(dispense_slp(lc.single.dispense_speed), PLUNGER_READY - plunger.position, plunger)
  plan.wait(lc.single.dispense_delay_ms)
  plunger.reset(PLUNGER_READY)
  return plan


def plan_mix(
  target: LiquidTarget,
  volume: float,
  repetitions: int,
  lc: McaLiquidClass,
  current,
  plunger: PlungerState,
  flow_rate: Optional[float] = None,
) -> Plan:
  """EVOware MCAMix: leading air gap once, immerse, cycle without leaving the
  liquid, then raise to travel height."""
  if plunger.position is None or plunger.holds_liquid or plunger.trail_air_gap:
    raise RuntimeError("MCA mix needs tips with an empty plunger (air gap allowed)")
  plan = Plan()
  _approach(plan, target, current, target.z("approach"))
  if not plunger.lead_air_gap:
    plan.plunger_move(aspirate_slp(lc.single.lead_speed), -lc.single.lead_air_gap, plunger)
    plunger.lead_air_gap = lc.single.lead_air_gap
  plan.add("PAA", None, None, target.aspirate_z(lc))
  _mix_cycles(plan, MixSpec(volume, repetitions, flow_rate), lc, plunger)
  plan.add("SSM", 1)
  plan.add("PAA", None, None, Z_TRAVEL)
  plan.add("SSM", 0)
  return plan


# ------------------------------------------------------------- plate gripper
#
# The MCA96 plate gripper (G axis, AGR "grip a plate", manual p.37) is optional; this
# instrument has none (evoware.inf [MCA] HasGripper=0). Not tested on hardware.
# EVOware moves the gripper with carrier vectors; these primitives take explicit
# firmware heights instead.

GRIP_SPEED = 100  # AGR speed as sent by EVOware (log 2024-09-19)
GRIP_PWM = 75


def plan_grip(geo: McaGeometry, resource: Resource, current, z_grip: int, grip_width: int,
              open_width: int, speed: int = GRIP_SPEED, pwm: int = GRIP_PWM) -> Plan:
  """Move over ``resource`` (channel A1 reference), open G, lower to ``z_grip``, grip
  to ``grip_width`` (AGR fails with error 9 if no plate is found), raise."""
  if not open_width > grip_width > 0:
    raise ValueError("open width must exceed the grip width")
  x, y = geo.xy(resource)
  plan = Plan()
  plan_move_xy(plan, x, y, current)
  plan.add("PAA", None, None, None, open_width)
  plan.add("PAA", None, None, z_grip)
  plan.add("AGR", grip_width, speed, pwm)
  plan.add("PAA", None, None, Z_TRAVEL)
  return plan


def plan_release(geo: McaGeometry, destination: Resource, current, z_release: int, open_width: int) -> Plan:
  """Move a gripped plate over ``destination``, lower to ``z_release``, open G, raise."""
  x, y = geo.xy(destination)
  plan = Plan()
  plan_move_xy(plan, x, y, current)
  plan.add("PAA", None, None, z_release)
  plan.add("PAA", None, None, None, open_width)
  plan.add("PAA", None, None, Z_TRAVEL)
  return plan


def track(current: Optional[Tuple[int, int, int]], plan: Plan) -> Optional[Tuple[int, int, int]]:
  """Head (x, y, z) after ``plan`` given the position before it (None = unknown)."""
  x, y, z = current if current is not None else (None, None, None)
  for command, params in plan.commands:
    if command == "PAA":
      p = list(params) + [None] * (4 - len(params))
      x = p[0] if p[0] is not None else x
      y = p[1] if p[1] is not None else y
      z = p[2] if p[2] is not None else z
    elif command == "ATF":
      z = params[2]  # Zretract
  if None in (x, y, z):
    return None
  return (x, y, z)  # type: ignore[return-value]


def normalize(commands: Sequence[str]) -> Tuple[List[str], List[str]]:
  """Reduce a W1 command list to its operation core for comparison with EVOware.

  Drops reads (RAD/RPA), SSM toggles, WAIT and Z-only moves before the first G
  move, and separates XY moves. Returns (core, xy_targets) with repeated XY removed.
  """
  core, xy = [], []
  seen_g = False
  for c in commands:
    c = c.rstrip(",")  # EVOware writes trailing empty fields; the firmware ignores them
    if c.startswith(("RAD", "RPA", "SSM", "WAIT")):
      continue
    if c.startswith("PAA"):
      params = c[3:].split(",")
      if len(params) >= 2 and params[0] and params[1] and not any(params[2:]):
        if not xy or xy[-1] != c:
          xy.append(c)
        continue
      if len(params) == 4 and not any(params[:3]) and params[3]:
        seen_g = True
      elif not seen_g and len(params) >= 3 and not params[0] and not params[1] and params[2]:
        continue
    core.append(c)
  return core, xy


def format_command(command: str, params: Sequence[Optional[int]]) -> str:
  """Wire text of a command, as ``TecanLiquidHandler._assemble_command`` builds it."""
  return command + ",".join("" if p is None else str(p) for p in params)
