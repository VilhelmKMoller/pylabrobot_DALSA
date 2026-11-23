"""EVOware liquid classes (DefaultLCs.XML / CustomLCs.XML) for the MCA96 and the LiHa (see the LiHa
section below for its conversions).

EVOware stores per liquid class and tip type ("MCADiTi" for MCA96 disposable
tips) volume sub-classes with air gaps, speeds (uL/s), delays (ms), retract and
position settings and a calibration (factor, offset). For the MCA96 plunger
(200 steps per uL) every value maps directly onto firmware parameters; this was
verified against EVOware USB captures for "Water free dispense" (1, 100, 175 uL,
200 uL tips) and "Water wet contact" (1, 50 uL, 50 uL tips):

  LAG / TAG volume (uL) * 200  -> leading / trailing air gap (PPR steps)
  speed (uL/s) * 200           -> SLP speed (steps/s)
  Retract speed (mm/s) * 10    -> SPA 2 Z speed (1/10 mm/s)
  steps = 200 * (factor * V + offset)

Numeric position codes are not documented in the EVOware help; only codes whose
meaning is confirmed by captures are accepted (see ``*_POSITIONS``). SLP start
and stop ramps are not part of the XML: aspiration always uses 1152000/3200000;
dispense ramps are known only for the speeds seen in captures (``DISPENSE_RAMPS``).
"""

import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union

STEPS_PER_UL = 200
ASPIRATE_RAMPS = (1152000, 3200000)
# dispense SLP speed (steps/s) -> (start ramp, stop ramp), from EVOware captures
DISPENSE_RAMPS: Dict[int, Tuple[int, int]] = {
  2000: (1152000, 3200000),
  10000: (1152000, 3200000),
  80000: (1200000, 3200000),
  100000: (6000000, 6200000),
}

# Z position codes, per field, confirmed by captures. None = stay where the head is.
ASPIRATE_POSITIONS = {0: "z_max"}
ASPIRATE_RETRACT_POSITIONS = {2: "z_start"}
DISPENSE_POSITIONS = {0: "z_dispense", 6: "z_max"}
DISPENSE_RETRACT_POSITIONS = {2: "z_start", 3: "z_dispense"}

Slp = Tuple[int, int, int]


def dispense_slp(speed_steps: int) -> Slp:
  if speed_steps in DISPENSE_RAMPS:
    start, stop = DISPENSE_RAMPS[speed_steps]
  elif speed_steps <= 10000:
    start, stop = ASPIRATE_RAMPS  # all captured speeds <= 50 uL/s use these ramps
  else:
    raise NotImplementedError(
      f"SLP ramps for an MCA dispense speed of {speed_steps / STEPS_PER_UL:g} uL/s are not known "
      f"(known: {sorted(s / STEPS_PER_UL for s in DISPENSE_RAMPS)} uL/s and <= 50 uL/s)"
    )
  return (start, speed_steps, stop)


def aspirate_slp(speed_steps: int) -> Slp:
  return (ASPIRATE_RAMPS[0], speed_steps, ASPIRATE_RAMPS[1])


def _steps(ul: float) -> int:
  return round(STEPS_PER_UL * ul)


@dataclass(frozen=True)
class PipettingMode:
  """Single or multi pipetting parameters of a sub-class (firmware units)."""

  lead_air_gap: int
  lead_speed: int
  trail_air_gap: int
  trail_speed: int
  aspirate_speed: int
  aspirate_delay_ms: int
  excess_ul: float
  dispense_speed: int
  dispense_delay_ms: int
  dispense_tag: bool  # multi: re-aspirate the trailing air gap after each dispense
  calibration_factor: float
  calibration_offset_ul: float

  def aspirate_steps(self, volume: float) -> int:
    return _steps(self.calibration_factor * volume + self.calibration_offset_ul)


@dataclass(frozen=True)
class McaLiquidClass:
  name: str
  tip_type: str
  volume_range: Tuple[float, float]  # min <= V < max (EVOware upper bounds are "x.01")
  single: PipettingMode
  multi: PipettingMode
  aspirate_position: str
  aspirate_offset: int  # 1/10 mm, positive lowers
  aspirate_retract_position: str
  aspirate_retract_offset: int
  aspirate_retract_z_speed: int  # SPA 2 Z units
  dispense_position: str
  dispense_offset: int
  dispense_retract_position: Optional[str]
  dispense_retract_offset: int
  dispense_retract_z_speed: int
  source: str = ""

  @property
  def dispense_contact(self) -> bool:
    return self.dispense_position == "z_max"

  # mix strokes use the single-mode speeds (EVOware MCAMix capture)
  @property
  def mix_aspirate_slp(self) -> Slp:
    return aspirate_slp(self.single.aspirate_speed)

  @property
  def mix_dispense_slp(self) -> Slp:
    return dispense_slp(self.single.dispense_speed)


def _attr(element, path, name, cast=float):
  node = element.find(path)
  if node is None or node.get(name) is None:
    raise ValueError(f"liquid class element {path}@{name} missing")
  value = node.get(name)
  return value == "True" if cast is bool else cast(value)


def _mode(sc, mode: str) -> PipettingMode:
  a, d, c = f"Aspirate/{mode}", f"Dispense/{mode}", f"Calibration/{mode}"
  excess_unit = int(_attr(sc, f"{a}/Excess", "unit"))
  if excess_unit != 0:
    raise NotImplementedError("excess volume in % is not supported")
  return PipettingMode(
    lead_air_gap=_steps(_attr(sc, f"{a}/LAG", "volume")),
    lead_speed=_steps(_attr(sc, f"{a}/LAG", "speed")),
    trail_air_gap=_steps(_attr(sc, f"{a}/TAG", "volume")),
    trail_speed=_steps(_attr(sc, f"{a}/TAG", "speed")),
    aspirate_speed=_steps(_attr(sc, a, "speed")),
    aspirate_delay_ms=int(_attr(sc, a, "delay")),
    excess_ul=_attr(sc, f"{a}/Excess", "volume"),
    dispense_speed=_steps(_attr(sc, d, "speed")),
    dispense_delay_ms=int(_attr(sc, d, "delay")),
    dispense_tag=_attr(sc, d, "tag", bool),
    calibration_factor=_attr(sc, c, "factor"),
    calibration_offset_ul=_attr(sc, c, "offset"),
  )


def _position(table, code, what, name):
  code = int(code)
  if code not in table:
    raise NotImplementedError(f"{name}: {what} position code {code} is not verified for the MCA")
  return table[code]


def subclass_to_mca(name: str, sc, source: str = "") -> McaLiquidClass:
  if _attr(sc, "Aspirate/LLD", "detect", bool) or _attr(sc, "Dispense/LLD", "detect", bool):
    raise NotImplementedError(f"{name}: the MCA96 has no liquid level detection")
  return McaLiquidClass(
    name=name,
    tip_type=sc.get("tipType"),
    volume_range=(float(sc.get("min")), float(sc.get("max"))),
    single=_mode(sc, "Single"),
    multi=_mode(sc, "Multi"),
    aspirate_position=_position(
      ASPIRATE_POSITIONS, _attr(sc, "Aspirate/LLD", "position"), "aspirate", name
    ),
    aspirate_offset=round(10 * _attr(sc, "Aspirate/LLD", "offset")),
    aspirate_retract_position=_position(
      ASPIRATE_RETRACT_POSITIONS,
      _attr(sc, "Aspirate/Retract", "position"),
      "aspirate retract",
      name,
    ),
    aspirate_retract_offset=round(10 * _attr(sc, "Aspirate/Retract", "offset")),
    aspirate_retract_z_speed=round(10 * _attr(sc, "Aspirate/Retract", "speed")),
    dispense_position=_position(
      DISPENSE_POSITIONS, _attr(sc, "Dispense/LLD", "position"), "dispense", name
    ),
    dispense_offset=round(10 * _attr(sc, "Dispense/LLD", "offset")),
    dispense_retract_position=_position(
      DISPENSE_RETRACT_POSITIONS,
      _attr(sc, "Dispense/Retract", "position"),
      "dispense retract",
      name,
    ),
    dispense_retract_offset=round(10 * _attr(sc, "Dispense/Retract", "offset")),
    dispense_retract_z_speed=round(10 * _attr(sc, "Dispense/Retract", "speed")),
    source=source,
  )


def _parse(path: Path):
  text = path.read_text(encoding="utf-8-sig")
  text = re.sub(r"<Signature [^>]*/>", "", text)  # signatures contain invalid XML char refs
  return ET.fromstring(text)


@dataclass
class LiquidClassLibrary:
  """Raw EVOware liquid classes; MCA classes are converted on lookup."""

  classes: Dict[str, Tuple[ET.Element, str]]

  @classmethod
  def load(cls, *files: Union[str, Path]) -> "LiquidClassLibrary":
    classes: Dict[str, Tuple[ET.Element, str]] = {}
    for f in files:
      p = Path(f)
      if not p.exists():
        continue
      for lc in _parse(p):
        classes[lc.get("name")] = (lc, p.name)  # later files (custom) override
    return cls(classes)

  @classmethod
  def from_database(cls, database_dir: Union[str, Path]) -> "LiquidClassLibrary":
    d = Path(database_dir)
    return cls.load(d / "DefaultLCs.XML", d / "CustomLCs.XML")

  def names(self) -> List[str]:
    return sorted(self.classes)

  def _subclass(self, name: str, volume: float, tip_type: str):
    if name not in self.classes:
      raise KeyError(f"liquid class {name!r} not found")
    lc, source = self.classes[name]
    for sc in lc.iter("SubClass"):
      if sc.get("tipType") != tip_type:
        continue
      low, high = float(sc.get("min", "nan")), float(sc.get("max", "nan"))
      if low <= volume < high:
        return lc, sc, source
    raise ValueError(f"liquid class {name!r} has no {tip_type} sub-class for {volume} uL")

  def mca(self, name: str, volume: float, tip_type: str = "MCADiTi") -> McaLiquidClass:
    _, sc, source = self._subclass(name, volume, tip_type)
    return subclass_to_mca(name, sc, source)

  def liha(self, name: str, volume: float, tip_type: str = "DiTi") -> "LihaLiquidClass":
    """The LiHa sub-class of ``name`` for ``volume`` uL with ``tip_type`` ("DiTi" for disposable
    tips), with the class-wide liquid detection settings."""
    lc, sc, source = self._subclass(name, volume, tip_type)
    return subclass_to_liha(name, lc.find("Global"), sc, source)


# --- LiHa -----------------------------------------------------------------------------------------
#
# Verified against EVOware LiHa run logs ("Water free dispense" and "Water wet contact", DiTi
# sub-classes 3-15, 15-200, 200-1000 and 50-200 uL; test_data/liha_logs/liquid_class_cases.json).
# The LiHa dilutor moves in full steps (DiluterSteps per DilutorVolume: 3 steps/uL here) and its
# speeds are set in half steps per second, so with s = steps per uL:
#   volume V uL                -> PPR / MTR  round(s * V)
#   speed v uL/s               -> SEP / SPP  round(2 * s * v)
#   aspirated volume           -> factor * V + offset (Calibration/Single or Multi)
#   Global LLD mode, conductivity -> SDM DetProc, Sense
#   Global LLD speed (mm/s)    -> SSL * 10;  doubleDist (mm) -> SDL * 10
#   LLD offset (mm)            -> SBL * 10 (submerge below the surface; negative = above it)
#   Retract speed (mm/s)       -> SSZ * 10
# Only position codes seen in the logs are accepted (LIHA_*_POSITIONS).

# Aspirate/LLD position -> meaning (only with detect="True")
LIHA_ASPIRATE_POSITIONS = {3: "liquid_level"}
# Aspirate/Retract position: 4 = relative to the detected surface (offset -5 = 5 mm above it)
LIHA_ASPIRATE_RETRACT_POSITIONS = {4: "liquid_level"}
# Dispense/LLD position: 0 without detection = labware z_dispense (free dispense);
# 3 with detection = detected surface (wet contact)
LIHA_DISPENSE_POSITIONS = {(False, 0): "z_dispense", (True, 3): "liquid_level"}
# Dispense/Retract position: 1 = labware z_dispense
LIHA_DISPENSE_RETRACT_POSITIONS = {1: "z_dispense"}


@dataclass(frozen=True)
class LihaMode:
  """Single or Multi pipetting values of a LiHa sub-class, in EVOware units (uL, uL/s, ms)."""

  aspirate_speed: float
  aspirate_delay_ms: int
  lead_air_gap: float
  lead_speed: float
  trail_air_gap: float
  trail_speed: float
  system_trail_air_gap: float
  system_trail_speed: float
  excess: float
  conditioning: float
  dispense_speed: float
  breakoff_speed: float
  dispense_delay_ms: int
  dispense_tag: bool
  calibration_factor: float
  calibration_offset: float

  def calibrated(self, volume: float) -> float:
    """Volume EVOware aspirates for a nominal ``volume`` (the notes' 'aspirate 156.00ul')."""
    return self.calibration_factor * volume + self.calibration_offset


@dataclass(frozen=True)
class LihaLiquidClass:
  """A LiHa liquid sub-class (EVOware units). Convert with ``LihaFirmwareValues``."""

  name: str
  tip_type: str
  volume_range: Tuple[float, float]  # min <= V < max
  single: LihaMode
  multi: LihaMode
  lld_mode: int  # SDM DetProc
  lld_sense: int  # SDM Sense ("conductivity")
  lld_speed: float  # mm/s
  lld_double_dist: float  # mm
  aspirate_position: str
  aspirate_offset: float  # mm below the surface
  aspirate_retract_position: str
  aspirate_retract_offset: float  # mm, negative = above
  aspirate_retract_speed: float  # mm/s
  dispense_lld: bool
  dispense_position: str
  dispense_offset: float
  dispense_retract_position: str
  dispense_retract_offset: float
  dispense_retract_speed: float
  source: str = ""

  def mode(self, multi: bool = False) -> LihaMode:
    return self.multi if multi else self.single


@dataclass(frozen=True)
class LihaFirmwareValues:
  """Firmware values of a LiHa liquid class for a dilutor with ``steps_per_ul`` full steps per uL
  (``liha.LihaMachine.steps_per_ul``, 3 on this instrument)."""

  lc: LihaLiquidClass
  steps_per_ul: float

  def steps(self, ul: float) -> int:
    """PPR / MTR / PPA plunger steps (full steps)."""
    return round(self.steps_per_ul * ul)

  def speed(self, ul_per_s: float) -> int:
    """SEP / SPP plunger speed (half steps per second)."""
    return round(2 * self.steps_per_ul * ul_per_s)

  def aspirate_steps(self, volume: float, multi: bool = False) -> int:
    """MTR steps for aspirating the nominal ``volume`` (calibrated)."""
    return self.steps(self.lc.mode(multi).calibrated(volume))

  def dispense_steps(self, volume: float, multi: bool = False) -> int:
    """MTR steps (negative) for a single-mode full dispense: the calibrated volume plus the
    leading and trailing air gaps (logged e.g. 156 + 5 + 10 uL -> MTR-513)."""
    if multi:
      raise NotImplementedError("multi-dispense volumes are not verified for the LiHa")
    m = self.lc.single
    return -self.steps(m.calibrated(volume) + m.lead_air_gap + m.trail_air_gap)

  @property
  def detection_mode(self) -> Tuple[int, int]:
    """SDM DetProc, Sense."""
    return (self.lc.lld_mode, self.lc.lld_sense)

  @property
  def search_speed(self) -> int:
    """SSL (1/10 mm/s)."""
    return round(10 * self.lc.lld_speed)

  @property
  def retract_distance(self) -> int:
    """SDL safe detection retract distance (1/10 mm)."""
    return round(10 * self.lc.lld_double_dist)

  def submerge(self, dispense: bool = False) -> int:
    """SBL (1/10 mm below the surface; negative = above)."""
    return round(10 * (self.lc.dispense_offset if dispense else self.lc.aspirate_offset))

  def retract_speed(self, dispense: bool = False) -> int:
    """SSZ for the retract move (1/10 mm/s)."""
    lc = self.lc
    return round(10 * (lc.dispense_retract_speed if dispense else lc.aspirate_retract_speed))

  def retract_z_above_surface(self) -> int:
    """Aspirate retract target relative to the detected surface (1/10 mm, positive = up)."""
    return -round(10 * self.lc.aspirate_retract_offset)


def _liha_mode(sc, mode: str) -> LihaMode:
  a, d, c = f"Aspirate/{mode}", f"Dispense/{mode}", f"Calibration/{mode}"
  if int(_attr(sc, f"{a}/Excess", "unit")) != 0:
    raise NotImplementedError("excess volume in % is not supported")
  return LihaMode(
    aspirate_speed=_attr(sc, a, "speed"),
    aspirate_delay_ms=int(_attr(sc, a, "delay")),
    lead_air_gap=_attr(sc, f"{a}/LAG", "volume"),
    lead_speed=_attr(sc, f"{a}/LAG", "speed"),
    trail_air_gap=_attr(sc, f"{a}/TAG", "volume"),
    trail_speed=_attr(sc, f"{a}/TAG", "speed"),
    system_trail_air_gap=_attr(sc, f"{a}/STAG", "volume"),
    system_trail_speed=_attr(sc, f"{a}/STAG", "speed"),
    excess=_attr(sc, f"{a}/Excess", "volume"),
    conditioning=_attr(sc, f"{a}/Conditioning", "volume"),
    dispense_speed=_attr(sc, d, "speed"),
    breakoff_speed=_attr(sc, d, "breakoff"),
    dispense_delay_ms=int(_attr(sc, d, "delay")),
    dispense_tag=_attr(sc, d, "tag", bool),
    calibration_factor=_attr(sc, c, "factor"),
    calibration_offset=_attr(sc, c, "offset"),
  )


def subclass_to_liha(name: str, global_, sc, source: str = "") -> LihaLiquidClass:
  """Convert a LiHa ``<SubClass>`` (with its class ``<Global>``); refuses unverified settings."""

  def refuse(what: str):
    raise NotImplementedError(
      f"{name} ({sc.get('tipType')} {sc.get('min')}-{sc.get('max')}): {what}"
    )

  if global_ is None or global_.find("LLD") is None:
    refuse("no Global/LLD settings")
  for path in ("Aspirate/Mix", "Dispense/Mix"):
    if _attr(sc, path, "enabled", bool):
      refuse(f"{path} in the liquid class is not verified for the LiHa")
  if _attr(sc, "Aspirate/ExitSignal", "detect", bool):
    refuse("exit signal detection is not verified for the LiHa")
  if int(_attr(sc, "Dispense/TipTouching", "direction")) != 0:
    refuse("tip touching is not verified for the LiHa")
  if not _attr(sc, "Aspirate/LLD", "detect", bool):
    refuse("aspiration without liquid level detection is not verified for the LiHa")

  def code(table, key, what):
    if key not in table:
      refuse(f"{what} position {key} is not verified for the LiHa")
    return table[key]

  dispense_lld = _attr(sc, "Dispense/LLD", "detect", bool)
  lld = global_.find("LLD")
  return LihaLiquidClass(
    name=name,
    tip_type=sc.get("tipType"),
    volume_range=(float(sc.get("min")), float(sc.get("max"))),
    single=_liha_mode(sc, "Single"),
    multi=_liha_mode(sc, "Multi"),
    lld_mode=int(lld.get("mode")),
    lld_sense=int(lld.get("conductivity")),
    lld_speed=float(lld.get("speed")),
    lld_double_dist=float(lld.get("doubleDist")),
    aspirate_position=code(
      LIHA_ASPIRATE_POSITIONS, int(_attr(sc, "Aspirate/LLD", "position")), "aspirate"
    ),
    aspirate_offset=_attr(sc, "Aspirate/LLD", "offset"),
    aspirate_retract_position=code(
      LIHA_ASPIRATE_RETRACT_POSITIONS,
      int(_attr(sc, "Aspirate/Retract", "position")),
      "aspirate retract",
    ),
    aspirate_retract_offset=_attr(sc, "Aspirate/Retract", "offset"),
    aspirate_retract_speed=_attr(sc, "Aspirate/Retract", "speed"),
    dispense_lld=dispense_lld,
    dispense_position=code(
      LIHA_DISPENSE_POSITIONS,
      (dispense_lld, int(_attr(sc, "Dispense/LLD", "position"))),
      "dispense",
    ),
    dispense_offset=_attr(sc, "Dispense/LLD", "offset"),
    dispense_retract_position=code(
      LIHA_DISPENSE_RETRACT_POSITIONS,
      int(_attr(sc, "Dispense/Retract", "position")),
      "dispense retract",
    ),
    dispense_retract_offset=_attr(sc, "Dispense/Retract", "offset"),
    dispense_retract_speed=_attr(sc, "Dispense/Retract", "speed"),
    source=source,
  )
