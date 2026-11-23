"""Read EVOware's instrument configuration (Carrier.cfg, evoware.inf).

EVOware stores every carrier and labware definition of an instrument in
``ProgramData/Tecan/EVOware/database/Carrier.cfg`` and arm offsets in
``evoware.inf``. Positions of an arm over labware follow from these values; for
the MCA96 (verified against EVOware USB captures):

  X = 250 * (grid - 1) + site_x + labware_x1 - carrier_x + XOfs
  Y = site_y + labware_y1 - carrier_y + YOfs
  Z = ZTransform + carrier_z + tip_length - labware_z

with ``labware_z`` one of the labware's z_travel / z_start / z_dispense / z_max
(1/10 mm, larger = deeper) and ``tip_length`` 0 for tip handling without tips.

Carrier.cfg records (semicolon separated, continuation lines start with 998):
  13;<carrier>;<id>/<?>;<x>/<y>/<z>;<?>/<?>;<n sites>;<?>;  + one 998 line per site:
     998;0;<w>/<h>;<site x>/<site y>/<site z>;;
  15;<labware>;0;<cols>/<rows>/<?>;<x1>/<y1>/<x2>/<y2>/<?>;<z_max>/<z_start>/<z_dispense>/<z_travel>;
     <area>;<positions>;...;<tip volume>;<tip length>;...
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union


@dataclass(frozen=True)
class EvowareSite:
  x: int
  y: int
  z: int
  width: int
  depth: int


@dataclass(frozen=True)
class EvowareCarrier:
  name: str
  id: int
  x: int
  y: int
  z: int
  sites: Tuple[EvowareSite, ...]


@dataclass(frozen=True)
class EvowareLabware:
  name: str
  columns: int
  rows: int
  x1: int
  y1: int
  x2: int
  y2: int
  z_max: int
  z_start: int
  z_dispense: int
  z_travel: int
  area: float
  tip_volume: int
  tip_length: int
  positions: int = 0
  """Field 8 of the record. Read as the number of addressable positions: 96 for 'Trough 300ml
  MCA96' (1 column x 8 rows, yet EVOware addresses 12 columns), 8 for 'Trough 100ml', 1 for plates.
  Not documented; inferred from EVOware LiHa logs (see liha.LihaGeometry)."""


@dataclass(frozen=True)
class ArmOffsets:
  x: int
  y: int
  z: int


@dataclass
class EvowareConfig:
  carriers: Dict[str, EvowareCarrier] = field(default_factory=dict)
  labware: Dict[str, EvowareLabware] = field(default_factory=dict)
  arms: Dict[str, ArmOffsets] = field(default_factory=dict)
  sections: Dict[str, Dict[str, str]] = field(default_factory=dict)  # evoware.inf, raw

  @property
  def carriers_by_id(self) -> Dict[int, EvowareCarrier]:
    return {c.id: c for c in self.carriers.values()}

  def carrier(self, name: str) -> EvowareCarrier:
    try:
      return self.carriers[name]
    except KeyError:
      raise KeyError(f"carrier {name!r} not in EVOware configuration") from None

  def labware_def(self, name: str) -> EvowareLabware:
    try:
      return self.labware[name]
    except KeyError:
      raise KeyError(f"labware {name!r} not in EVOware configuration") from None


def _ints(text: str) -> List[int]:
  return [int(v) for v in text.split("/") if v.strip() != ""]


def parse_carrier_cfg(text: str, config: Optional[EvowareConfig] = None) -> EvowareConfig:
  config = config or EvowareConfig()
  lines = text.splitlines()
  i = 0
  while i < len(lines):
    parts = lines[i].split(";")
    kind = parts[0]
    if kind == "13" and len(parts) >= 7:
      ident = _ints(parts[2])[0]
      x, y, z = _ints(parts[3])[:3]
      n = int(parts[5])
      sites = []
      for site_line in lines[i + 1 : i + 1 + n]:
        sp = site_line.split(";")
        w, d = _ints(sp[2])[:2]
        sx, sy, sz = _ints(sp[3])[:3]
        sites.append(EvowareSite(sx, sy, sz, w, d))
      config.carriers[parts[1]] = EvowareCarrier(parts[1], ident, x, y, z, tuple(sites))
      i += 1 + n
      continue
    if kind == "15" and len(parts) >= 15:
      cols, rows = _ints(parts[3])[:2]
      x1, y1, x2, y2 = _ints(parts[4])[:4]
      z_max, z_start, z_dispense, z_travel = _ints(parts[5])[:4]
      config.labware[parts[1]] = EvowareLabware(
        parts[1],
        cols,
        rows,
        x1,
        y1,
        x2,
        y2,
        z_max,
        z_start,
        z_dispense,
        z_travel,
        float(parts[6]),
        int(parts[13]),
        int(parts[14]),
        int(parts[7]),
      )
    i += 1
  return config


def parse_evoware_inf(text: str, config: Optional[EvowareConfig] = None) -> EvowareConfig:
  """Arm offsets (``XOfs``, ``YOfs``, ``ZTransform``) per ``[section]``; every
  section's raw ``key=value`` pairs are kept in ``config.sections``."""
  config = config or EvowareConfig()
  section, values = None, {}

  def flush():
    if section:
      config.sections[section] = dict(values)
    if section and {"XOfs", "YOfs", "ZTransform"} <= values.keys():
      config.arms[section] = ArmOffsets(
        int(values["XOfs"]), int(values["YOfs"]), int(values["ZTransform"])
      )

  for raw in text.splitlines():
    line = raw.strip()
    if line.startswith("[") and line.endswith("]"):
      flush()
      section, values = line[1:-1], {}
    elif "=" in line and not line.startswith(";"):
      k, v = line.split("=", 1)
      values[k.strip()] = v.strip()
  flush()
  return config


def load(database_dir: Union[str, Path]) -> EvowareConfig:
  """Load ``Carrier.cfg`` and ``evoware.inf`` from an EVOware ``database`` folder."""
  d = Path(database_dir)
  config = parse_carrier_cfg((d / "Carrier.cfg").read_text(encoding="latin-1"))
  return parse_evoware_inf((d / "evoware.inf").read_text(encoding="latin-1"), config)


def parse_ewt_layout(text: str) -> Dict[int, Tuple[int, List[Tuple[int, str, str]]]]:
  """Worktable (.ewt / embedded in .esc): {grid: (carrier id, [(site, labware, label)])}.

  Grid numbering as in EVOware scripts (first grid = 1).
  """
  lines = text.splitlines()
  start = next(n for n, line in enumerate(lines) if line.startswith("999;"))
  ids = lines[start + 1].split(";")
  carriers = {index - 1: int(c) for index, c in enumerate(ids) if c not in ("", "-1")}
  layout: Dict[int, Tuple[int, List[Tuple[int, str, str]]]] = {}
  j, block = start + 2, 0
  while j < len(lines) and lines[j].startswith("998;") and block <= len(ids):
    parts = lines[j].split(";")
    n = int(parts[1]) if parts[1].isdigit() else 0
    if block >= 1 and n > 0:
      labels = lines[j + 1].split(";")[1 : 1 + n]
      grid = block
      items = [
        (s + 1, name, label)
        for s, (name, label) in enumerate(zip(parts[2 : 2 + n], labels))
        if name
      ]
      if grid in carriers:
        layout[grid] = (carriers[grid], items)
      j += 2
    else:
      j += 1
    block += 1
  for grid, ident in carriers.items():
    layout.setdefault(grid, (ident, []))
  return layout
