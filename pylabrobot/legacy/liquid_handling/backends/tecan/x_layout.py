"""EVO arm X layout built from operator-measured pairwise separations.

The operator measured (2026-11-11) the closest acceptable X distance between
neighbouring arms, clearance already included, and each arm's permitted X range.
These are *pairwise* separations, not individual arm widths. The planner in
``x_clearance`` works with per-arm extents, so each separation ``s`` is split
into integer extents ``right(left arm) + left(right arm) = s - margin``; the
planner then enforces exactly ``gap >= s`` between neighbours. Non-adjacent
arms cannot meet without first meeting the arm between them (fixed rail order),
so adjacent separations are sufficient.

All values are firmware X coordinates in 1/10 mm (same frame for all arms,
confirmed by the operator on 2026-11-19).
"""

from dataclasses import dataclass
import time
from typing import Callable, Optional, Sequence, Tuple

from .x_clearance import Arm, Planner

# Integer margin so every planned target is an integer firmware coordinate.
MARGIN = 1


@dataclass(frozen=True)
class ArmSpec:
  name: str
  module: str
  kind: str
  minimum: int
  maximum: int


# Left-to-right rail order with the operator-reported permitted X ranges.
EVO_ARMS: Tuple[ArmSpec, ...] = (
  ArmSpec("liha", "C5", "LiHa", -140, 13540),
  ArmSpec("mca", "W1", "MCA", 1180, 14860),
  ArmSpec("roma", "C1", "RoMa", 3620, 17280),
)

# Z travel heights for X moves. RoMa travels at its existing PLR park height
# (``EVOBackend._park_roma`` uses Z=2464; its Z range is 2540). LiHa and MCA
# travel fully raised (their Z range), so they are not listed.
EVO_Z_TRAVEL = {"roma": 2464}

# Minimum reference-X distance between neighbours (clearance included).
# MCA-RoMa was 2440 at the minimum end and 2420 at the maximum end; the
# larger (more conservative) value is used everywhere.
# LiHa-MCA: the recorded 1320 was too small. On 2026-11-23 the LiHa (Y -348, moving right to
# X 11347) bumped the MCA (X 12667, Y 1000) and stopped with X drive error 10 at X 10797: contact at
# a separation of about 1870. 2200 = that contact plus ~33 mm clearance (operator-observed bump).
EVO_SEPARATIONS: Tuple[int, ...] = (2200, 2440)


def arms_from_separations(
  specs: Sequence[ArmSpec], separations: Sequence[int], margin: int = MARGIN
) -> Tuple[Arm, ...]:
  """Convert pairwise separations into planner extents (all integers)."""
  if len(separations) != len(specs) - 1:
    raise ValueError("need exactly one separation per neighbouring pair")
  if type(margin) is not int or margin <= 0:
    raise ValueError("margin must be a positive integer")
  left = [0] * len(specs)
  right = [0] * len(specs)
  for i, s in enumerate(separations):
    if type(s) is not int or s <= margin:
      raise ValueError("separations must be integers larger than the margin")
    budget = s - margin
    right[i] = budget // 2
    left[i + 1] = budget - budget // 2
  return tuple(
    Arm(spec.name, spec.module, spec.kind, left[i], right[i], spec.minimum, spec.maximum)
    for i, spec in enumerate(specs)
  )


def evo_planner(
  max_age: float = 5.0,
  clock: Optional[Callable[[], float]] = None,
  specs: Sequence[ArmSpec] = EVO_ARMS,
  separations: Sequence[int] = EVO_SEPARATIONS,
) -> Planner:
  """Planner for the installed LiHa / MCA / RoMa layout."""
  return Planner(
    arms_from_separations(specs, separations),
    margin=MARGIN,
    max_age=max_age,
    clock=clock or time.monotonic,
  )
