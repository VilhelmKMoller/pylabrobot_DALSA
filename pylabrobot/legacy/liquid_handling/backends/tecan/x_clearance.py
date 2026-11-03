"""Offline X-only interval planning and an injected sequential simulation harness.

No hardware adapter, geometry defaults, speed settings, or movement API integration.
Existing EVO setup/movement calls bypass this module. Passing these checks does not
establish instrument safety: the model omits Y/Z/R, tools, deck obstacles, firmware
behaviour and physical stopping. All coordinates must share one verified unit/frame.
"""

import asyncio
from dataclasses import dataclass
from fractions import Fraction as F
import math
from types import MappingProxyType
from typing import Awaitable, Callable, Mapping, Tuple, Union


def _finite(value: float, label: str) -> None:
  try:
    valid = type(value) in (int, float) and math.isfinite(value)
  except OverflowError:
    valid = False
  if not valid:
    raise ValueError(f"{label} must be a finite int or float (not bool)")


def _outward(value: F, direction: int) -> float:
  """Round only outward, never spending the configured positive margin on rounding."""
  try:
    result = float(value)
  except OverflowError as exc:
    raise ValueError("unrepresentable clearance target") from exc
  if (F(result) - value) * direction < 0:
    result = math.nextafter(result, math.inf if direction > 0 else -math.inf)
  _finite(result, "clearance target")
  return result


@dataclass(frozen=True)
class Arm:
  """Instance configuration validated when registered in a Planner.

  At reference X the occupied interval is [X - left, X + right]. Extents are
  conservative X projections across the entire permitted tool/Y/Z/R envelope;
  limits constrain reference X, not interval edges. No geometry is inferred from
  kind or module. Module identifiers are opaque integers/strings, not wire commands.
  """

  name: str
  module: Union[int, str]
  kind: str
  left: float
  right: float
  minimum: float
  maximum: float


@dataclass(frozen=True)
class Observation:
  """Caller-attested X and acquisition time in the injected monotonic clock's epoch.

  A True flag is a contract with the readback provider, not proof of homing or an
  independent position measurement. Cached backend positions are not such proof.
  """

  x: float
  observed_at: float
  trustworthy: bool


@dataclass(frozen=True)
class Snapshot:
  """Defensively copied observations, indexed by instance name (not arm kind)."""

  positions: Mapping[str, Observation]

  def __post_init__(self) -> None:
    object.__setattr__(self, "positions", MappingProxyType(dict(self.positions)))


@dataclass(frozen=True)
class Move:
  """One absolute, sequential X segment; no speed or backend command is implied."""

  arm: str
  start: float
  target: float


@dataclass(frozen=True)
class Planner:
  """Pure planner with an explicit frozen left-to-right registry in ``arms``.

  Repeated kinds are allowed; instance names and module identifiers are unique.
  Clearance is at least the strictly positive margin; equality is allowed.
  The age bound is inclusive. All configuration must come from external validation.
  The clock must be injected (e.g. time.monotonic for a later validated provider).
  """

  arms: Tuple[Arm, ...]
  margin: float
  max_age: float
  clock: Callable[[], float]

  def __post_init__(self) -> None:
    object.__setattr__(self, "arms", tuple(self.arms))
    if not self.arms:
      raise ValueError("registry must contain arms")
    names, modules = set(), set()
    for arm in self.arms:
      if not isinstance(arm, Arm):
        raise ValueError("registry entries must be Arm configurations")
      if not isinstance(arm.name, str) or not arm.name.strip() or arm.name in names:
        raise ValueError("arm instance names must be nonempty and unique")
      if not (
        (type(arm.module) is int and arm.module >= 0)
        or (type(arm.module) is str and arm.module.strip())
      ):
        raise ValueError("module address must be a nonnegative integer or nonempty string")
      if arm.module in modules:
        raise ValueError("module addresses must be unique")
      if arm.kind not in ("LiHa", "MCA", "RoMa"):
        raise ValueError("unsupported arm kind")
      names.add(arm.name)
      modules.add(arm.module)
      for label in ("left", "right", "minimum", "maximum"):
        _finite(getattr(arm, label), label)
      if arm.left < 0 or arm.right < 0 or arm.left + arm.right <= 0:
        raise ValueError("extents must be nonnegative with positive total width")
      _finite(arm.left + arm.right, "total width")
      if arm.minimum > arm.maximum:
        raise ValueError("inverted reference-X limits")
    for label in ("margin", "max_age"):
      value = getattr(self, label)
      _finite(value, label)
      if value <= 0:
        raise ValueError(f"{label} must be positive")

  def validate(self, state: Snapshot) -> None:
    """Require a complete fresh trusted snapshot consistent with fixed arm order."""
    now = self.clock()
    _finite(now, "monotonic time")
    if not isinstance(state, Snapshot) or set(state.positions) != {a.name for a in self.arms}:
      raise ValueError("snapshot must contain exactly the registered instances")
    for arm in self.arms:
      observation = state.positions[arm.name]
      if not isinstance(observation, Observation) or observation.trustworthy is not True:
        raise ValueError(f"untrustworthy position: {arm.name}")
      _finite(observation.x, "observed X")
      _finite(observation.observed_at, "observation time")
      age = F(now) - F(observation.observed_at)
      if not 0 <= age <= F(self.max_age):
        raise ValueError(f"stale or future observation: {arm.name}")
      if not arm.minimum <= observation.x <= arm.maximum:
        raise ValueError(f"observed X outside limits: {arm.name}")
    for left, right in zip(self.arms, self.arms[1:]):
      gap = (
        F(state.positions[right.name].x)
        - F(right.left)
        - (F(state.positions[left.name].x) + F(left.right))
      )
      if gap < F(self.margin):
        raise ValueError("overlap, insufficient clearance, or changed arm order")

  def plan(self, state: Snapshot, name: str, target: float) -> Tuple[Move, ...]:
    """Preflight the entire chain, returning an immutable farthest-first sequence.

    Neighbours only move outward when required, to the least outward binary float
    satisfying clearance. Exact rational comparisons prevent float rounding from
    consuming margin. Every segment's complete swept interval is checked against
    every stationary arm; straight monotone travel and fixed order are required.
    Impossible requests raise ValueError without returning or applying any moves.
    """
    self.validate(state)
    _finite(target, "target X")
    if not isinstance(name, str) or name not in state.positions:
      raise ValueError("request must identify a registered arm instance")
    arm = next(a for a in self.arms if a.name == name)
    if not arm.minimum <= target <= arm.maximum:
      raise ValueError("target X outside limits")
    positions = {a.name: state.positions[a.name].x for a in self.arms}
    moves = []

    def displace(index: int, destination: float) -> None:
      moving = self.arms[index]
      start = positions[moving.name]
      if destination == start:
        return
      _finite(destination, "clearance target")
      if not moving.minimum <= destination <= moving.maximum:
        raise ValueError(f"impossible clearance chain at {moving.name}")
      if destination > start and index + 1 < len(self.arms):
        neighbour = self.arms[index + 1]
        required = F(destination) + F(moving.right) + F(neighbour.left) + F(self.margin)
        if F(positions[neighbour.name]) < required:
          displace(index + 1, _outward(required, 1))
      if destination < start and index > 0:
        neighbour = self.arms[index - 1]
        required = F(destination) - F(moving.left) - F(neighbour.right) - F(self.margin)
        if F(positions[neighbour.name]) > required:
          displace(index - 1, _outward(required, -1))
      low = F(min(start, destination)) - F(moving.left)
      high = F(max(start, destination)) + F(moving.right)
      for other in self.arms:
        if other.name == moving.name:
          continue
        other_low = F(positions[other.name]) - F(other.left)
        other_high = F(positions[other.name]) + F(other.right)
        if not (high + F(self.margin) <= other_low or other_high + F(self.margin) <= low):
          raise ValueError("swept path violates clearance")
      moves.append(Move(moving.name, start, destination))
      positions[moving.name] = destination

    displace(self.arms.index(arm), target)
    return tuple(moves)


@dataclass(frozen=True)
class SequentialExecutor:
  """Abstract callback harness for offline simulation; no hardware adapter is provided.

  All cooperating callers must share the injected lock. Approval receives the next
  step plus the entire remaining sequence and must return literal True. Readback
  after approval must exactly match expected positions for every arm; the entire
  remainder is then replanned and compared. Each move is awaited and followed by
  complete trusted/fresh readback, including the final move. No tolerance is guessed.

  Exceptions (including cancellation) propagate without retry or further steps.
  The lock is released, not an emergency halt: callers must keep the run paused,
  resolve uncertain state and explicitly authorize any new request. There is no
  global fault latch, timeout policy, physical stop, recovery, or automatic resume.
  A move callback must await actual completion, and a read callback must acquire
  trustworthy measurements, not make cached positions fresh by relabelling them.
  Callbacks must not re-enter this executor/lock. These contracts are not verified
  hardware implementations; do not attach an instrument without separate validation.
  """

  planner: Planner
  lock: asyncio.Lock
  read: Callable[[], Awaitable[Snapshot]]
  move: Callable[[Arm, Move], Awaitable[None]]
  approve: Callable[[Move, Tuple[Move, ...]], Awaitable[bool]]

  async def execute(self, name: str, target: float) -> Tuple[Move, ...]:
    """Plan from readback under the lock; execute only individually approved steps."""
    async with self.lock:
      state = await self.read()
      moves = self.planner.plan(state, name, target)
      expected = {key: observation.x for key, observation in state.positions.items()}

      async def read_expected() -> Snapshot:
        readback = await self.read()
        self.planner.validate(readback)
        if {key: observation.x for key, observation in readback.positions.items()} != expected:
          raise ValueError("readback differs from expected positions; remaining plan invalid")
        return readback

      for index, step in enumerate(moves):
        if await self.approve(step, moves[index:]) is not True:
          raise PermissionError("individual move was not explicitly approved")
        state = await read_expected()
        if self.planner.plan(state, name, target) != moves[index:]:
          raise ValueError("remaining plan changed during revalidation")
        arm = next(a for a in self.planner.arms if a.name == step.arm)
        await self.move(arm, step)
        expected[step.arm] = step.target
        await read_expected()
      return moves
