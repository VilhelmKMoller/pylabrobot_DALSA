"""X-axis collision avoidance for the EVO arms (LiHa / MCA / RoMa).

One coordinator per backend. Every X move requested through it:

1. reads fresh X (and Z) of every registered arm (RPX / RPA),
2. plans with ``x_clearance.Planner`` (blocking arms are pushed outward by the
   smallest amount that restores the measured pairwise separation),
3. for each planned step: ensures the moving arm is at its Z travel height
   (raises it if allowed), sends an X-only move, waits for arrival by polling
   the position readback, then re-reads every arm and checks nothing else moved,
4. on any surprise latches a fault and stops. Nothing is retried.

Scope: only moves that go through this coordinator are protected. ``setup()``
homing, raw ``send_command`` calls and the legacy movement helpers bypass it.
``stop()`` / USB teardown is not a physical halt.
"""

import asyncio
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

from .x_clearance import Arm, Move, Observation, Planner, Snapshot

RECOVERY_CHOICES = "Choose: (1) home the robot, (2) continue after recovery, or (3) abort the run."


class XMoveFault(RuntimeError):
  """Raised when an X move cannot be completed safely; the coordinator is latched."""

  def __init__(self, message: str, moves: Tuple[Move, ...] = (), completed: int = 0):
    super().__init__(f"{message}. Motion stopped; coordinator latched. {RECOVERY_CHOICES}")
    self.moves = moves
    self.completed = completed


@dataclass(frozen=True)
class ArmState:
  x: int
  z: Tuple[int, ...]


class EvoXCoordinator:
  """Sequential, fault-latched X coordinator for one EVO backend.

  Args:
    backend: object with ``async send_command(module, command, params)``.
    planner: X planner (see ``x_layout.evo_planner``).
    poll_interval: seconds between position reads while waiting for arrival.
    arrival_timeout: seconds before an unfinished move is treated as a fault.
    position_tolerance: allowed |readback - target| in 1/10 mm for arrival and
      for "unchanged" checks of stationary arms.
    settle_reads: consecutive in-tolerance reads required to accept arrival.
    auto_lift: raise an arm below its Z travel height before moving it in X;
      if False such a move is a fault instead.
    z_tolerance: an arm counts as raised when Z >= its travel height - z_tolerance.
    z_travel: per-arm Z travel height {arm name: Z}; arms not listed use their
      Z range (fully raised). Lifting moves the arm to this height.
    log: optional callable(event: str, **fields) for a durable transcript.
  """

  def __init__(
    self,
    backend,
    planner: Planner,
    *,
    poll_interval: float = 0.05,
    arrival_timeout: float = 60.0,
    position_tolerance: int = 0,
    settle_reads: int = 2,
    auto_lift: bool = True,
    z_tolerance: int = 50,
    z_travel: Optional[Dict[str, int]] = None,
    log: Optional[Callable[..., None]] = None,
    sleep: Callable[[float], "asyncio.Future"] = asyncio.sleep,
    clock: Callable[[], float] = time.monotonic,
  ):
    if not isinstance(planner, Planner):
      raise TypeError("planner must be an x_clearance.Planner")
    for arm in planner.arms:
      if type(arm.module) is not str or len(arm.module) != 2:
        raise ValueError("EVO module addresses must be two-character strings")
    if position_tolerance < 0 or z_tolerance < 0 or settle_reads < 1:
      raise ValueError("invalid tolerance settings")
    self.backend = backend
    self.planner = planner
    self.poll_interval = poll_interval
    self.arrival_timeout = arrival_timeout
    self.position_tolerance = position_tolerance
    self.settle_reads = settle_reads
    self.auto_lift = auto_lift
    self.z_tolerance = z_tolerance
    self._log = log or (lambda event, **fields: None)
    self._sleep = sleep
    self._clock = clock
    self.lock = asyncio.Lock()
    self.fault: Optional[XMoveFault] = None
    self._z_range: Dict[str, int] = {}
    # Reference X per arm: (value, allowed deviation). Readbacks jitter by a
    # unit or so. A commanded position allows +-tolerance; a reference taken
    # from a reading (itself jittered) allows +-2*tolerance. While a reading is
    # within that band, planning uses the reference, so jitter cannot make a
    # just-cleared arm look "1 unit short" or a stationary arm look moved.
    self._reference: Dict[str, Tuple[int, int]] = {}
    self.z_travel = dict(z_travel or {})
    for name, value in self.z_travel.items():
      self.arm(name)
      if type(value) is not int:
        raise ValueError("Z travel heights must be integers")

  # ------------------------------------------------------------------ helpers

  def arm(self, name: str) -> Arm:
    for arm in self.planner.arms:
      if arm.name == name:
        return arm
    raise ValueError(f"unknown arm {name!r}")

  async def _cmd(self, arm: Arm, command: str, params: Optional[List[Optional[int]]] = None):
    resp = await self.backend.send_command(module=arm.module, command=command, params=params)
    if not isinstance(resp, dict) or resp.get("module", arm.module) != arm.module:
      raise ValueError(f"unexpected response to {arm.module}{command}: {resp!r}")
    data = resp.get("data", [])
    if not isinstance(data, list):
      raise ValueError(f"malformed response to {arm.module}{command}: {resp!r}")
    return data

  @staticmethod
  def _ints(data, count: Optional[int], what: str) -> List[int]:
    if any(type(v) is not int for v in data) or (count is not None and len(data) < count) or not data:
      raise ValueError(f"malformed {what} readback: {data!r}")
    return list(data if count is None else data[:count])

  async def _read_x(self, arm: Arm, selector: int = 0) -> int:
    if arm.kind == "MCA":
      return self._ints(await self._cmd(arm, "RPA", [selector]), 4, "MCA RPA")[0]
    return self._ints(await self._cmd(arm, "RPX", [selector]), 1, "RPX")[0]

  async def _read_z(self, arm: Arm, selector: int = 0) -> Tuple[int, ...]:
    if arm.kind == "MCA":
      return (self._ints(await self._cmd(arm, "RPA", [selector]), 4, "MCA RPA")[2],)
    return tuple(self._ints(await self._cmd(arm, "RPZ", [selector]), None, "RPZ"))

  async def z_range(self, arm: Arm) -> int:
    if arm.name not in self._z_range:
      self._z_range[arm.name] = min(await self._read_z(arm, 5))
    return self._z_range[arm.name]

  async def read_state(self) -> Tuple[Snapshot, Dict[str, ArmState]]:
    """Fresh X and Z of every arm. Freshness starts before each wire read."""
    observations, states = {}, {}
    for arm in self.planner.arms:
      started = self.planner.clock()
      x = await self._read_x(arm)
      z = await self._read_z(arm)
      observations[arm.name] = Observation(x, started, True)
      states[arm.name] = ArmState(x, z)
    model = {}
    for name, obs in observations.items():
      ref = self._reference.get(name)
      if ref is not None and abs(obs.x - ref[0]) <= ref[1]:
        model[name] = Observation(ref[0], obs.observed_at, True)
      else:
        self._reference[name] = (obs.x, 2 * self.position_tolerance)
        model[name] = obs
    snapshot = Snapshot(model)
    self._log("snapshot", positions={k: (v.x, list(v.z)) for k, v in states.items()},
              model={k: v.x for k, v in model.items()})
    self.planner.validate(snapshot)
    return snapshot, states

  @staticmethod
  def _check_unchanged(snapshot: Snapshot, expected: Dict[str, int]) -> None:
    for name, x in expected.items():
      if snapshot.positions[name].x != x:
        raise ValueError(f"{name} X is {snapshot.positions[name].x}, expected {x}")

  async def travel_height(self, arm: Arm) -> int:
    rng = await self.z_range(arm)
    top = self.z_travel.get(arm.name, rng)
    if top > rng:
      raise ValueError(f"{arm.name} Z travel height {top} exceeds Z range {rng}")
    return top

  async def _ensure_raised(self, arm: Arm, z_now: Tuple[int, ...]) -> None:
    top = await self.travel_height(arm)
    if min(z_now) >= top - self.z_tolerance:
      return
    if not self.auto_lift:
      raise ValueError(f"{arm.name} Z {list(z_now)} is below travel height {top}")
    self._log("lift", arm=arm.name, z=list(z_now), target=top)
    if arm.kind == "MCA":
      await self._cmd(arm, "PAA", [None, None, top])
    else:
      await self._cmd(arm, "PAZ", [top] * len(z_now))
    deadline = self._clock() + self.arrival_timeout
    while True:
      z = await self._read_z(arm)
      if min(z) >= top - self.z_tolerance:
        return
      if self._clock() > deadline:
        raise TimeoutError(f"{arm.name} did not reach Z travel height (Z={list(z)})")
      await self._sleep(self.poll_interval)

  async def _send_x(self, arm: Arm, target: int) -> None:
    if arm.kind == "MCA":
      await self._cmd(arm, "PAA", [target])
    else:
      await self._cmd(arm, "PAX", [target])

  async def _wait_arrival(self, arm: Arm, target: int) -> int:
    """Poll position until it is within tolerance for ``settle_reads`` reads."""
    deadline = self._clock() + self.arrival_timeout
    good, last = 0, None
    while True:
      x = await self._read_x(arm)
      last = x
      good = good + 1 if abs(x - target) <= self.position_tolerance else 0
      if good >= self.settle_reads:
        return x
      if self._clock() > deadline:
        raise TimeoutError(f"{arm.name} did not arrive at X={target} (last X={last})")
      await self._sleep(self.poll_interval)

  # --------------------------------------------------------------- public API

  def acknowledge_fault(self) -> None:
    """Operator has resolved the fault (homed / inspected); re-enable moves."""
    self._log("fault_acknowledged", fault=str(self.fault))
    self.fault = None

  async def plan(self, name: str, target: int) -> Tuple[Move, ...]:
    """Dry run from live readback: no motion commands are sent."""
    self.arm(name)
    if type(target) is not int:
      raise ValueError("target must be an integer firmware X coordinate")
    async with self.lock:
      snapshot, _ = await self.read_state()
      return self.planner.plan(snapshot, name, target)

  async def move_x(self, name: str, target: int) -> Tuple[Move, ...]:
    """Move one arm to ``target`` X, pushing blocking arms aside first."""
    return await self._run(name, target, include_target=True)

  async def clear_path(self, name: str, target: int) -> Tuple[Move, ...]:
    """Move only the blocking arms so ``name`` may travel to ``target``.

    The caller then performs its own move of ``name`` (e.g. a pipetting PAA)
    while holding no other motion in parallel.
    """
    return await self._run(name, target, include_target=False)

  async def _run(self, name: str, target: int, include_target: bool) -> Tuple[Move, ...]:
    self.arm(name)
    if type(target) is not int:
      raise ValueError("target must be an integer firmware X coordinate")
    async with self.lock:
      if self.fault is not None:
        raise XMoveFault(f"coordinator is latched by an earlier fault ({self.fault.args[0]})")
      moves: Tuple[Move, ...] = ()
      done = 0
      try:
        snapshot, states = await self.read_state()
        moves = self.planner.plan(snapshot, name, target)
        if not include_target:
          moves = tuple(m for m in moves if m.arm != name)
        for step in moves:
          if type(step.target) not in (int, float) or not float(step.target).is_integer():
            raise ValueError(f"non-integer planned target {step!r}")
        self._log("plan", request=[name, target], moves=[[m.arm, m.start, m.target] for m in moves])
        expected = {k: v.x for k, v in snapshot.positions.items()}
        for index, step in enumerate(moves):
          snapshot, states = await self.read_state()
          self._check_unchanged(snapshot, expected)
          remaining = self.planner.plan(snapshot, name, target)
          if not include_target:
            remaining = tuple(m for m in remaining if m.arm != name)
          if [(m.arm, m.target) for m in remaining] != [(m.arm, m.target) for m in moves[index:]]:
            raise ValueError("plan changed before step; refusing to continue")
          arm = self.arm(step.arm)
          await self._ensure_raised(arm, states[arm.name].z)
          goal = int(step.target)
          self._log("move", arm=arm.name, module=arm.module, start=step.start, target=goal)
          await self._send_x(arm, goal)
          arrived = await self._wait_arrival(arm, goal)
          self._log("arrived", arm=arm.name, x=arrived)
          expected[arm.name] = goal
          self._reference[arm.name] = (goal, self.position_tolerance)
          snapshot, states = await self.read_state()
          self._check_unchanged(snapshot, expected)
          done += 1
        return moves
      except BaseException as exc:
        if isinstance(exc, (asyncio.CancelledError, KeyboardInterrupt)):
          self.fault = XMoveFault(f"interrupted ({type(exc).__name__})", moves, done)
          self._reference.clear()
          self._log("fault", error=str(self.fault), completed=done)
          raise
        if isinstance(exc, XMoveFault):
          fault = exc
        else:
          fault = XMoveFault(f"{type(exc).__name__}: {exc}", moves, done)
          fault.__cause__ = exc
        self.fault = fault
        self._reference.clear()
        self._log("fault", error=str(fault), completed=done)
        raise fault
