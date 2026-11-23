"""In-memory-only X coordination experiment, NOT a hardware integration.

No backend, transport, connection, real completion, safety interlock or physical
approval is implemented. Trace records are semantic sketches, not wire frames.
Coordinates and pairwise separations are provisional operator reports, not geometry.
The mock instantaneously completes its own moves; firmware completion is unknown.
"""
import asyncio
import time

from .x_clearance import Arm, Move, Observation, Planner, SequentialExecutor, Snapshot


class MockCoordinator:
  """Self-contained fake controller. Never accepts a transport or callback.

  ``enabled=False`` deliberately bypasses separation planning in the simulation;
  it is not a safety switch for any real robot. A fault permanently latches this
  instance. Creating another mock is not a physical recovery procedure.
  """

  def __init__(self, positions, *, enabled=True):
    if type(enabled) is not bool:
      raise ValueError('enabled must be bool')
    self.enabled = enabled
    self.trace = []
    self.faulted = False
    self._positions = dict(positions)
    self._lock = asyncio.Lock()
    # Nonphysical interval decomposition: neighbour totals 1320 and 2440.
    self.planner = Planner((
      Arm('liha', 'C5', 'LiHa', 1, 1319, -140, 13540),
      Arm('mca', 'W1', 'MCA', 0, 2439, 1180, 14860),
      Arm('roma', 'C1', 'RoMa', 0, 1, 3620, 17280),
    ), margin=1, max_age=10, clock=time.monotonic)

  async def _read(self):
    observations = {}
    for arm in self.planner.arms:
      command = 'RPA' if arm.kind == 'MCA' else 'RPX'
      self.trace.append((arm.module, command, (0,)))
      observations[arm.name] = Observation(self._positions[arm.name], time.monotonic(), True)
    return Snapshot(observations)

  async def _move(self, arm, step):
    command = 'PAA' if arm.kind == 'MCA' else 'PAX'
    args = (step.target, None, None, None) if arm.kind == 'MCA' else (step.target,)
    self.trace.append((arm.module, command, args))
    self._positions[arm.name] = step.target

  async def _approve(self, step, remaining):
    # Simulated approval only; never represents an operator decision.
    return True

  async def request(self, name, target):
    async with self._lock:
      if self.faulted:
        raise RuntimeError('mock fault is latched')
      try:
        if type(target) is not int:
          raise ValueError('mock target must be an integer')
        if self.enabled:
          # Outer lock also guards the fault latch and disabled path.
          executor = SequentialExecutor(self.planner, asyncio.Lock(), self._read, self._move, self._approve)
          return await executor.execute(name, target)
        state = await self._read()
        self.planner.validate(state)
        arm = next((a for a in self.planner.arms if a.name == name), None)
        if arm is None or type(target) is not int or not arm.minimum <= target <= arm.maximum:
          raise ValueError('invalid mock request')
        step = Move(name, state.positions[name].x, target)
        await self._move(arm, step)
        after = await self._read()
        expected = {key: value.x for key, value in state.positions.items()}
        expected[name] = target
        if {key: value.x for key, value in after.positions.items()} != expected:
          raise ValueError('mock readback differs from expected positions')
        return (step,)
      except BaseException:
        self.faulted = True
        raise
