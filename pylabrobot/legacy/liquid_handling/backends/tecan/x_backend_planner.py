"""Experimental backend X planning: read-only; no physical movement is enabled.

Registry, geometry, homing and firmware completion are not validated here. Other
backend entry points do not share this lock or latch. Never treat a report as proof
of physical standstill, homing, or collision safety.
"""
import asyncio

from .x_clearance import Observation, Planner, Snapshot


class UnverifiedXCompletion(RuntimeError):
  """A nonempty plan was computed but cannot safely be dispatched."""

  def __init__(self, moves):
    super().__init__('X motion blocked: firmware motion completion is unverified')
    self.moves = moves


class BackendXPlanner:
  def __init__(self, backend, planner):
    if not isinstance(planner, Planner):
      raise TypeError('planner must be a Planner')
    self.backend = backend
    self.planner = planner
    self.lock = asyncio.Lock()
    self.faulted = False

  async def request(self, planner, name, target):
    async with self.lock:
      if self.faulted:
        raise RuntimeError('X planner fault is latched; no automatic recovery')
      try:
        if planner is not self.planner:
          raise ValueError('X planner configuration cannot be replaced on this backend')
        if type(target) is not int:
          raise ValueError('firmware X target must be an integer')
        observations = {}
        for arm in self.planner.arms:
          # Start time is conservative: transport latency counts towards age.
          acquired = self.planner.clock()
          command = 'RPA' if arm.kind == 'MCA' else 'RPX'
          response = await self.backend.send_command(module=arm.module, command=command, params=[0])
          if not isinstance(response, dict) or response.get('module') != arm.module:
            raise ValueError('unexpected X response module')
          data = response.get('data')
          expected = 4 if arm.kind == 'MCA' else 1
          if not isinstance(data, list) or len(data) != expected or any(type(x) is not int for x in data):
            raise ValueError('malformed X response')
          # Trust only means the value came from this request; it is not homing proof.
          observations[arm.name] = Observation(data[0], acquired, True)
        moves = self.planner.plan(Snapshot(observations), name, target)
        if moves:
          raise UnverifiedXCompletion(moves)
        return moves
      except BaseException:
        self.faulted = True
        raise
