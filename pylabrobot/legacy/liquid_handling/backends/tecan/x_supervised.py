"""Experimental supervised X execution. Not a global interlock or physical stop.

No connection is opened here. The caller supplies validated geometry, a shared lock,
exclusive control and an already-open transcript file. All other backend access must
be excluded externally. A controller reply/readback is not independent proof of
physical completion; callbacks and the entire installation require validation.
"""
import asyncio
import json
import os
import time
from typing import Awaitable, Callable

from .x_clearance import Move, Observation, Snapshot


class DurableTranscript:
  """Append JSON lines and synchronously flush them to the supplied file descriptor."""

  def __init__(self, file):
    self.file = file

  def record(self, event, **fields):
    entry = {"event": event, "monotonic": time.monotonic(), **fields}
    self.file.write(json.dumps(entry, sort_keys=True) + "\n")
    self.file.flush()
    os.fsync(self.file.fileno())


class TranscriptIO:
  """Pass through the backend transport; persist exact bytes and I/O failures."""

  def __init__(self, io, transcript):
    self.io = io
    self.transcript = transcript

  def __getattr__(self, name):
    return getattr(self.io, name)

  async def write(self, data, *args, **kwargs):
    self.transcript.record("tx", hex=bytes(data).hex())  # durable before sending
    try:
      result = await self.io.write(data, *args, **kwargs)
    except BaseException as exc:
      self.transcript.record("write_error", error=repr(exc))
      raise
    self.transcript.record("write_return", result=repr(result))
    return result

  async def read(self, *args, **kwargs):
    try:
      data = await self.io.read(*args, **kwargs)
    except BaseException as exc:
      self.transcript.record("read_error", error=repr(exc))
      raise
    self.transcript.record("rx", hex=bytes(data).hex())
    return data


class SupervisedXCoordinator:
  """One fault-latched coordinator; no retry/reset or automatic recovery.

  Callbacks are async: approve(full_moves) and confirm_completed(step), each
  returning literal True. Completion confirmation must reflect actual observation.
  Freshness begins before each wire read, not after the response. Shared locking
  only covers users of the same lock; exclusive control is an external contract.
  """

  def __init__(self, backend, planner, approve: Callable[[tuple[Move, ...]], Awaitable[bool]],
               confirm_completed: Callable[[Move], Awaitable[bool]], *, lock,
               transcript, exclusive_control):
    if exclusive_control is not True or not callable(approve) or not callable(confirm_completed):
      raise ValueError("exclusive control and both callbacks are required")
    if not isinstance(lock, asyncio.Lock) or not callable(getattr(transcript, "record", None)):
      raise ValueError("shared lock and durable transcript are required")
    if isinstance(backend.io, TranscriptIO):
      raise ValueError("transport is already wrapped")
    if any(type(a.module) is not str or len(a.module) != 2 for a in planner.arms):
      raise ValueError("EVO module addresses must be two-character strings")
    self.backend, self.planner = backend, planner
    self.approve, self.confirm_completed = approve, confirm_completed
    self.lock, self.transcript = lock, transcript
    self.faulted = False
    self.io = TranscriptIO(backend.io, transcript)
    backend.io = self.io

  async def _command(self, arm, command, params, length):
    if self.backend.io is not self.io:
      raise RuntimeError("transcript transport was replaced")
    response = await self.backend.send_command(module=arm.module, command=command, params=params)
    if (type(response) is not dict or set(response) != {"module", "data"}
        or response["module"] != arm.module or type(response["data"]) is not list
        or len(response["data"]) != length
        or any(type(value) is not int for value in response["data"])):
      raise ValueError("unexpected controller response shape")
    return response["data"]

  async def _snapshot(self):
    positions = {}
    for arm in self.planner.arms:
      started = self.planner.clock()
      data = await self._command(arm, "RPA" if arm.kind == "MCA" else "RPX", [0],
                                 4 if arm.kind == "MCA" else 1)
      positions[arm.name] = Observation(data[0], started, True)
    state = Snapshot(positions)
    self.planner.validate(state)
    return state

  async def move_x(self, name, target):
    async with self.lock:
      if self.faulted:
        raise RuntimeError("coordinator fault latched; investigation required")
      try:
        if type(target) is not int:
          raise ValueError("target must be an integer controller coordinate")
        state = await self._snapshot()
        moves = self.planner.plan(state, name, target)
        if any(type(step.target) not in (int, float) or not float(step.target).is_integer()
               for step in moves):
          raise ValueError("non-integer clearance target")
        expected = {key: obs.x for key, obs in state.positions.items()}
        if await self.approve(moves) is not True:
          raise PermissionError("full sequence not approved")
        # Revalidate even an empty plan after an approval pause.
        state = await self._snapshot()
        if {k: v.x for k, v in state.positions.items()} != expected:
          raise ValueError("position changed during approval")
        if self.planner.plan(state, name, target) != moves:
          raise ValueError("plan changed during approval")
        for index, step in enumerate(moves):
          if index:
            state = await self._snapshot()
            if {k: v.x for k, v in state.positions.items()} != expected:
              raise ValueError("position changed before move")
            if self.planner.plan(state, name, target) != moves[index:]:
              raise ValueError("remaining plan changed")
          arm = next(a for a in self.planner.arms if a.name == step.arm)
          await self._command(arm, "PAA" if arm.kind == "MCA" else "PAX", [int(step.target)], 0)
          if await self.confirm_completed(step) is not True:
            raise RuntimeError("observed completion not confirmed")
          expected[step.arm] = step.target
          state = await self._snapshot()
          if {k: v.x for k, v in state.positions.items()} != expected:
            raise ValueError("post-move readback differs from expected")
        return moves
      except BaseException:
        self.faulted = True
        raise
