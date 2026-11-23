"""Offline tests only: real backend command framing/parsing over fake transport."""
import asyncio
import json
import time

import pytest

from .EVO_backend import TecanLiquidHandler
from .x_clearance import Arm, Planner
from .x_supervised import DurableTranscript, SupervisedXCoordinator


class FakeIO:
  def __init__(self):
    self.positions = {"C5": 0, "W1": 20, "C1": 40}
    self.commands = []
    self.response_override = None
    self.fail_read = False

  async def write(self, data, **kwargs):
    cmd = data[1:-1].decode()
    self.commands.append(cmd)
    self.module, self.command, arg = cmd[:2], cmd[2:5], cmd[5:]
    if self.command in ("PAX", "PAA"):
      self.positions[self.module] = int(arg)

  async def read(self, **kwargs):
    if self.fail_read:
      raise TimeoutError("fake timeout")
    if self.response_override is not None:
      return self.response_override
    data = []
    if self.command in ("RPX", "RPA"):
      data = [self.positions[self.module]] + ([0, 0, 0] if self.command == "RPA" else [])
    return b'\x02' + self.module.encode() + b'\x80' + ','.join(map(str, data)).encode() + b'\x00'


class Backend:
  _assemble_command = TecanLiquidHandler._assemble_command
  parse_response = TecanLiquidHandler.parse_response
  send_command = TecanLiquidHandler.send_command

  def __init__(self):
    self.io = FakeIO()
    self._cache = {}


def fixture(tmp_path, approve=None, confirm=None, clock=time.monotonic, max_age=10):
  backend = Backend()
  planner = Planner((Arm('liha', 'C5', 'LiHa', 1, 1, -100, 100),
                     Arm('mca', 'W1', 'MCA', 1, 1, -100, 100),
                     Arm('roma', 'C1', 'RoMa', 1, 1, -100, 100)),
                    margin=1, max_age=max_age, clock=clock)
  async def yes(_):
    return True
  file = (tmp_path / 'transcript.jsonl').open('w+')
  c = SupervisedXCoordinator(backend, planner, approve or yes, confirm or yes,
                             lock=asyncio.Lock(), transcript=DurableTranscript(file),
                             exclusive_control=True)
  return c, backend, file


def test_blocker_and_raw_transcript(tmp_path):
  async def run():
    seen = []
    async def confirm(step):
      seen.append(step.arm)
      return True
    c, backend, file = fixture(tmp_path, confirm=confirm)
    moves = await c.move_x('mca', 40)
    assert [m.arm for m in moves] == ['roma', 'mca']
    assert seen == ['roma', 'mca']
    assert [x for x in backend.io.io.commands if x[2:5] in ('PAA', 'PAX')] == ['C1PAX43', 'W1PAA40']
    file.seek(0)
    events = [json.loads(line) for line in file]
    assert any(e['event'] == 'tx' and bytes.fromhex(e['hex']) == b'\x02W1PAA40\x00' for e in events)
    assert any(e['event'] == 'rx' for e in events)
    file.close()
  asyncio.run(run())


def test_approval_pause_change_latches(tmp_path):
  async def run():
    async def approve(_):
      backend.io.io.positions['C5'] = 1
      return True
    c, backend, file = fixture(tmp_path, approve=approve)
    with pytest.raises(ValueError, match='changed'):
      await c.move_x('mca', 40)
    with pytest.raises(RuntimeError, match='latched'):
      await c.move_x('mca', 21)
    assert not any(x[2:5] in ('PAX', 'PAA') for x in backend.io.io.commands)
    file.close()
  asyncio.run(run())


@pytest.mark.parametrize('bad', [b'\x02C5\x801,2\x00', b'\x02C5\x80x\x00', b'\x02C1\x800\x00', b'\x02C5\x80\x00'])
def test_bad_readback_stops(tmp_path, bad):
  async def run():
    c, backend, file = fixture(tmp_path)
    backend.io.io.response_override = bad
    with pytest.raises(ValueError):
      await c.move_x('mca', 40)
    assert c.faulted
    assert len(backend.io.io.commands) == 1
    file.close()
  asyncio.run(run())


def test_slow_read_cannot_refresh_timestamp(tmp_path):
  async def run():
    times = iter([0, 0, 0, 11])
    c, backend, file = fixture(tmp_path, clock=lambda: next(times), max_age=10)
    with pytest.raises(ValueError, match='stale'):
      await c.move_x('mca', 40)
    assert len(backend.io.io.commands) == 3
    file.close()
  asyncio.run(run())


def test_read_failure_is_logged_and_latches(tmp_path):
  async def run():
    c, backend, file = fixture(tmp_path)
    backend.io.io.fail_read = True
    with pytest.raises(TimeoutError):
      await c.move_x('mca', 40)
    file.seek(0)
    assert [json.loads(x)['event'] for x in file] == ['tx', 'write_return', 'read_error']
    assert c.faulted
    file.close()
  asyncio.run(run())


def test_transcript_failure_prevents_send(tmp_path):
  async def run():
    c, backend, file = fixture(tmp_path)
    file.close()
    with pytest.raises(ValueError):
      await c.move_x('mca', 40)
    assert not backend.io.io.commands
    assert c.faulted
  asyncio.run(run())


def test_completion_rejection_stops_sequence(tmp_path):
  async def run():
    async def no(_):
      return False
    c, backend, file = fixture(tmp_path, confirm=no)
    with pytest.raises(RuntimeError, match='completion'):
      await c.move_x('mca', 40)
    assert [x for x in backend.io.io.commands if x[2:5] in ('PAX', 'PAA')] == ['C1PAX43']
    assert c.faulted
    file.close()
  asyncio.run(run())
