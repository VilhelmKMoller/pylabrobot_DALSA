"""Offline backend entry point tests; transport is never opened."""
import asyncio
import time
import unittest
from unittest.mock import AsyncMock

from pylabrobot.legacy.liquid_handling.backends.tecan.EVO_backend import EVOBackend
from pylabrobot.legacy.liquid_handling.backends.tecan.x_clearance import Arm, Planner


class BackendXPlannerTests(unittest.IsolatedAsyncioTestCase):
  def setUp(self):
    self.backend = EVOBackend()
    self.planner = Planner((Arm('liha', 'C5', 'LiHa', 1, 1, 0, 100),
                            Arm('mca', 'W1', 'MCA', 1, 1, 0, 100)),
                           margin=1, max_age=10, clock=time.monotonic)
    async def reply(module, command, params):
      return {'module': module, 'data': [10] if module == 'C5' else [30, 0, 0, 0]}
    self.backend.send_command = AsyncMock(side_effect=reply)

  async def test_nonempty_plan_is_blocked_before_any_motion(self):
    with self.assertRaisesRegex(RuntimeError, 'completion') as caught:
      await self.backend.move_x_only(self.planner, 'liha', 40)
    self.assertEqual([(s.arm, s.target) for s in caught.exception.moves],
                     [('mca', 43), ('liha', 40)])
    self.assertEqual([(c.kwargs['module'], c.kwargs['command'])
                      for c in self.backend.send_command.await_args_list],
                     [('C5', 'RPX'), ('W1', 'RPA')])

  async def test_noop_reads_all_arms_and_returns_empty_plan(self):
    self.assertEqual(await self.backend.move_x_only(self.planner, 'liha', 10), ())
    self.assertEqual(self.backend.send_command.await_count, 2)

  async def test_read_failure_latches_future_requests(self):
    self.backend.send_command.side_effect = TimeoutError('read failed')
    with self.assertRaises(TimeoutError):
      await self.backend.move_x_only(self.planner, 'liha', 10)
    with self.assertRaisesRegex(RuntimeError, 'latched'):
      await self.backend.move_x_only(self.planner, 'liha', 10)
    self.assertEqual(self.backend.send_command.await_count, 1)

  async def test_malformed_mca_response_fails_closed(self):
    self.backend.send_command.side_effect = [{'module':'C5','data':[10]}, {'module':'W1','data':[30]}]
    with self.assertRaises(ValueError):
      await self.backend.move_x_only(self.planner, 'liha', 10)
    with self.assertRaisesRegex(RuntimeError, 'latched'):
      await self.backend.move_x_only(self.planner, 'liha', 10)

  async def test_real_backend_framing_with_mocked_io(self):
    backend = EVOBackend()
    backend.io.write = AsyncMock()
    backend.io.read = AsyncMock(side_effect=[b'\x02C5\x8010\x00', b'\x02W1\x8030,0,0,0\x00'])
    with self.assertRaisesRegex(RuntimeError, 'completion'):
      await backend.move_x_only(self.planner, 'liha', 40)
    self.assertEqual([call.args[0] for call in backend.io.write.await_args_list],
                     [b'\x02C5RPX0\x00', b'\x02W1RPA0\x00'])

  async def test_cancellation_latches(self):
    self.backend.send_command.side_effect = asyncio.CancelledError()
    with self.assertRaises(asyncio.CancelledError):
      await self.backend.move_x_only(self.planner, 'liha', 10)
    with self.assertRaisesRegex(RuntimeError, 'latched'):
      await self.backend.move_x_only(self.planner, 'liha', 10)

  async def test_configuration_change_latches(self):
    await self.backend.move_x_only(self.planner, 'liha', 10)
    other = Planner(self.planner.arms, margin=2, max_age=10, clock=time.monotonic)
    with self.assertRaisesRegex(ValueError, 'configuration'):
      await self.backend.move_x_only(other, 'liha', 10)
    with self.assertRaisesRegex(RuntimeError, 'latched'):
      await self.backend.move_x_only(self.planner, 'liha', 10)

  async def test_concurrent_requests_are_serialized(self):
    entered = asyncio.Event()
    release = asyncio.Event()
    async def reply(module, command, params):
      if module == 'C5' and not entered.is_set():
        entered.set()
        await release.wait()
      return {'module': module, 'data': [10] if module == 'C5' else [30, 0, 0, 0]}
    self.backend.send_command.side_effect = reply
    first = asyncio.create_task(self.backend.move_x_only(self.planner, 'liha', 10))
    await entered.wait()
    second = asyncio.create_task(self.backend.move_x_only(self.planner, 'liha', 10))
    await asyncio.sleep(0)
    self.assertEqual(self.backend.send_command.await_count, 1)
    release.set()
    self.assertEqual(await asyncio.gather(first, second), [(), ()])
