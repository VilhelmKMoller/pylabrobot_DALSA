import unittest
import asyncio
from unittest.mock import AsyncMock

from pylabrobot.legacy.liquid_handling.backends.tecan.x_mock_integration import MockCoordinator


class MockIntegrationTests(unittest.IsolatedAsyncioTestCase):
  async def test_mca_request_displaces_roma_first(self):
    rig = MockCoordinator({'liha': 45, 'mca': 6000, 'roma': 9000})
    moves = await rig.request('mca', 9000)
    self.assertEqual([(m.arm, m.target) for m in moves], [('roma', 11440), ('mca', 9000)])
    self.assertEqual([e for e in rig.trace if e[1] in ('PAX', 'PAA')],
                     [('C1', 'PAX', (11440,)), ('W1', 'PAA', (9000, None, None, None))])

  async def test_disabled_bypasses_planning_but_readbacks_are_fresh(self):
    rig = MockCoordinator({'liha': 45, 'mca': 6000, 'roma': 9000}, enabled=False)
    await rig.request('mca', 9000)
    self.assertEqual([e[1] for e in rig.trace], ['RPX', 'RPA', 'RPX', 'PAA', 'RPX', 'RPA', 'RPX'])

  async def test_bad_target_does_not_issue_motion_and_latches(self):
    rig = MockCoordinator({'liha': 45, 'mca': 6000, 'roma': 9000})
    with self.assertRaises(ValueError):
      await rig.request('mca', 14860)
    self.assertFalse(any(e[1] in ('PAA', 'PAX') for e in rig.trace))
    with self.assertRaisesRegex(RuntimeError, 'latched'):
      await rig.request('mca', 7000)

  async def test_fault_after_motion_stops_and_latches(self):
    rig = MockCoordinator({'liha': 45, 'mca': 6000, 'roma': 9000})
    original = rig._read
    count = 0
    async def fail_after_move():
      nonlocal count
      count += 1
      if count == 3:
        raise TimeoutError('mock read fault')
      return await original()
    rig._read = fail_after_move
    with self.assertRaises(TimeoutError):
      await rig.request('mca', 9000)
    self.assertEqual([e[0] for e in rig.trace if e[1] in ('PAX', 'PAA')], ['C1'])
    with self.assertRaisesRegex(RuntimeError, 'latched'):
      await rig.request('liha', 100)

  async def test_request_validation_rejects_fractional_firmware_target(self):
    rig = MockCoordinator({'liha': 45, 'mca': 6000, 'roma': 9000})
    with self.assertRaises(ValueError):
      await rig.request('liha', 100.5)
    self.assertFalse(any(e[1] in ('PAA', 'PAX') for e in rig.trace))

  async def test_concurrent_requests_serialize(self):
    rig = MockCoordinator({'liha': 45, 'mca': 6000, 'roma': 9000})
    results = await asyncio.gather(rig.request('liha', 100), rig.request('liha', 200))
    self.assertEqual([m[0].start for m in results], [45, 100])

  async def test_rejected_approval_latches_without_motion(self):
    rig = MockCoordinator({'liha': 45, 'mca': 6000, 'roma': 9000})
    rig._approve = AsyncMock(return_value=False)
    with self.assertRaises(PermissionError):
      await rig.request('mca', 9000)
    self.assertTrue(rig.faulted)
    self.assertFalse(any(e[1] in ('PAA', 'PAX') for e in rig.trace))

  async def test_leftward_chain_and_firmware_queries(self):
    rig = MockCoordinator({'liha': 2000, 'mca': 4000, 'roma': 9000})
    moves = await rig.request('roma', 3620)
    self.assertEqual([(m.arm, m.target) for m in moves],
                     [('liha', -140), ('mca', 1180), ('roma', 3620)])
    self.assertEqual([e[0:2] for e in rig.trace if e[1] in ('PAX', 'PAA')],
                     [('C5', 'PAX'), ('W1', 'PAA'), ('C1', 'PAX')])

  async def test_final_readback_mismatch_latches(self):
    rig = MockCoordinator({'liha': 45, 'mca': 6000, 'roma': 9000})
    original_move = rig._move
    async def wrong_position(arm, step):
      await original_move(arm, step)
      rig._positions[arm.name] += 1
    rig._move = wrong_position
    with self.assertRaisesRegex(ValueError, 'differs'):
      await rig.request('liha', 100)
    self.assertTrue(rig.faulted)

  async def test_no_transport_injection_interface(self):
    with self.assertRaises(TypeError):
      MockCoordinator({'liha': 45, 'mca': 6000, 'roma': 9000}, transport=object())

  async def test_disabled_still_checks_completion_readback(self):
    rig = MockCoordinator({'liha': 45, 'mca': 6000, 'roma': 9000}, enabled=False)
    original_move = rig._move
    async def wrong_position(arm, step):
      await original_move(arm, step)
      rig._positions[arm.name] += 1
    rig._move = wrong_position
    with self.assertRaisesRegex(ValueError, 'differs'):
      await rig.request('liha', 100)
    self.assertTrue(rig.faulted)

  async def test_incomplete_snapshot_fails_closed(self):
    rig = MockCoordinator({'liha': 45, 'mca': 6000})
    with self.assertRaises(KeyError):
      await rig.request('liha', 100)
    self.assertTrue(rig.faulted)
    self.assertFalse(any(e[1] in ('PAA', 'PAX') for e in rig.trace))
