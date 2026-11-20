"""Offline tests for the EVO X coordinator against a simulated controller."""

import asyncio
import itertools
import random
import unittest

from pylabrobot.legacy.liquid_handling.backends.tecan.EVO_backend import EVOBackend
from pylabrobot.legacy.liquid_handling.backends.tecan.x_coordinator import (
  EvoXCoordinator,
  XMoveFault,
)
from pylabrobot.legacy.liquid_handling.backends.tecan.x_layout import (
  EVO_ARMS,
  EVO_SEPARATIONS,
  arms_from_separations,
  evo_planner,
)


class FakeEvo:
  """Simulated controller. ``lag`` = polls before a commanded X move completes.

  Every X position the arms occupy (including intermediate positions while a
  move is in progress) is checked against the measured separations.
  """

  Z_RANGE = {"C5": 2100, "W1": 2370, "C1": 2540}

  def __init__(self, x=None, lag=0):
    self.x = dict(x or {"C5": 45, "W1": 6000, "C1": 9000})
    self.z = {"C5": [2100] * 8, "W1": [2370], "C1": [2464]}
    self.lag = lag
    self.pending = {}
    self.sent = []
    self.stuck = set()
    self.drift = None  # (module, delta) applied on next read of that module
    self.jitter = None  # random.Random -> readbacks of X are +-1 around the true value
    self.violations = []

  def _check(self):
    order = ["C5", "W1", "C1"]
    for (a, b), s in zip(zip(order, order[1:]), EVO_SEPARATIONS):
      if self.x[b] - self.x[a] < s:
        self.violations.append((a, b, self.x[a], self.x[b]))

  def _advance(self, module):
    if module in self.pending and module not in self.stuck:
      target, left = self.pending[module]
      if left <= 0:
        self.x[module] = target
        del self.pending[module]
      else:
        step = (target - self.x[module]) // (left + 1)
        self.x[module] += step
        self.pending[module] = (target, left - 1)
      self._check()

  async def send_command(self, module, command, params=None, **kwargs):
    self.sent.append((module, command, params))
    if command in ("PAX", "PAA", "PAZ"):
      self.in_motion_at_send = getattr(self, "in_motion_at_send", [])
      self.in_motion_at_send.append((module, command, dict(self.pending)))
    if command in ("RPX", "RPA", "RPZ"):
      self._advance(module)
      if self.drift and self.drift[0] == module:
        self.x[module] += self.drift[1]
        self.drift = None
      sel = params[0]
      if command == "RPZ":
        data = [self.Z_RANGE[module]] * len(self.z[module]) if sel == 5 else list(self.z[module])
      x = self.x[module] + (self.jitter.choice([-1, 0, 1]) if self.jitter and sel == 0 else 0)
      if command == "RPZ":
        pass
      elif command == "RPX":
        data = [x]
      else:
        z = self.Z_RANGE[module] if sel == 5 else self.z[module][0]
        data = [x, 1000, z, 690]
      return {"module": module, "data": data}
    if command in ("PAX", "PAA") and params and params[0] is not None:
      if self.lag:
        self.pending[module] = (params[0], self.lag)
      else:
        self.x[module] = params[0]
        self._check()
      if command == "PAA" and len(params) >= 3 and params[2] is not None:
        self.z[module] = [params[2]]
      return {"module": module, "data": []}
    if command == "PAA":  # MCA Z-only lift
      self.z[module] = [params[2]]
      return {"module": module, "data": []}
    if command == "PAZ":
      self.z[module] = list(params)
      return {"module": module, "data": []}
    raise AssertionError(f"unexpected command {module}{command}")


async def _no_sleep(_):
  await asyncio.sleep(0)


def coordinator(fake, **kw):
  kw.setdefault("z_travel", {"roma": 2464})
  kw.setdefault("sleep", _no_sleep)
  kw.setdefault("arrival_timeout", 5)
  return EvoXCoordinator(fake, evo_planner(), **kw)


def motion(fake):
  return [(m, c, p) for m, c, p in fake.sent if c in ("PAX", "PAA", "PAZ")]


class LayoutTests(unittest.TestCase):
  def test_extents_reproduce_measured_separations(self):
    arms = arms_from_separations(EVO_ARMS, EVO_SEPARATIONS)
    for (a, b), s in zip(zip(arms, arms[1:]), EVO_SEPARATIONS):
      self.assertEqual(a.right + b.left + 1, s)
    for arm in arms:
      for value in (arm.left, arm.right, arm.minimum, arm.maximum):
        self.assertIs(type(value), int)

  def test_rejects_bad_separations(self):
    with self.assertRaises(ValueError):
      arms_from_separations(EVO_ARMS, (1320,))
    with self.assertRaises(ValueError):
      arms_from_separations(EVO_ARMS, (1320, 1))


class CoordinatorTests(unittest.IsolatedAsyncioTestCase):
  async def test_mca_to_9100_moves_roma_first(self):
    fake = FakeEvo()
    moves = await coordinator(fake).move_x("mca", 9100)
    self.assertEqual([(m.arm, m.target) for m in moves], [("roma", 11540), ("mca", 9100)])
    self.assertEqual(motion(fake), [("C1", "PAX", [11540]), ("W1", "PAA", [9100])])
    self.assertEqual(fake.x, {"C5": 45, "W1": 9100, "C1": 11540})
    self.assertEqual(fake.violations, [])

  async def test_plan_is_dry_run(self):
    fake = FakeEvo()
    moves = await coordinator(fake).plan("mca", 9100)
    self.assertEqual(len(moves), 2)
    self.assertEqual(motion(fake), [])

  async def test_unblocked_move_moves_only_requested_arm(self):
    fake = FakeEvo()
    await coordinator(fake).move_x("roma", 12000)
    self.assertEqual(motion(fake), [("C1", "PAX", [12000])])

  async def test_three_arm_chain(self):
    fake = FakeEvo()
    moves = await coordinator(fake).move_x("liha", 12000)
    self.assertEqual([m.arm for m in moves], ["roma", "mca", "liha"])
    self.assertEqual(fake.violations, [])

  async def test_leftward_push(self):
    fake = FakeEvo({"C5": 4000, "W1": 6000, "C1": 9000})
    moves = await coordinator(fake).move_x("roma", 6500)
    self.assertEqual([(m.arm, m.target) for m in moves],
                     [("liha", 2740), ("mca", 4060), ("roma", 6500)])
    self.assertEqual(fake.violations, [])

  async def test_clear_path_leaves_requested_arm(self):
    fake = FakeEvo()
    moves = await coordinator(fake).clear_path("mca", 9100)
    self.assertEqual([(m.arm, m.target) for m in moves], [("roma", 11540)])
    self.assertEqual(fake.x["W1"], 6000)

  async def test_impossible_request_sends_nothing(self):
    fake = FakeEvo()
    with self.assertRaises(XMoveFault):
      await coordinator(fake).move_x("mca", 14860)  # RoMa would exceed 17280
    self.assertEqual(motion(fake), [])

  async def test_reply_before_completion_waits_for_arrival(self):
    fake = FakeEvo(lag=4)
    await coordinator(fake).move_x("mca", 9100)
    self.assertEqual(fake.x, {"C5": 45, "W1": 9100, "C1": 11540})
    # No motion command is ever sent while another arm is still moving.
    self.assertEqual([p for _, _, p in fake.in_motion_at_send], [{}, {}])
    self.assertEqual(fake.violations, [])

  async def test_stuck_arm_times_out_and_latches(self):
    fake = FakeEvo(lag=2)
    fake.stuck.add("C1")
    times = itertools.count(0, 1.0)
    coord = coordinator(fake, clock=lambda: next(times), arrival_timeout=3)
    with self.assertRaises(XMoveFault) as ctx:
      await coord.move_x("mca", 9100)
    self.assertIn("did not arrive", str(ctx.exception))
    self.assertNotIn(("W1", "PAA", [9100]), fake.sent)
    sent = len(fake.sent)
    with self.assertRaisesRegex(XMoveFault, "latched"):
      await coord.move_x("liha", 100)
    self.assertEqual(len(fake.sent), sent)
    coord.acknowledge_fault()
    self.assertIsNone(coord.fault)

  async def test_unexpected_motion_of_stationary_arm_faults(self):
    fake = FakeEvo()
    coord = coordinator(fake)

    original = coord._wait_arrival

    async def wait_and_drift(arm, target):
      result = await original(arm, target)
      fake.drift = ("C5", 50)
      return result

    coord._wait_arrival = wait_and_drift
    with self.assertRaisesRegex(XMoveFault, "liha X"):
      await coord.move_x("mca", 9100)
    self.assertNotIn(("W1", "PAA", [9100]), fake.sent)

  async def test_lowered_blocker_is_raised_before_x_move(self):
    fake = FakeEvo()
    fake.z["C1"] = [1000]
    await coordinator(fake).move_x("mca", 9100)
    m = motion(fake)
    self.assertEqual(m[0], ("C1", "PAZ", [2464]))
    self.assertEqual(m[1], ("C1", "PAX", [11540]))

  async def test_lowered_arm_faults_without_auto_lift(self):
    fake = FakeEvo()
    fake.z["C1"] = [1000]
    with self.assertRaisesRegex(XMoveFault, "below travel height"):
      await coordinator(fake, auto_lift=False).move_x("mca", 9100)
    self.assertEqual(motion(fake), [])

  async def test_mca_and_liha_lift_commands(self):
    fake = FakeEvo({"C5": 45, "W1": 6000, "C1": 9000})
    fake.z["W1"] = [1500]
    fake.z["C5"] = [900] * 8
    await coordinator(fake).move_x("liha", 5000)
    m = motion(fake)
    self.assertIn(("W1", "PAA", [None, None, 2370]), m)
    self.assertIn(("C5", "PAZ", [2100] * 8), m)

  async def test_out_of_range_start_rejected(self):
    fake = FakeEvo({"C5": 45, "W1": 6000, "C1": 20000})
    with self.assertRaises(XMoveFault):
      await coordinator(fake).move_x("mca", 7000)
    self.assertEqual(motion(fake), [])

  async def test_non_integer_target_rejected(self):
    with self.assertRaises(ValueError):
      await coordinator(FakeEvo()).move_x("mca", 9100.0)

  async def test_random_requests_never_violate_separation(self):
    rng = random.Random(1234)
    for _ in range(300):
      liha = rng.randint(-140, 3000)
      mca = rng.randint(liha + 1320, liha + 6000)
      roma = rng.randint(mca + 2440, mca + 6000)
      if mca > 14860 or roma > 17280:
        continue
      fake = FakeEvo({"C5": liha, "W1": mca, "C1": roma}, lag=rng.choice([0, 3]))
      name, spec = rng.choice([("liha", EVO_ARMS[0]), ("mca", EVO_ARMS[1]), ("roma", EVO_ARMS[2])])
      target = rng.randint(spec.minimum, spec.maximum)
      try:
        await coordinator(fake).move_x(name, target)
      except XMoveFault:
        continue  # infeasible requests may be refused
      self.assertEqual(fake.violations, [], (name, target))
      self.assertEqual(fake.x[spec.module], target)


class JitterTests(unittest.IsolatedAsyncioTestCase):
  async def test_hardware_run2_regression_roma_reads_one_short(self):
    # Real run 2026-11-20: RoMa commanded 11540, then read 11539; the re-plan
    # from the raw reading wanted RoMa to move again and the run faulted.
    fake = FakeEvo({"C5": 45, "W1": 7000, "C1": 10000})
    original = fake.send_command

    async def one_short(module, command, params=None, **kw):
      resp = await original(module, command, params)
      if module == "C1" and command == "RPX" and params == [0] and fake.x["C1"] == 11540:
        resp = {"module": "C1", "data": [11539]}
      return resp

    fake.send_command = one_short
    moves = await coordinator(fake, position_tolerance=1).move_x("mca", 9100)
    self.assertEqual([(m.arm, m.target) for m in moves], [("roma", 11540), ("mca", 9100)])
    self.assertEqual(motion(fake), [("C1", "PAX", [11540]), ("W1", "PAA", [9100])])

  async def test_random_jitter_with_tolerance(self):
    rng = random.Random(7)
    for _ in range(100):
      fake = FakeEvo(lag=rng.choice([0, 2]))
      fake.jitter = rng
      coord = coordinator(fake, position_tolerance=1)
      await coord.move_x("mca", 9100)
      await coord.move_x("liha", 12000)
      self.assertEqual(fake.violations, [])

  async def test_jitter_larger_than_tolerance_faults(self):
    fake = FakeEvo()
    fake.jitter = random.Random(3)
    with self.assertRaises(XMoveFault):
      for _ in range(20):
        await coordinator(fake, position_tolerance=0).move_x("roma", 12000)


class TravelHeightTests(unittest.IsolatedAsyncioTestCase):
  async def test_roma_at_park_height_is_not_lifted(self):
    fake = FakeEvo()  # RoMa Z 2464 < range 2540
    await coordinator(fake).move_x("roma", 12000)
    self.assertEqual(motion(fake), [("C1", "PAX", [12000])])

  async def test_without_travel_override_roma_counts_as_lowered(self):
    fake = FakeEvo()
    with self.assertRaisesRegex(XMoveFault, "below travel height"):
      await coordinator(fake, z_travel={}, auto_lift=False).move_x("roma", 12000)

  async def test_travel_height_above_range_rejected(self):
    with self.assertRaisesRegex(XMoveFault, "exceeds Z range"):
      await coordinator(FakeEvo(), z_travel={"roma": 3000}).move_x("roma", 12000)


class BackendHookTests(unittest.TestCase):
  def test_backend_creates_single_coordinator(self):
    backend = EVOBackend()
    coord = backend.x_coordinator(position_tolerance=2)
    self.assertIs(backend.x_coordinator(), coord)
    self.assertEqual(coord.position_tolerance, 2)
    with self.assertRaises(ValueError):
      backend.x_coordinator(position_tolerance=3)


if __name__ == "__main__":
  unittest.main()
