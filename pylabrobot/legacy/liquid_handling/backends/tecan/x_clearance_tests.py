"""Synthetic coordinates only; no hardware, transport or instrument configuration."""

import unittest
from typing import Any, Dict

from pylabrobot.legacy.liquid_handling.backends.tecan import x_clearance as xc


def arm(
  name: Any = "left",
  module: Any = 17,
  kind: Any = "LiHa",
  left: Any = 2,
  right: Any = 3,
  minimum: Any = 0,
  maximum: Any = 100,
):
  return xc.Arm(name, module, kind, left, right, minimum, maximum)


def snapshot(positions: Any, at: Any = 10, trustworthy: Any = True):
  return xc.Snapshot({name: xc.Observation(x, at, trustworthy) for name, x in positions.items()})


class PlannerTests(unittest.TestCase):
  def test_registry_rejects_invalid_configuration(self):
    invalid_arms: Any = [
      (),
      (arm(), arm()),
      (arm(), arm("other")),
      (arm(kind="unknown"),),
      (arm(name=""),),
      (arm(module=True),),
      (arm(module=-1),),
      (arm(module=1.5),),
      (arm(left=-1),),
      (arm(right=-1),),
      (arm(left=0, right=0),),
      (arm(minimum=5, maximum=4),),
    ]
    for value in (float("nan"), float("inf"), -float("inf"), True, "1", None):
      for field in ("left", "right", "minimum", "maximum"):
        invalid_arms.append((arm(**{field: value}),))
    for arms in invalid_arms:
      with self.subTest(arms=arms), self.assertRaises(ValueError):
        xc.Planner(arms, margin=1, max_age=2, clock=lambda: 10)
    for field in ("margin", "max_age"):
      for value in (0, -1, float("nan"), float("inf"), True, "1", None):
        args: Dict[str, Any] = dict(margin=1, max_age=2, clock=lambda: 10)
        args[field] = value
        with self.subTest(field=field, value=value), self.assertRaises(ValueError):
          xc.Planner((arm(),), **args)

  def test_registry_and_snapshot_copy_mutable_inputs(self):
    from dataclasses import FrozenInstanceError

    arms: Any = [arm()]
    planner = xc.Planner(arms, margin=1, max_age=2, clock=lambda: 10)
    positions = {"left": xc.Observation(10, 10, True)}
    state: Any = xc.Snapshot(positions)
    arms.clear()
    positions.clear()
    self.assertEqual(planner.arms, (arm(),))
    self.assertEqual(planner.plan(state, "left", 20), (xc.Move("left", 10, 20),))
    with self.assertRaises(TypeError):
      state.positions["left"] = xc.Observation(99, 10, True)
    with self.assertRaises(FrozenInstanceError):
      setattr(planner, "arms", ())
    with self.assertRaises(FrozenInstanceError):
      setattr(planner.arms[0], "left", 0)
    with self.assertRaises(FrozenInstanceError):
      setattr(state.positions["left"], "x", 0)

  def test_rejects_untrustworthy_incomplete_stale_or_invalid_snapshots(self):
    planner = xc.Planner((arm(), arm("right", 42, "RoMa")), 1, 2, lambda: 10)
    bad: Any = [
      snapshot({}),
      snapshot({"left": 10}),
      snapshot({"left": 10, "right": 30, "unknown": 50}),
      snapshot({"left": 10, "right": 30}, at=7.999),
      snapshot({"left": 10, "right": 30}, at=10.001),
      snapshot({"left": 10, "right": 30}, trustworthy=False),
      snapshot({"left": 10, "right": 30}, trustworthy=1),
      snapshot({"left": -1, "right": 30}),
      snapshot({"left": 10, "right": 101}),
      snapshot({"left": 10, "right": 15.5}),  # positive gap, insufficient margin
      snapshot({"left": 10, "right": 14}),
      snapshot({"left": 30, "right": 10}),
      xc.Snapshot({"left": None, "right": xc.Observation(30, 10, True)}),
    ]
    for value in (float("nan"), float("inf"), -float("inf"), True, "1", None, 10**400):
      bad.extend(
        (snapshot({"left": value, "right": 30}), snapshot({"left": 10, "right": 30}, at=value))
      )
    for state in bad:
      with self.subTest(state=state), self.assertRaises(ValueError):
        planner.plan(state, "left", 10)  # invalid even for a no-op
    self.assertEqual(planner.plan(snapshot({"left": 10, "right": 16}, at=8), "left", 10), ())
    invalid_times: Any = (float("nan"), float("inf"), True, "10", None, 10**400)
    for now in invalid_times:
      with self.subTest(now=now), self.assertRaises(ValueError):
        xc.Planner(planner.arms, 1, 2, lambda: now).plan(
          snapshot({"left": 10, "right": 30}), "left", 10
        )

  def test_request_requires_registered_instance_and_finite_in_limit_target(self):
    planner = xc.Planner((arm(),), 1, 2, lambda: 10)
    state = snapshot({"left": 10})
    invalid_targets: Any = (
      float("nan"),
      float("inf"),
      -float("inf"),
      True,
      "2",
      None,
      10**400,
      -0.001,
      100.001,
    )
    for target in invalid_targets:
      with self.subTest(target=target), self.assertRaises(ValueError):
        planner.plan(state, "left", target)
    invalid_names: Any = ("unknown", "LiHa", 17, None, [])
    for name in invalid_names:
      with self.subTest(name=name), self.assertRaises(ValueError):
        planner.plan(state, name, 20)
    for target in (0, 100):
      self.assertEqual(planner.plan(state, "left", target), (xc.Move("left", 10, target),))

  def assert_sequential_paths_clear(self, planner, state, moves):
    # Independent swept-interval oracle: stationary intervals cannot intersect any
    # moving interval inflated by the configured margin, including intermediate X.
    from fractions import Fraction as F

    positions = {name: obs.x for name, obs in state.positions.items()}
    by_name = {a.name: a for a in planner.arms}
    for move in moves:
      moving = by_name[move.arm]
      self.assertEqual(positions[move.arm], move.start)
      self.assertLessEqual(moving.minimum, min(move.start, move.target))
      self.assertGreaterEqual(moving.maximum, max(move.start, move.target))
      low = F(min(move.start, move.target)) - F(moving.left)
      high = F(max(move.start, move.target)) + F(moving.right)
      for other in planner.arms:
        if other.name == move.arm:
          continue
        other_low = F(positions[other.name]) - F(other.left)
        other_high = F(positions[other.name]) + F(other.right)
        self.assertTrue(
          high + F(planner.margin) <= other_low or other_high + F(planner.margin) <= low
        )
      positions[move.arm] = move.target
    return positions

  def test_pushes_right_chain_minimally_farthest_first_for_each_configuration(self):
    for kinds in (("LiHa", "RoMa"), ("LiHa", "LiHa", "RoMa"), ("LiHa", "MCA", "RoMa")):
      with self.subTest(kinds=kinds):
        arms = tuple(
          arm(f"instance-{i}", module=100 + 7 * i, kind=kind, left=2 + i, right=3 + i)
          for i, kind in enumerate(kinds)
        )
        planner = xc.Planner(arms, 1, 2, lambda: 10)
        state = snapshot({a.name: 10 + 20 * i for i, a in enumerate(arms)})
        expected_positions: list[float] = [60]
        for left, right in zip(arms, arms[1:]):
          expected_positions.append(expected_positions[-1] + left.right + right.left + 1)
        expected = tuple(
          xc.Move(arms[i].name, state.positions[arms[i].name].x, expected_positions[i])
          for i in reversed(range(len(arms)))
        )
        moves = planner.plan(state, arms[0].name, 60)
        self.assertEqual(moves, expected)
        final = self.assert_sequential_paths_clear(planner, state, moves)
        self.assertEqual([final[a.name] for a in arms], expected_positions)

  def test_pushes_left_chain_minimally_farthest_first(self):
    arms = (
      arm("one", 83, left=1, right=4),
      arm("two", 7, left=3, right=2),
      arm("three", 216, "RoMa", left=6, right=1),
    )
    planner = xc.Planner(arms, 2, 2, lambda: 10)
    state = snapshot({"one": 30, "two": 50, "three": 70})
    two_target = 20 - arms[2].left - arms[1].right - planner.margin
    one_target = two_target - arms[1].left - arms[0].right - planner.margin
    moves = planner.plan(state, "three", 20)
    self.assertEqual(
      moves,
      (xc.Move("one", 30, one_target), xc.Move("two", 50, two_target), xc.Move("three", 70, 20)),
    )
    self.assert_sequential_paths_clear(planner, state, moves)
    # A middle instance pushes only in the travel direction; the other stays put.
    for name, target, untouched in (("two", 80, "one"), ("two", 15, "three")):
      moves = planner.plan(state, name, target)
      final = self.assert_sequential_paths_clear(planner, state, moves)
      self.assertEqual(final[untouched], state.positions[untouched].x)
      self.assertEqual(final[name], target)

  def test_impossible_chain_is_atomic_at_either_limit(self):
    planner = xc.Planner((arm("a", 1), arm("b", 2), arm("c", 3, "RoMa")), 1, 2, lambda: 10)
    state = snapshot({"a": 20, "b": 40, "c": 60})
    before = dict(state.positions)
    for name, target in (("a", 95), ("c", 5)):
      with self.subTest(name=name), self.assertRaisesRegex(ValueError, "impossible"):
        planner.plan(state, name, target)
      self.assertEqual(dict(state.positions), before)
    # An exact limiting chain fits; moving past it by a small amount does not.
    for name, target in (("a", 100 - 2 * 6), ("c", 2 * 6)):
      self.assert_sequential_paths_clear(planner, state, planner.plan(state, name, target))
    # A pinned neighbour cannot be displaced even when the requested arm has travel.
    pinned = xc.Planner((arm("a", 1), arm("b", 2, minimum=40, maximum=40)), 1, 2, lambda: 10)
    with self.assertRaises(ValueError):
      pinned.plan(snapshot({"a": 20, "b": 40}), "a", 35)

  def test_exact_clearance_does_not_move_a_neighbour(self):
    planner = xc.Planner((arm("a", 1), arm("b", 2, "RoMa")), 1, 2, lambda: 10)
    state = snapshot({"a": 10, "b": 40})
    moves = planner.plan(state, "a", 40 - 6)
    self.assertEqual(moves, (xc.Move("a", 10, 40 - 6),))
    self.assert_sequential_paths_clear(planner, state, moves)

  def test_positive_margin_is_not_lost_to_float_rounding(self):
    import math

    base = float(2**53)
    arms = (
      arm("a", 1, left=0.25, right=0.25, minimum=-base * 2, maximum=base * 2),
      arm("b", 2, "RoMa", left=0.25, right=0.25, minimum=-base * 2, maximum=base * 2),
    )
    planner = xc.Planner(arms, 0.25, 2, lambda: 10)
    state = snapshot({"a": base, "b": base + 2})
    moves = planner.plan(state, "a", base + 2)
    self.assertEqual(moves[0].target, math.nextafter(base + 2, math.inf))
    self.assert_sequential_paths_clear(planner, state, moves)
    state = snapshot({"a": -base - 2, "b": -base})
    moves = planner.plan(state, "b", -base - 2)
    self.assertEqual(moves[0].target, math.nextafter(-base - 2, -math.inf))
    self.assert_sequential_paths_clear(planner, state, moves)

  def test_exhaustive_small_configurations_match_independent_feasibility_oracle(self):
    for kinds in (("LiHa", "RoMa"), ("LiHa", "LiHa", "RoMa"), ("LiHa", "MCA", "RoMa")):
      arms = tuple(
        arm(f"arm-{i}", 71 + i * 13, kind, left=1 + i, right=3 - i, maximum=60)
        for i, kind in enumerate(kinds)
      )
      planner = xc.Planner(arms, 2, 2, lambda: 10)
      initial: list[float] = [5 + i * 15 for i in range(len(arms))]
      state = snapshot(dict(reversed([(a.name, x) for a, x in zip(arms, initial)])))
      for index, requested in enumerate(arms):
        for target in range(-1, 62):
          with self.subTest(kinds=kinds, index=index, target=target):
            expected: list[float] = initial.copy()
            expected[index] = target
            if target > initial[index]:
              for j in range(index + 1, len(arms)):
                expected[j] = max(
                  expected[j], expected[j - 1] + arms[j - 1].right + arms[j].left + 2
                )
            else:
              for j in reversed(range(index)):
                expected[j] = min(
                  expected[j], expected[j + 1] - arms[j + 1].left - arms[j].right - 2
                )
            if any(not a.minimum <= x <= a.maximum for a, x in zip(arms, expected)):
              with self.assertRaises(ValueError):
                planner.plan(state, requested.name, target)
            else:
              moves = planner.plan(state, requested.name, target)
              final = self.assert_sequential_paths_clear(planner, state, moves)
              self.assertEqual([final[a.name] for a in arms], expected)
              self.assertEqual(len(moves), sum(a != b for a, b in zip(initial, expected)))

  def test_snapshot_freshness_is_per_arm_and_rechecked_with_injected_time(self):
    now: list[float] = [10]
    planner = xc.Planner((arm("a", 1), arm("b", 2)), 1, 2, lambda: now[0])
    state = xc.Snapshot({"a": xc.Observation(10, 8, True), "b": xc.Observation(30, 10, True)})
    planner.validate(state)
    now[0] = 10.001
    with self.assertRaises(ValueError):
      planner.plan(state, "b", 40)

  def test_extreme_finite_requests_cannot_round_clearance_outside_limits(self):
    import sys

    bound = sys.float_info.max
    arms = (
      arm("a", 1, left=0.25, right=0.25, minimum=-bound, maximum=bound),
      arm("b", 2, "RoMa", left=0.25, right=0.25, minimum=-bound, maximum=bound),
    )
    planner = xc.Planner(arms, 0.25, 2, lambda: 10)
    for state, name, target in (
      (snapshot({"a": 0, "b": bound}), "a", bound),
      (snapshot({"a": -bound, "b": 0}), "b", -bound),
    ):
      with self.subTest(name=name), self.assertRaises(ValueError):
        planner.plan(state, name, target)
    # A very large finite extent sum must not overflow unnoticed at registration.
    with self.assertRaises(ValueError):
      xc.Planner((arm(left=bound, right=bound),), 1, 2, lambda: 10)

  def test_snapshot_float_rounding_cannot_hide_insufficient_clearance(self):
    base = float(2**53)
    planner = xc.Planner(
      (
        arm("a", 1, left=0.25, right=0.25, maximum=base * 2),
        arm("b", 2, left=0.25, right=0.25, maximum=base * 2),
      ),
      1.75,
      2,
      lambda: 10,
    )
    # Reference gap 2 minus two extents .25 gives 1.5, not the rounded gap 2.
    with self.assertRaises(ValueError):
      planner.plan(snapshot({"a": base, "b": base + 2}), "a", base)

  def test_registry_preserves_explicit_string_module_addresses_without_inference(self):
    arms = (
      arm("liha-primary", "module-left"),
      arm("liha-secondary", "module-middle"),
      arm("unloaded-gripper", "module-right", "RoMa"),
    )
    planner = xc.Planner(arms, 1, 2, lambda: 10)
    self.assertEqual(
      tuple(a.module for a in planner.arms), ("module-left", "module-middle", "module-right")
    )
    state = snapshot({a.name: 10 + 20 * i for i, a in enumerate(arms)})
    self.assert_sequential_paths_clear(planner, state, planner.plan(state, arms[0].name, 60))
    with self.assertRaises(ValueError):
      xc.Planner((arm("a", "same"), arm("b", "same")), 1, 2, lambda: 10)
    for address in ("", "  ", None, [], {}, False):
      with self.subTest(address=address), self.assertRaises(ValueError):
        xc.Planner((arm(module=address),), 1, 2, lambda: 10)

  def test_snapshot_age_comparison_cannot_round_stale_time_to_zero(self):
    planner = xc.Planner((arm(),), 1, 0.5, lambda: 2**53 + 1)
    with self.assertRaisesRegex(ValueError, "stale"):
      planner.plan(snapshot({"left": 10}, at=float(2**53)), "left", 10)

  def test_single_arm_request_is_pure_and_sequential(self):
    planner = xc.Planner((arm(),), margin=1, max_age=2, clock=lambda: 10)
    state = snapshot({"left": 10})
    plan = planner.plan(state, "left", 40)
    self.assertEqual(plan, (xc.Move("left", 10, 40),))
    self.assertEqual(state.positions["left"].x, 10)
    self.assertEqual(planner.plan(state, "left", 10), ())


class ExecutorTests(unittest.IsolatedAsyncioTestCase):
  async def asyncSetUp(self):
    import asyncio

    self.lock = asyncio.Lock()
    self.planner = xc.Planner((arm("a", 1), arm("b", 2, "RoMa")), 1, 2, lambda: 10)
    self.positions = {"a": 10, "b": 30}
    self.events = []

  async def read(self):
    self.assertTrue(self.lock.locked())
    self.events.append("read")
    return snapshot(self.positions)

  async def approve(self, move, remaining):
    self.assertTrue(self.lock.locked())
    self.assertEqual(move, remaining[0])
    self.events.append(("approve", move.arm))
    return True

  async def move(self, config, step):
    self.assertTrue(self.lock.locked())
    self.assertEqual(config.name, step.arm)
    self.events.append(("move", step.arm))
    self.assertEqual(self.positions[step.arm], step.start)
    self.positions[step.arm] = step.target

  def executor(self, **overrides):
    args: Dict[str, Any] = dict(
      planner=self.planner, lock=self.lock, read=self.read, move=self.move, approve=self.approve
    )
    args.update(overrides)
    return xc.SequentialExecutor(**args)

  async def test_approval_requires_literal_true_before_every_individual_move(self):
    for denied_step in (1, 2):
      for answer in (False, None, 1, "yes"):
        with self.subTest(denied_step=denied_step, answer=answer):
          self.positions = {"a": 10, "b": 30}
          self.events = []
          count = 0

          async def approve(step, remaining):
            nonlocal count
            count += 1
            return answer if count == denied_step else True

          with self.assertRaises(PermissionError):
            await self.executor(approve=approve).execute("a", 40)
          self.assertEqual(
            [e for e in self.events if isinstance(e, tuple)],
            [] if denied_step == 1 else [("move", "b")],
          )
          self.assertFalse(self.lock.locked())

  async def test_changed_state_after_approval_invalidates_plan_before_movement(self):
    for changed in ("a", "b"):
      with self.subTest(changed=changed):
        self.positions = {"a": 10, "b": 30}
        self.events = []

        async def approve(step, remaining):
          self.positions[changed] += 1
          return True

        with self.assertRaisesRegex(ValueError, "readback"):
          await self.executor(approve=approve).execute("a", 40)
        self.assertEqual([e for e in self.events if isinstance(e, tuple)], [])

  async def test_mismatch_after_move_stops_before_next_approval(self):
    for changed in ("a", "b"):
      with self.subTest(changed=changed):
        self.positions = {"a": 10, "b": 30}
        self.events = []

        async def inaccurate_move(config, step):
          await self.move(config, step)
          self.positions[changed] += 1

        with self.assertRaisesRegex(ValueError, "readback"):
          await self.executor(move=inaccurate_move).execute("a", 40)
        self.assertEqual(
          [e for e in self.events if isinstance(e, tuple)], [("approve", "b"), ("move", "b")]
        )

  async def test_callback_faults_propagate_without_retry_or_next_step(self):
    import asyncio

    for stage in ("read", "approve", "move"):
      for error_type in (RuntimeError, TimeoutError, asyncio.CancelledError):
        with self.subTest(stage=stage, error=error_type):
          self.positions = {"a": 10, "b": 30}
          self.events = []
          calls = 0
          failure = error_type("injected offline fault")

          async def fail(*args):
            nonlocal calls
            calls += 1
            raise failure

          with self.assertRaises(error_type) as caught:
            await self.executor(**{stage: fail}).execute("a", 40)
          self.assertIs(caught.exception, failure)
          self.assertEqual(calls, 1)
          self.assertEqual(self.positions, {"a": 10, "b": 30})
          self.assertFalse(self.lock.locked())

  async def test_stale_or_untrusted_readback_blocks_each_stage(self):
    for failing_read in (1, 2, 3, 4, 5):
      for invalid in ("stale", "unknown", "incomplete"):
        with self.subTest(failing_read=failing_read, invalid=invalid):
          self.positions = {"a": 10, "b": 30}
          self.events = []
          count = 0

          async def read():
            nonlocal count
            count += 1
            if count == failing_read:
              if invalid == "stale":
                return snapshot(self.positions, at=7)
              if invalid == "unknown":
                return snapshot(self.positions, trustworthy=False)
              return snapshot({"a": self.positions["a"]})
            return await self.read()

          with self.assertRaises(ValueError):
            await self.executor(read=read).execute("a", 40)
          self.assertEqual(count, failing_read)
          self.assertEqual(
            sum(isinstance(e, tuple) and e[0] == "move" for e in self.events),
            0 if failing_read < 3 else 1 if failing_read < 5 else 2,
          )
          self.assertFalse(self.lock.locked())

  async def test_impossible_request_never_requests_approval_or_moves(self):
    with self.assertRaises(ValueError):
      await self.executor().execute("a", 100)
    self.assertEqual(self.events, ["read"])

  async def test_noop_still_reads_and_validates_without_approval(self):
    self.assertEqual(await self.executor().execute("a", 10), ())
    self.assertEqual(self.events, ["read"])

  async def test_shared_lock_serializes_two_executor_instances(self):
    import asyncio

    entered = asyncio.Event()
    release = asyncio.Event()

    async def delayed_move(config, step):
      entered.set()
      await release.wait()
      await self.move(config, step)

    first = asyncio.create_task(self.executor(move=delayed_move).execute("a", 20))
    await entered.wait()
    second = asyncio.create_task(self.executor().execute("b", 40))
    try:
      await asyncio.sleep(0)
      self.assertFalse(second.done())
      self.assertEqual(self.events, ["read", ("approve", "a"), "read"])
    finally:
      release.set()
      await asyncio.gather(first, second)
    self.assertEqual(self.positions, {"a": 20, "b": 40})
    self.assertEqual(
      self.events,
      [
        "read",
        ("approve", "a"),
        "read",
        ("move", "a"),
        "read",
        "read",
        ("approve", "b"),
        "read",
        ("move", "b"),
        "read",
      ],
    )

  async def test_fault_after_partial_simulated_motion_does_not_retry_or_resume(self):
    async def partial_move(config, step):
      self.events.append(("partial", step.arm))
      self.positions[step.arm] = (step.start + step.target) / 2
      raise TimeoutError("completion unknown")

    with self.assertRaises(TimeoutError):
      await self.executor(move=partial_move).execute("a", 40)
    self.assertEqual(self.events, ["read", ("approve", "b"), "read", ("partial", "b")])
    self.assertEqual(self.positions["a"], 10)
    self.assertFalse(self.lock.locked())

  async def test_executes_approved_sequence_under_shared_lock_with_each_readback(self):
    expected = self.planner.plan(snapshot(self.positions), "a", 40)
    executed = await self.executor().execute("a", 40)
    self.assertEqual(executed, expected)
    self.assertEqual(self.positions, {"a": 40, "b": 46})
    self.assertEqual(
      self.events,
      [
        "read",
        ("approve", "b"),
        "read",
        ("move", "b"),
        "read",
        ("approve", "a"),
        "read",
        ("move", "a"),
        "read",
      ],
    )
    self.assertFalse(self.lock.locked())


if __name__ == "__main__":
  unittest.main()
