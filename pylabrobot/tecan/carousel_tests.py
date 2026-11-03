"""Offline tests for the Tecan carousel driver: the transport is simulated, never opened."""

import asyncio
import unittest
from typing import List, Optional, Tuple

Script = List[Tuple[bytes, bytes]]


class ScriptedSerial:
  """A simulated serial transport that answers a scripted command/reply sequence.

  The script is a list of ``(command, reply)`` pairs, so a test states the exact bytes on the wire
  and fails if the driver sends anything else or sends too much. ``auto`` answers any command left
  after the script, for tests about unbounded polling.
  """

  def __init__(self, script: Script, auto: Optional[bytes] = None) -> None:
    self.expected = list(script)
    self.auto = auto
    self.writes: List[bytes] = []
    self.setup_count = 0
    self.stop_count = 0
    self._buffer = bytearray()

  async def setup(self) -> None:
    self.setup_count += 1

  async def stop(self) -> None:
    self.stop_count += 1

  async def reset_input_buffer(self) -> None:
    self._buffer.clear()

  async def reset_output_buffer(self) -> None:
    pass

  async def write(self, data: bytes) -> None:
    self.writes.append(data)
    if self.expected:
      command, reply = self.expected.pop(0)
      if command != data:
        raise AssertionError(f"Expected {command!r}, driver sent {data!r}")
    elif self.auto is not None:
      reply = self.auto
    else:
      raise AssertionError(f"Unexpected command {data!r}: the script is exhausted")
    self._buffer.extend(reply + b"\r\n")

  async def read(self, num_bytes: int = 1) -> bytes:
    if not self._buffer:
      return b""
    chunk = bytes(self._buffer[:num_bytes])
    del self._buffer[:num_bytes]
    return chunk


def make_device(script: Script, *, auto: Optional[bytes] = None, **kwargs):
  """Build a driver over a scripted transport, with the polling delays removed."""
  from pylabrobot.tecan import TecanCarousel

  io = ScriptedSerial(script, auto=auto)
  device = TecanCarousel(port="operator-selected", **kwargs)
  device._SETTLE_DELAY = 0.0  # type: ignore[attr-defined]
  device._POLL_INTERVAL = 0.0  # type: ignore[attr-defined]
  device.io = io  # type: ignore[assignment]
  return device, io


def handshake(ready=b"1", error=b"0") -> Script:
  """The frames every operation starts with: connect, then the ready/error preconditions."""
  return [(b"CR\r", b"CC"), (b"RD 1915\r", ready), (b"RD 1814\r", error)]


def selection(
  cartridge: int,
  level: int,
  *,
  current_cartridge: int = 1,
  current_level: Optional[int] = None,
) -> Script:
  """The frames of a rotation and lift, including the readbacks that confirm it.

  A register is only written when its current value differs, which is the driver's idempotence.
  ``current_level`` defaults to the requested level, so no ``DM5`` write is expected.
  """
  script: Script = []
  if current_cartridge != cartridge:
    script += [
      (b"RD DM0\r", f"{current_cartridge:05d}".encode()),
      (f"WR DM0 {cartridge}\r".encode(), b"OK"),
    ]
  else:
    script += [(b"RD DM0\r", f"{cartridge:05d}".encode())]
  if current_level is not None and current_level != level:
    script += [
      (b"RD DM5\r", f"{current_level:05d}".encode()),
      (f"WR DM5 {level}\r".encode(), b"OK"),
    ]
  else:
    script += [(b"RD DM5\r", f"{level:05d}".encode())]
  script += [
    (b"RD 1915\r", b"1"),
    (b"RD DM0\r", f"{cartridge:05d}".encode()),
    (b"RD DM1\r", f"{cartridge:05d}".encode()),
  ]
  return script


def selection_without_writes(cartridge: int, level: int) -> Script:
  """The frames of a rotation to where the carousel already is: no write is needed."""
  return selection(cartridge, level, current_cartridge=cartridge)


class SetupTests(unittest.IsolatedAsyncioTestCase):
  async def test_setup_opens_and_handshakes_without_initializing(self):
    device, io = make_device([(b"CR\r", b"CC")])
    await device.setup()
    await device.setup()
    self.assertEqual(io.writes, [b"CR\r"])
    self.assertEqual(io.setup_count, 1)

  async def test_stop_quits_the_conversation_and_closes(self):
    device, io = make_device([(b"CR\r", b"CC"), (b"CQ\r", b"CF")])
    await device.setup()
    await device.stop()
    self.assertEqual(io.writes, [b"CR\r", b"CQ\r"])
    self.assertEqual(io.stop_count, 1)

  async def test_stop_tolerates_a_rejected_quit(self):
    """An unsupported CQ must not leak the open port."""
    device, io = make_device([(b"CR\r", b"CC"), (b"CQ\r", b"E1")])
    await device.setup()
    await device.stop()
    self.assertEqual(io.stop_count, 1)

  async def test_stop_after_fault_skips_the_quit_frame(self):
    device, io = make_device([(b"CR\r", b"CC"), (b"RD DM0\r", b"OK")])
    await device.setup()
    with self.assertRaises(ValueError):  # Expected five digits.
      await device.get_selected_cartridge()
    await device.stop()
    self.assertEqual(io.writes, [b"CR\r", b"RD DM0\r"])

  async def test_failed_handshake_closes_and_requires_stop(self):
    for reply in (b"OK", b"CC!"):
      with self.subTest(reply=reply):
        device, io = make_device([(b"CR\r", reply)])
        with self.assertRaises((ValueError, TimeoutError)):
          await device.setup()
        self.assertEqual(io.stop_count, 1)
        with self.assertRaises(RuntimeError):
          await device.setup()
        await device.stop()

  async def test_reads_before_setup_are_refused(self):
    device, io = make_device([])
    with self.assertRaises(RuntimeError):
      await device.is_ready()
    self.assertEqual(io.writes, [])


class StatusTests(unittest.IsolatedAsyncioTestCase):
  async def test_individual_reads_use_exact_frames(self):
    script = [
      (b"CR\r", b"CC"),
      (b"RD 1915\r", b"1"),
      (b"RD 1814\r", b"0"),
      (b"RD 1801\r", b"1"),
      (b"RD 1815\r", b"0"),
      (b"RD 1812\r", b"1"),
      (b"RD 1813\r", b"0"),
      (b"RD DM0\r", b"00005"),
      (b"RD DM5\r", b"00022"),
    ]
    device, io = make_device(script)
    await device.setup()
    self.assertIs(await device.is_ready(), True)
    self.assertIs(await device.has_error(), False)
    self.assertIs(await device.is_active(), True)
    self.assertIs(await device.is_plate_ready(), False)
    self.assertIs(await device.is_plate_on_shovel(), True)
    self.assertIs(await device.is_plate_at_transfer_station(), False)
    self.assertEqual(await device.get_selected_cartridge(), 5)
    self.assertEqual(await device.get_selected_level(), 22)
    self.assertEqual(io.expected, [])

  async def test_status_snapshot_reads_every_register(self):
    script = [
      (b"CR\r", b"CC"),
      (b"RD 1915\r", b"1"),
      (b"RD 1814\r", b"0"),
      (b"RD 1801\r", b"1"),
      (b"RD 1815\r", b"0"),
      (b"RD 1812\r", b"0"),
      (b"RD 1813\r", b"0"),
      (b"RD DM0\r", b"00009"),
      (b"RD DM5\r", b"00022"),
      (b"RD DM200\r", b"09491"),
    ]
    device, io = make_device(script)
    await device.setup()
    status = await device.get_status()
    self.assertEqual(
      (status.ready, status.error, status.active, status.plate_ready), (True, False, True, False)
    )
    self.assertEqual((status.plate_on_shovel, status.plate_at_transfer_station), (False, False))
    self.assertEqual((status.cartridge, status.level, status.process_status), (9, 22, 9491))
    self.assertEqual(io.expected, [])

  async def test_controller_rejection_keeps_the_session(self):
    """E0-E5 refuse the command but leave the link in sync, so the session survives."""
    from pylabrobot.tecan import CarouselRejected

    device, io = make_device([(b"CR\r", b"CC"), (b"RD 1815\r", b"E1"), (b"RD 1915\r", b"1")])
    await device.setup()
    with self.assertRaises(CarouselRejected):
      await device.is_plate_ready()
    self.assertIs(await device.is_ready(), True)
    self.assertEqual(len(io.writes), 3)

  async def test_malformed_frames_fault_the_session(self):
    for reply in (b"2", b" 1", b"10", b""):
      with self.subTest(reply=reply):
        device, _ = make_device([(b"CR\r", b"CC"), (b"RD 1915\r", reply)])
        await device.setup()
        with self.assertRaises((ValueError, TimeoutError)):
          await device.is_ready()
        with self.assertRaises(RuntimeError):
          await device.is_ready()
        await device.stop()

  async def test_process_status_accepts_only_five_digits(self):
    for reply in (b"1", b"+0005", b"0000x", b"081920"):
      with self.subTest(reply=reply):
        device, _ = make_device([(b"CR\r", b"CC"), (b"RD DM200\r", reply)])
        await device.setup()
        with self.assertRaises(ValueError):
          await device.get_process_status()

  async def test_concurrent_reads_are_serialized(self):
    device, io = make_device([(b"CR\r", b"CC"), (b"RD 1915\r", b"1"), (b"RD 1814\r", b"0")])
    await device.setup()
    entered = asyncio.Event()
    release = asyncio.Event()
    original_read = io.read
    held = False

    async def read_byte(num_bytes):
      """Hold the first reply byte until the competing reader has queued."""
      nonlocal held
      if not held:
        held = True
        entered.set()
        await release.wait()
      return await original_read(num_bytes)

    io.read = read_byte  # type: ignore[assignment]
    first = asyncio.create_task(device.is_ready())
    await entered.wait()
    second = asyncio.create_task(device.has_error())
    await asyncio.sleep(0)
    self.assertEqual(io.writes, [b"CR\r", b"RD 1915\r"])
    release.set()
    self.assertEqual(await asyncio.gather(first, second), [True, False])


class HandlingTests(unittest.IsolatedAsyncioTestCase):
  async def test_select_cartridge_rotates_without_touching_the_lift(self):
    """Rotation writes DM0 only: DM5 is never read, written or validated."""
    script = handshake() + [(b"RD 1812\r", b"0"), (b"RD DM0\r", b"00001"), (b"RD DM1\r", b"00000")]
    script += [(b"WR DM0 3\r", b"OK"), (b"RD 1915\r", b"1")]
    script += [(b"RD DM0\r", b"00003"), (b"RD DM1\r", b"00003")]
    device, io = make_device(script)
    await device.setup()
    await device.select_cartridge(3)
    self.assertEqual(io.expected, [])
    self.assertFalse([write for write in io.writes if b"DM5" in write])

  def test_level_for_slot_maps_a_standard_cassette_one_to_one(self):
    """A 22-slot standard cassette addresses its slots as controller levels 1..22."""
    device, _ = make_device([])
    self.assertEqual([device.level_for_slot(2, slot) for slot in (1, 7, 22)], [1, 7, 22])
    self.assertEqual([device.slot_for_level(2, level) for level in (1, 7, 22)], [1, 7, 22])

  def test_level_for_slot_maps_a_deep_well_cassette_three_levels_per_slot(self):
    """Measured on hardware: a 7-slot deep-well cassette steps three controller levels per slot."""
    device, _ = make_device([])
    device.cassette_slots[1] = 7
    device.cassette_level_step[1] = 3
    self.assertEqual(
      [device.level_for_slot(1, slot) for slot in range(1, 8)], [1, 4, 7, 10, 13, 16, 19]
    )
    self.assertEqual(
      [device.slot_for_level(1, level) for level in (1, 4, 7, 10, 13, 16, 19)],
      [1, 2, 3, 4, 5, 6, 7],
    )

  def test_level_for_slot_rejects_slots_beyond_the_installed_cassette(self):
    device, _ = make_device([])
    device.cassette_slots[1] = 7
    device.cassette_level_step[1] = 3
    for slot in (0, 8, True, "1"):
      with self.subTest(slot=slot):
        with self.assertRaises(ValueError):
          device.level_for_slot(1, slot)  # type: ignore[arg-type]

  def test_slot_for_level_rejects_levels_between_physical_slots(self):
    device, _ = make_device([])
    device.cassette_slots[1] = 7
    device.cassette_level_step[1] = 3
    for level in (2, 3, 5, 20, 21, 22):
      with self.subTest(level=level):
        with self.assertRaises(ValueError):
          device.slot_for_level(1, level)

  async def test_select_cartridge_rewrites_a_stale_selection(self):
    """DM0 naming the target while DM1 reports another position still needs a fresh WR DM0."""
    script = handshake() + [(b"RD 1812\r", b"0"), (b"RD DM0\r", b"00003"), (b"RD DM1\r", b"00001")]
    script += [(b"WR DM0 3\r", b"OK"), (b"RD 1915\r", b"1")]
    script += [(b"RD DM0\r", b"00003"), (b"RD DM1\r", b"00003")]
    device, io = make_device(script)
    await device.setup()
    await device.select_cartridge(3)
    self.assertEqual(io.expected, [])
    self.assertIn(b"WR DM0 3\r", io.writes)

  async def test_select_cartridge_ignores_the_transfer_station_sensor(self):
    """A plate on the station does not block rotation, and the station is not even read."""
    script = handshake() + [(b"RD 1812\r", b"0"), (b"RD DM0\r", b"00001"), (b"RD DM1\r", b"00000")]
    script += [(b"WR DM0 3\r", b"OK"), (b"RD 1915\r", b"1")]
    script += [(b"RD DM0\r", b"00003"), (b"RD DM1\r", b"00003")]
    device, io = make_device(script)
    await device.setup()
    await device.select_cartridge(3)
    self.assertEqual(io.expected, [])
    self.assertFalse([write for write in io.writes if b"1813" in write])

  async def test_select_cartridge_refuses_a_plate_on_the_shovel(self):
    device, io = make_device(handshake() + [(b"RD 1812\r", b"1")])
    await device.setup()
    with self.assertRaises(Exception):
      await device.select_cartridge(3)
    self.assertFalse([write for write in io.writes if write.startswith(b"WR")])

  async def test_select_cartridge_rejects_addresses_outside_the_configuration(self):
    for cartridge in (0, 11, True, 1.0, "3"):
      with self.subTest(cartridge=cartridge):
        device, io = make_device([(b"CR\r", b"CC")])
        await device.setup()
        with self.assertRaises(ValueError):
          await device.select_cartridge(cartridge)  # type: ignore[arg-type]
        self.assertEqual(io.writes, [b"CR\r"])

  async def test_export_plate_sequence(self):
    script = handshake()
    script += [(b"RD 1812\r", b"0"), (b"RD 1813\r", b"0")]
    script += selection(9, 22, current_cartridge=3)
    script += [(b"ST 1905\r", b"OK"), (b"RD 1915\r", b"1")]
    script += [(b"RD 1812\r", b"0"), (b"RD 1813\r", b"1")]
    device, io = make_device(script)
    await device.setup()
    await device.export_plate(9, 22)
    self.assertEqual(io.expected, [])
    self.assertEqual(io.writes.count(b"ST 1905\r"), 1)
    self.assertNotIn(b"ST 1904\r", io.writes)

  async def test_export_polls_through_busy(self):
    script = handshake()
    script += [(b"RD 1812\r", b"0"), (b"RD 1813\r", b"0")]
    script += selection(9, 22)
    script += [(b"ST 1905\r", b"OK"), (b"RD 1915\r", b"0"), (b"RD 1814\r", b"0")]
    script += [(b"RD 1915\r", b"1"), (b"RD 1812\r", b"0"), (b"RD 1813\r", b"1")]
    device, io = make_device(script)
    await device.setup()
    await device.export_plate(9, 22)
    self.assertEqual(io.expected, [])
    self.assertEqual(io.writes.count(b"ST 1905\r"), 1)

  async def test_export_at_the_current_position_writes_nothing(self):
    script = handshake()
    script += [(b"RD 1812\r", b"0"), (b"RD 1813\r", b"0")]
    script += selection_without_writes(9, 22)
    script += [(b"ST 1905\r", b"OK"), (b"RD 1915\r", b"1")]
    script += [(b"RD 1812\r", b"0"), (b"RD 1813\r", b"1")]
    device, io = make_device(script)
    await device.setup()
    await device.export_plate(9, 22)
    self.assertFalse([write for write in io.writes if write.startswith(b"WR")])

  async def test_export_refuses_a_busy_controller(self):
    from pylabrobot.tecan import CarouselError

    device, io = make_device([(b"CR\r", b"CC"), (b"RD 1915\r", b"0")])
    await device.setup()
    with self.assertRaises(CarouselError):
      await device.export_plate(9, 22)
    self.assertNotIn(b"ST 1905\r", io.writes)

  async def test_export_refuses_a_latched_error(self):
    from pylabrobot.tecan import CarouselError

    device, io = make_device(
      [(b"CR\r", b"CC"), (b"RD 1915\r", b"1"), (b"RD 1814\r", b"1"), (b"RD DM200\r", b"09491")]
    )
    await device.setup()
    with self.assertRaises(CarouselError) as raised:
      await device.export_plate(9, 22)
    self.assertEqual(raised.exception.code, 9491)
    self.assertNotIn(b"ST 1905\r", io.writes)

  async def test_export_refuses_an_occupied_transfer_station(self):
    device, io = make_device(
      [
        (b"CR\r", b"CC"),
        (b"RD 1915\r", b"1"),
        (b"RD 1814\r", b"0"),
        (b"RD 1812\r", b"0"),
        (b"RD 1813\r", b"1"),
      ]
    )
    await device.setup()
    with self.assertRaises(Exception):
      await device.export_plate(9, 22)
    self.assertNotIn(b"ST 1905\r", io.writes)

  async def test_export_refuses_a_plate_already_on_the_shovel(self):
    device, io = make_device(
      [(b"CR\r", b"CC"), (b"RD 1915\r", b"1"), (b"RD 1814\r", b"0"), (b"RD 1812\r", b"1")]
    )
    await device.setup()
    with self.assertRaises(Exception):
      await device.export_plate(9, 22)
    self.assertNotIn(b"ST 1905\r", io.writes)

  async def test_export_reports_a_controller_error_without_retrying(self):
    from pylabrobot.tecan import CarouselError

    script = handshake()
    script += [(b"RD 1812\r", b"0"), (b"RD 1813\r", b"0")]
    script += selection(9, 22)
    script += [(b"ST 1905\r", b"OK"), (b"RD 1915\r", b"0"), (b"RD 1814\r", b"1")]
    script += [(b"RD DM200\r", b"10253")]
    device, io = make_device(script)
    await device.setup()
    with self.assertRaises(CarouselError) as raised:
      await device.export_plate(9, 22)
    self.assertEqual(raised.exception.code, 10253)
    self.assertEqual(io.writes.count(b"ST 1905\r"), 1)

  async def test_export_deadline_does_not_retry(self):
    script = handshake()
    script += [(b"RD 1812\r", b"0"), (b"RD 1813\r", b"0")]
    script += selection(9, 22)
    script += [(b"ST 1905\r", b"OK")]
    device, io = make_device(script, auto=b"0")
    await device.setup()
    with self.assertRaises(TimeoutError):
      await device.export_plate(9, 22, timeout=0.5)
    self.assertEqual(io.writes.count(b"ST 1905\r"), 1)
    with self.assertRaises(RuntimeError):
      await device.is_ready()

  async def test_cancellation_after_the_acknowledgement_faults(self):
    script = handshake()
    script += [(b"RD 1812\r", b"0"), (b"RD 1813\r", b"0")]
    script += selection(9, 22)
    script += [(b"ST 1905\r", b"OK")]
    device, io = make_device(script, auto=b"0")
    await device.setup()
    task = asyncio.create_task(device.export_plate(9, 22))
    while len(io.writes) < 12:
      await asyncio.sleep(0)
    task.cancel()
    with self.assertRaises(asyncio.CancelledError):
      await task
    self.assertEqual(io.writes.count(b"ST 1905\r"), 1)
    with self.assertRaises(RuntimeError):
      await device.is_ready()

  async def test_export_reports_sensors_that_contradict_completion(self):
    from pylabrobot.tecan import CarouselError

    script = handshake()
    script += [(b"RD 1812\r", b"0"), (b"RD 1813\r", b"0")]
    script += selection(9, 22)
    script += [(b"ST 1905\r", b"OK"), (b"RD 1915\r", b"1")]
    script += [(b"RD 1812\r", b"1"), (b"RD 1813\r", b"0")]
    device, io = make_device(script)
    await device.setup()
    with self.assertRaises(CarouselError):
      await device.export_plate(9, 22)
    self.assertEqual(io.writes.count(b"ST 1905\r"), 1)

  async def test_import_plate_sequence(self):
    script = handshake()
    script += [(b"RD 1812\r", b"0"), (b"RD 1813\r", b"1")]
    script += selection(4, 3, current_cartridge=1)
    script += [(b"ST 1904\r", b"OK"), (b"RD 1915\r", b"1"), (b"RD 1813\r", b"0")]
    device, io = make_device(script)
    await device.setup()
    await device.import_plate(4, 3)
    self.assertEqual(io.expected, [])
    self.assertNotIn(b"ST 1905\r", io.writes)

  async def test_import_requires_a_plate_at_the_transfer_station(self):
    device, io = make_device(
      [
        (b"CR\r", b"CC"),
        (b"RD 1915\r", b"1"),
        (b"RD 1814\r", b"0"),
        (b"RD 1812\r", b"0"),
        (b"RD 1813\r", b"0"),
      ]
    )
    await device.setup()
    with self.assertRaises(Exception):
      await device.import_plate(4, 3)
    self.assertNotIn(b"ST 1904\r", io.writes)

  async def test_put_plate_on_transfer_station(self):
    script = handshake()
    script += [(b"RD 1812\r", b"1"), (b"RD 1813\r", b"0")]
    script += [(b"WR DM0 9\r", b"OK"), (b"WR DM5 22\r", b"OK"), (b"RD 1915\r", b"1")]
    script += [(b"ST 1906\r", b"OK"), (b"RD 1915\r", b"1")]
    script += [(b"RD 1812\r", b"0"), (b"RD 1813\r", b"1")]
    device, io = make_device(script)
    await device.setup()
    await device.put_plate_on_transfer_station(9, 22)
    self.assertEqual(io.expected, [])
    self.assertEqual(
      [write for write in io.writes if write.startswith((b"WR", b"ST"))],
      [b"WR DM0 9\r", b"WR DM5 22\r", b"ST 1906\r"],
    )

  async def test_put_plate_requires_a_plate_on_the_shovel(self):
    device, io = make_device(
      [(b"CR\r", b"CC"), (b"RD 1915\r", b"1"), (b"RD 1814\r", b"0"), (b"RD 1812\r", b"0")]
    )
    await device.setup()
    with self.assertRaises(Exception):
      await device.put_plate_on_transfer_station(9, 22)
    self.assertNotIn(b"ST 1906\r", io.writes)

  async def test_get_plate_from_transfer_station(self):
    script = handshake()
    script += [(b"RD 1812\r", b"0"), (b"RD 1813\r", b"1")]
    script += [(b"WR DM0 1\r", b"OK"), (b"WR DM5 1\r", b"OK"), (b"RD 1915\r", b"1")]
    script += [(b"ST 1907\r", b"OK"), (b"RD 1915\r", b"1"), (b"RD 1812\r", b"1")]
    device, io = make_device(script)
    await device.setup()
    await device.get_plate_from_transfer_station(1, 1)
    self.assertIn(b"ST 1907\r", io.writes)

  async def test_pick_and_place_validate_their_sensors(self):
    pick = handshake() + [(b"RD 1812\r", b"0")] + selection(2, 15, current_cartridge=1)
    pick += [(b"ST 1908\r", b"OK"), (b"RD 1915\r", b"1"), (b"RD 1812\r", b"1")]
    pick_device, pick_io = make_device(pick)
    await pick_device.setup()
    await pick_device.pick_plate(2, 15)
    self.assertIn(b"ST 1908\r", pick_io.writes)

    place = handshake() + [(b"RD 1812\r", b"1")] + selection(2, 15, current_cartridge=1)
    place += [(b"ST 1909\r", b"OK"), (b"RD 1915\r", b"1"), (b"RD 1812\r", b"0")]
    place_device, place_io = make_device(place)
    await place_device.setup()
    await place_device.place_plate(2, 15)
    self.assertIn(b"ST 1909\r", place_io.writes)

  async def test_pick_reports_an_empty_slot(self):
    script = handshake() + [(b"RD 1812\r", b"0")] + selection(2, 15, current_cartridge=1)
    script += [(b"ST 1908\r", b"OK"), (b"RD 1915\r", b"1"), (b"RD 1812\r", b"0")]
    device, _ = make_device(script)
    await device.setup()
    with self.assertRaises(Exception):
      await device.pick_plate(2, 15)

  async def test_move_plate_verifies_the_pick_before_placing(self):
    script = handshake()
    script += [(b"RD 1812\r", b"0"), (b"RD 1813\r", b"0")]
    script += selection(2, 17, current_cartridge=1)
    script += [(b"ST 1908\r", b"OK"), (b"RD 1915\r", b"1"), (b"RD 1812\r", b"1")]
    script += selection(2, 15, current_cartridge=2)
    script += [(b"ST 1909\r", b"OK"), (b"RD 1915\r", b"1"), (b"RD 1812\r", b"0")]
    device, io = make_device(script)
    await device.setup()
    await device.move_plate(2, 17, 2, 15)
    self.assertEqual(io.expected, [])
    self.assertNotIn(b"ST 1905\r", io.writes)

  async def test_move_plate_stops_when_nothing_was_picked(self):
    script = handshake()
    script += [(b"RD 1812\r", b"0"), (b"RD 1813\r", b"0")]
    script += selection(2, 17, current_cartridge=1)
    script += [(b"ST 1908\r", b"OK"), (b"RD 1915\r", b"1"), (b"RD 1812\r", b"0")]
    device, io = make_device(script)
    await device.setup()
    with self.assertRaises(Exception):
      await device.move_plate(2, 17, 2, 15)
    self.assertNotIn(b"ST 1909\r", io.writes)


class ValidationTests(unittest.IsolatedAsyncioTestCase):
  async def test_invalid_positions_never_reach_the_wire(self):
    cases = [
      (True, 1),
      (0, 1),
      (11, 1),
      (1.0, 1),
      ("1", 1),
      (1, True),
      (1, 0),
      (1, 23),
      (1, 1.0),
    ]
    for cartridge, level in cases:
      with self.subTest(cartridge=cartridge, level=level):
        device, io = make_device([(b"CR\r", b"CC")])
        await device.setup()
        with self.assertRaises(ValueError):
          await device.export_plate(cartridge, level)  # type: ignore[arg-type]
        self.assertEqual(io.writes, [b"CR\r"])

  async def test_geometry_is_configurable(self):
    device, io = make_device([(b"CR\r", b"CC")], number_of_cartridges=4, levels_per_cartridge=11)
    await device.setup()
    with self.assertRaises(ValueError):
      await device.pick_plate(5, 1)
    with self.assertRaises(ValueError):
      await device.pick_plate(1, 12)
    self.assertEqual(io.writes, [b"CR\r"])

  def test_constructor_rejects_bad_geometry(self):
    from pylabrobot.tecan import TecanCarousel

    for kwargs in (
      {"number_of_cartridges": 0},
      {"number_of_cartridges": True},
      {"levels_per_cartridge": -1},
      {"levels_per_cartridge": 2.0},
    ):
      with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
        TecanCarousel(port="p", **kwargs)  # type: ignore[arg-type]
    with self.assertRaises(ValueError):
      TecanCarousel(port="")


class LifecycleTests(unittest.IsolatedAsyncioTestCase):
  async def test_initialize_is_idempotent_when_active(self):
    device, io = make_device(handshake() + [(b"RD 1801\r", b"1")])
    await device.setup()
    await device.initialize()
    self.assertEqual(io.writes, [b"CR\r", b"RD 1915\r", b"RD 1814\r", b"RD 1801\r"])

  async def test_initialize_activates_an_inactive_handling(self):
    script = handshake() + [(b"RD 1801\r", b"0"), (b"ST 1801\r", b"OK"), (b"RD 1915\r", b"1")]
    script += [(b"RD 1801\r", b"1")]
    device, io = make_device(script)
    await device.setup()
    await device.initialize()
    self.assertEqual(io.expected, [])

  async def test_initialize_requires_a_ready_controller(self):
    device, io = make_device([(b"CR\r", b"CC"), (b"RD 1915\r", b"0")])
    await device.setup()
    with self.assertRaises(Exception):
      await device.initialize()
    self.assertNotIn(b"ST 1801\r", io.writes)

  async def test_reset_and_soft_reset_send_their_commands(self):
    reset, reset_io = make_device([(b"CR\r", b"CC"), (b"ST 1900\r", b"OK"), (b"RD 1915\r", b"1")])
    await reset.setup()
    await reset.reset()
    self.assertIn(b"ST 1900\r", reset_io.writes)

    soft, soft_io = make_device([(b"CR\r", b"CC"), (b"ST 1800\r", b"OK"), (b"RD 1915\r", b"1")])
    await soft.setup()
    await soft.soft_reset()
    self.assertIn(b"ST 1800\r", soft_io.writes)

  async def test_soft_reset_polls_out_of_a_stuck_state(self):
    script = [
      (b"CR\r", b"CC"),
      (b"ST 1800\r", b"OK"),
      (b"RD 1915\r", b"0"),
      (b"RD 1814\r", b"0"),
      (b"RD 1915\r", b"1"),
    ]
    device, io = make_device(script)
    await device.setup()
    await device.soft_reset()
    self.assertEqual(io.writes.count(b"ST 1800\r"), 1)


if __name__ == "__main__":
  unittest.main()
