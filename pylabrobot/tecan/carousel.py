"""Tecan Carousel LPT220 EVO Microplate Hotel driver.

The unit is sold and documented by Tecan and manufactured by Liconic, whose StoreX command set it
speaks. The carousel is a rotating plate hotel with a lift and a shovel that moves plates between
the cartridge slots and the transfer station, where another robot picks them up. It is controlled
over RS-232 with the ASCII command language in *StoreX Series Remote Operation V0711*: ``CR`` opens
the link, every command is answered by exactly one ``<payload>\\r\\n`` frame, and handling operations
are started with ``ST`` and complete when the ready bit (``RD 1915``) returns to ``1``.

Commands are sent from PyLabRobot rather than from a firmware macro, so the sequencing, the
preconditions and the decisions live here in readable Python.
"""

import asyncio
import logging
import time
from dataclasses import dataclass
from typing import Awaitable, Callable, Mapping, Optional, Sequence, TypeVar

from pylabrobot.io.serial import Serial

logger = logging.getLogger(__name__)

_T = TypeVar("_T")

# Controller-level rejections (Remote Operation V0711, section 1.2.7). The frame was understood and
# refused, so the serial link is still in sync and the session is not faulted.
_CONTROLLER_ERRORS = {
  b"E0": "relay error: unknown timer, counter or data memory, or invalid unit",
  b"E1": "command error: invalid command or interrupted transmission",
  b"E2": "program error: controller firmware lost",
  b"E3": "hardware error: controller hardware fault",
  b"E4": "write protected: unauthorized access",
  b"E5": "base unit error: unauthorized access",
}

# Barcode replies do not use the ordinary one-frame-per-command shape. ``ST 1710`` answers ``OK``
# followed by STX/ETX-framed binary segments: a ``SPS:`` position field, then the barcode payload.
_SCAN_STX = b"\x02"
_SCAN_ETX = b"\x03"

# ``DM200`` reads 16384 while the handler is in barcode positioning mode (Liconic FAQ0097), and 0
# outside it on this unit; the driver uses it only to check that ``RS 1910`` actually left the mode.
_BCR_PROCESS_WORD = 16384

# Ordered aim offsets for a barcode read, in register units relative to the requested level's
# nominal read position, positive above it. Measured on this unit: the beam sits 1-2 mm below the
# label at ``DM27 = 0`` and one plate was only read 50-100 units above nominal, so a nominal-only
# scan is not reliable. ``DM23`` (788 for one controller level) bounds the useful band.
_DEFAULT_BCR_SWEEP = (0, 50, 100, 150, 200)


def _strip_latched_frames(data: bytes) -> bytes:
  """Drop complete STX/ETX segments from the front of a reply.

  In barcode mode the reader echoes its previous result inside the reply to the next ``WR``/``ST``
  command, so an ordinary line reply can arrive behind one or two framed binary segments. The
  protocol is one reply per command, so anything framed ahead of the line is stale and is discarded.
  """
  while data.startswith(_SCAN_STX):
    end = data.find(_SCAN_ETX)
    if end < 0:
      return data
    data = data[end + 1 :]
  return data


def _parse_scan_frame(data: bytes) -> tuple[Optional[str], Optional[str]]:
  """Split an ``ST 1710`` reply into its position field and barcode payload.

  Returns ``(position, payload)``. ``position`` is the raw text after ``SPS:``, or ``None`` when no
  scanned frame is present. ``payload`` is the raw string after the position segment, including the
  trailing Code128 check character, or ``None`` when no payload segment followed it. A payload of
  ``"??"`` is the reader reporting that there is nothing to read at that position, not an error.

  Payload segments that arrive before the ``SPS:`` field are latched results echoed from an earlier
  read and are discarded: only a payload carried by a fresh scan counts.
  """
  body = data[len(b"OK\r\n") :] if data.startswith(b"OK\r\n") else data
  position: Optional[str] = None
  payload: Optional[str] = None
  seen_position = False
  for segment in body.split(_SCAN_ETX):
    text = segment.replace(_SCAN_STX, b"").strip(b"\r\n").decode("ascii", errors="replace")
    if not text or text == "OK":
      continue
    if text.startswith("SPS:"):
      position = text[len("SPS:") :]
      seen_position = True
      continue
    if seen_position:
      payload = text
  return position, payload


class CarouselError(RuntimeError):
  """A carousel operation did not complete as intended.

  Attributes:
    code: raw ``DM200`` process word when the controller reported one, otherwise ``None``.
    stage: the operation that raised, for logs where the message alone is ambiguous.
  """

  def __init__(self, message: str, *, code: Optional[int] = None, stage: Optional[str] = None):
    super().__init__(message)
    self.code = code
    self.stage = stage


class CarouselRejected(CarouselError):
  """The controller refused a command with ``E0``-``E5``; the link itself is still usable."""


@dataclass(frozen=True)
class CarouselStatus:
  """One read-only snapshot of the controller's state registers.

  ``process_status`` is the raw ``DM200`` word. It is not decoded: the documented StoreX error table
  does not describe the values this unit reports, so no meaning is invented for them.
  """

  ready: bool
  error: bool
  active: bool
  plate_ready: bool
  plate_on_shovel: bool
  plate_at_transfer_station: bool
  cartridge: int
  level: int
  process_status: int


@dataclass(frozen=True)
class BarcodeRead:
  """One barcode scan of a carousel position.

  Attributes:
    cartridge: cartridge address that was scanned.
    level: controller level that was scanned.
    position: the reader's raw ``SPS:`` field, which encodes the position it actually scanned.
    payload: raw barcode text including the trailing Code128 check character, or ``"??"`` when the
      reader saw nothing.
    barcode: the payload without the check character, or ``None`` when the position was empty.
    aim_delta: the aim offset, in register units, that produced this result: negative below the
      level's nominal read position, positive above it, 0 at the nominal position.
  """

  cartridge: int
  level: int
  position: str
  payload: str
  barcode: Optional[str]
  aim_delta: int = 0


class TecanCarousel:
  """A Tecan Carousel plate hotel, manufactured by Liconic.

  Public methods are the carousel's operations: reads of controller state, and plate handling
  between cartridge levels and the transfer station. Every operation is bounded by a deadline,
  checks the ready/error flags before it writes, waits for the ready bit to return after a handling
  command, and never retries: an interrupted or uncertain operation faults the session until
  ``stop()`` and ``setup()`` are called again.

  ``setup()`` only opens the serial link. Opening a connection is not initialization: call
  ``initialize()`` before handling plates. Closing the session is not a physical stop.

  Cartridge and level numbers are controller addresses. Level numbering starts at the bottom
  (level 1 is the lowest plate position); it is not the top-down slot numbering used in lab notes,
  and the mapping between them has not been verified for this unit.

  Barcode reading (``read_barcode``) is the one operation that changes the positioning mode of the
  whole handler: ``ST 1910`` puts the lift into barcode-read positioning and ``RS 1910`` is sent to
  leave it. The entry/exit sequence and the read itself were measured on one LPT220 EVO unit, and
  the exit is confirmed through ``DM200`` before the session is used for handling again.

  Not implemented, because their effect on this installation is not established: door and gate
  control (``ST 1901``, ``ST 1902``), terminate access (``ST 1903``), short import/export, and the
  climate and shaker commands of incubator variants.
  """

  _EXCHANGE_TIMEOUT = 1.0
  _EXCHANGE_LIMIT = 256
  _SETTLE_DELAY = 0.3
  _POLL_INTERVAL = 0.2
  _SCAN_SETTLE = 0.15
  _SCAN_QUIET = 0.8
  _SCAN_TIMEOUT = 10.0
  _SCAN_LIMIT = 4096

  def __init__(
    self,
    port: str,
    *,
    number_of_cartridges: int = 10,
    levels_per_cartridge: int = 22,
    cassette_slots: Optional[Mapping[int, int]] = None,
    cassette_level_step: Optional[Mapping[int, int]] = None,
    cassette_levels_increase_downward: Optional[Mapping[int, bool]] = None,
    cassette_top_level: Optional[Mapping[int, int]] = None,
  ) -> None:
    """Configure an explicit operator-selected serial port without opening it.

    Args:
      port: serial device, chosen by the operator rather than discovered automatically.
      number_of_cartridges: cartridges on the rotating plate. The default is the manual's maximum
        of ten; the installed count is an installation detail.
      levels_per_cartridge: plate positions per cartridge for the installed cassettes. The default
        of 22 is a standard cassette; half-height and deep-well cassettes hold fewer.
      cassette_top_level: controller level of each cassette's topmost slot, for cassettes that are
        not mounted so that their top slot sits at the level the geometry would otherwise assume.
        On the unit this driver was developed against, the 7-slot deep-well cassette in cartridge 1
        has its top slot at level 22, so slot 1 is level 22 and slot 7 is level 4, not 19 and 1.
    """
    if not port:
      raise ValueError("An explicit port is required")
    if type(number_of_cartridges) is not int or number_of_cartridges <= 0:
      raise ValueError("number_of_cartridges must be a positive integer")
    if type(levels_per_cartridge) is not int or levels_per_cartridge <= 0:
      raise ValueError("levels_per_cartridge must be a positive integer")
    self.number_of_cartridges = number_of_cartridges
    self.levels_per_cartridge = levels_per_cartridge
    self.cassette_slots: dict[int, int] = {
      cartridge: (cassette_slots or {}).get(cartridge, levels_per_cartridge)
      for cartridge in range(1, number_of_cartridges + 1)
    }
    self.cassette_level_step: dict[int, int] = {
      cartridge: (cassette_level_step or {}).get(cartridge, 1)
      for cartridge in range(1, number_of_cartridges + 1)
    }
    # True means the controller's level 1 is the topmost slot (standard cassette);
    # False means level 1 is the bottommost slot (deep-well cassette on this unit).
    # Positions in the public API always count from the top either way.
    self.cassette_levels_increase_downward: dict[int, bool] = {
      cartridge: (cassette_levels_increase_downward or {}).get(cartridge, True)
      for cartridge in range(1, number_of_cartridges + 1)
    }
    # Controller level of the cassette's topmost slot, when it is not where the geometry alone
    # would put it. None keeps the derived default (level 1 for a downward-counting cassette, the
    # highest level for an upward-counting one, assuming the cassette starts at level 1).
    self.cassette_top_level: dict[int, Optional[int]] = {
      cartridge: (cassette_top_level or {}).get(cartridge)
      for cartridge in range(1, number_of_cartridges + 1)
    }
    self.io = Serial(
      human_readable_device_name="Tecan carousel",
      port=port,
      baudrate=9600,
      bytesize=8,
      parity="E",
      stopbits=1,
      timeout=1,
      write_timeout=1,
      rtscts=True,
    )
    self._lock = asyncio.Lock()
    self._opened = False
    self._ready = False
    self._faulted = False
    # True while the handler has been put into barcode positioning mode (ST 1910) and not confirmed
    # out of it, so stop() can try to leave the mode before closing the link.
    self._bcr_mode = False
    self._bcr_exit_problem: Optional[str] = None

  # Transport

  async def _exchange(self, command: bytes) -> bytes:
    """Send one CR-terminated frame and read one CRLF-terminated reply; caller holds the lock."""
    logger.debug("Tecan request %r", command)
    await self.io.write(command)
    reply = bytearray()
    while len(reply) < self._EXCHANGE_LIMIT:
      chunk = await self.io.read(1)
      if not chunk:
        raise TimeoutError(f"Incomplete Tecan reply to {command!r}: {bytes(reply)!r}")
      reply.extend(chunk)
      if _strip_latched_frames(bytes(reply)).endswith(b"\n"):
        break
    raw = _strip_latched_frames(bytes(reply))
    logger.debug("Tecan reply %r", raw)
    if not raw.endswith(b"\r\n") or b"\r" in raw[:-2] or b"\n" in raw[:-2]:
      raise ValueError(f"Malformed Tecan frame: {raw!r}")
    return raw[:-2]

  async def _payload(self, command: bytes) -> bytes:
    """Exchange one frame and map controller rejections; caller holds the lock."""
    payload = await asyncio.wait_for(self._exchange(command), timeout=self._EXCHANGE_TIMEOUT)
    if len(payload) == 2 and payload[:1] == b"E":
      description = _CONTROLLER_ERRORS.get(payload, "unknown controller error")
      raise CarouselRejected(f"{command!r} rejected: {payload.decode('ascii')} ({description})")
    return payload

  async def _read_bool(self, command: bytes) -> bool:
    """Read a one-character flag; caller holds the lock."""
    payload = await self._payload(command)
    if payload not in (b"0", b"1"):
      raise ValueError(f"Expected boolean reply to {command!r}, received {payload!r}")
    return payload == b"1"

  async def _read_int(self, command: bytes) -> int:
    """Read a five digit data memory; caller holds the lock."""
    payload = await self._payload(command)
    if len(payload) != 5 or not all(48 <= byte <= 57 for byte in payload):
      raise ValueError(f"Expected five ASCII digits from {command!r}, received {payload!r}")
    return int(payload)

  async def _write(self, command: bytes) -> None:
    """Send a command that must answer ``OK``; caller holds the lock."""
    payload = await self._payload(command)
    if payload != b"OK":
      raise ValueError(f"Expected OK from {command!r}, received {payload!r}")

  async def _set_register(self, register: str, value: int) -> None:
    """Write one data memory and require its acknowledgement; caller holds the lock."""
    await self._write(f"WR {register} {value}\r".encode("ascii"))

  async def _read_scan_frame(
    self, command: bytes, *, stage: str, expect_scan: bool = False
  ) -> bytes:
    """Send a command whose reply may carry binary frames; caller holds the lock.

    Barcode-mode replies do not have the one-line shape the rest of the protocol uses: ``OK`` is
    followed by STX/ETX-framed segments, and the controller also re-sends its previous barcode
    inside unrelated replies. Reading to the first newline would leave those bytes in the input
    buffer for the next exchange to misparse, so the reply is collected until it has been quiet for
    ``_SCAN_QUIET`` seconds instead.

    With ``expect_scan`` the reply is only considered complete once it carries both the ``SPS:``
    position and a payload segment. On 2026-11-05 the reader delivered those in separate bursts for
    some positions, and a quiet-window-only read truncated the frame after the position field.
    """
    await self.io.reset_input_buffer()
    await self.io.write(command)
    await asyncio.sleep(self._SCAN_SETTLE)
    buffer = bytearray()
    last = time.monotonic()
    deadline = last + self._SCAN_TIMEOUT
    while time.monotonic() < deadline:
      chunk = await self.io.read(64)
      if chunk:
        buffer.extend(chunk)
        last = time.monotonic()
        if len(buffer) > self._SCAN_LIMIT:
          raise CarouselError(
            f"{stage}: reply to {command!r} exceeded {self._SCAN_LIMIT} bytes", stage=stage
          )
        continue
      quiet = time.monotonic() - last >= self._SCAN_QUIET
      if expect_scan:
        position, payload = _parse_scan_frame(bytes(buffer))
        if position is not None and payload is not None and quiet:
          break
      elif buffer and quiet:
        break
      await asyncio.sleep(0.02)
    data = bytes(buffer)
    logger.debug("Tecan scan reply to %r: %s", command, data.hex())
    return data

  async def _scan_command(self, command: bytes, *, stage: str, expect_scan: bool = False) -> bytes:
    """Send a barcode-mode command and require its ``OK``; caller holds the lock."""
    data = await self._read_scan_frame(command, stage=stage, expect_scan=expect_scan)
    head = data.split(b"\r\n", 1)[0]
    if len(head) == 2 and head[:1] == b"E":
      description = _CONTROLLER_ERRORS.get(head, "unknown controller error")
      raise CarouselRejected(f"{command!r} rejected: {head.decode('ascii')} ({description})")
    if head != b"OK":
      raise CarouselError(
        f"{stage}: expected OK from {command!r}, received {data[:60]!r}", stage=stage
      )
    return data

  async def _leave_bcr_mode(
    self, *, stage: str, barcode_mode_at_entry: Optional[bool] = None
  ) -> bool:
    """Best-effort return from barcode positioning mode; never raises. Caller holds the lock.

    Sends ``ST 1713`` (reader off), ``RS 1200`` (two-way communication off) and ``RS 1910`` (mode
    exit), then reads ``DM200``, which reports 16384 while the handler is in barcode mode.

    ``DM200`` is not a clean indicator: on 2026-11-05 the unit reported 16384 at rest, before any
    barcode command in the session had been sent, so a 16384 reading cannot prove that this call is
    what left the handler in the mode. The check therefore only fails when the value was *not*
    already 16384 when the read started; an unchanged 16384 is reported as a warning.
    """
    self._bcr_exit_problem = None
    if not self._bcr_mode:
      return True
    try:
      await self._scan_command(b"ST 1713\r", stage=f"{stage}: reader off")
      await self._scan_command(b"RS 1200\r", stage=f"{stage}: two-way off")
      await self._scan_command(b"RS 1910\r", stage=f"{stage}: mode exit")
      code = await self._read_int(b"RD DM200\r")
    except Exception as error:
      self._bcr_exit_problem = repr(error)
      logger.warning(
        "Tecan barcode-mode cleanup failed; the handler mode is unknown", exc_info=True
      )
      return False
    if code == _BCR_PROCESS_WORD:
      if barcode_mode_at_entry:
        logger.warning(
          "DM200 still reports barcode mode (16384) after RS 1910, but it did so before the read "
          "started as well; the exit was sent, the mode state is unchanged from entry"
        )
      else:
        self._bcr_exit_problem = "DM200 still reports barcode mode (16384) after RS 1910"
        logger.warning("%s", self._bcr_exit_problem)
        return False
    self._bcr_mode = False
    return True

  async def _operation(
    self,
    operation: Callable[[], Awaitable[_T]],
    *,
    stage: str,
    timeout: float,
  ) -> _T:
    """Run one operation under the session lock, bounding it and latching uncertain failures."""
    async with self._lock:
      if not self._ready or self._faulted:
        raise RuntimeError(
          "Not connected: call setup(); a faulted session requires stop() then setup()"
        )
      try:
        return await asyncio.wait_for(operation(), timeout=timeout)
      except CarouselRejected:
        logger.warning("Tecan %s rejected by the controller; session kept in sync", stage)
        raise
      except BaseException:
        self._faulted = True
        self._ready = False
        logger.exception("Tecan %s failed or was interrupted; session faulted, no retry", stage)
        raise

  # Waiting and preconditions

  async def _check_ready(self, stage: str) -> None:
    """Require a ready controller with no latched error; caller holds the lock."""
    if not await self._read_bool(b"RD 1915\r"):
      raise CarouselError(f"{stage}: controller is busy; no command was sent", stage=stage)
    if await self._read_bool(b"RD 1814\r"):
      code = await self._read_int(b"RD DM200\r")
      raise CarouselError(f"{stage}: controller has an error, DM200={code}", code=code, stage=stage)

  async def _wait_ready(self, stage: str) -> None:
    """Poll until handling is ready again, failing on an error flag; caller holds the lock."""
    await asyncio.sleep(self._SETTLE_DELAY)
    while True:
      if await self._read_bool(b"RD 1915\r"):
        return
      if await self._read_bool(b"RD 1814\r"):
        code = await self._read_int(b"RD DM200\r")
        raise CarouselError(
          f"{stage}: controller reported an error, DM200={code}; no retry", code=code, stage=stage
        )
      await asyncio.sleep(self._POLL_INTERVAL)

  def _validate_position(self, cartridge: int, level: int, stage: str) -> None:
    """Reject addresses outside the configured geometry before anything reaches the wire."""
    if type(cartridge) is not int or not 1 <= cartridge <= self.number_of_cartridges:
      raise ValueError(
        f"{stage}: cartridge must be an integer in 1..{self.number_of_cartridges}, got {cartridge!r}"
      )
    slots = self.cassette_slots[cartridge]
    step = self.cassette_level_step[cartridge]
    top = self._top_level(cartridge)
    topmost = top + (slots - 1) * step if self.cassette_levels_increase_downward[cartridge] else top
    max_level = min(self.levels_per_cartridge, topmost)
    if type(level) is not int or not 1 <= level <= max_level:
      raise ValueError(f"{stage}: level must be an integer in 1..{max_level}, got {level!r}")

  def _top_level(self, cartridge: int) -> int:
    """The controller level of a cassette's topmost slot, from the config or the geometry."""
    configured = self.cassette_top_level[cartridge]
    if configured is not None:
      return configured
    if self.cassette_levels_increase_downward[cartridge]:
      return 1
    return 1 + (self.cassette_slots[cartridge] - 1) * self.cassette_level_step[cartridge]

  def level_for_slot(self, cartridge: int, slot: int) -> int:
    """Translate a physical slot number to the controller level that addresses it.

    Slots are counted from the **top** of the cassette: slot 1 is the topmost plate. The controller
    counts *levels* along the lift stroke, and how its level numbering runs depends on the cassette
    type. A standard 22-slot cassette counts downward from the top, so slot 1 sits at controller
    level 1. A 7-slot deep-well cassette on this unit counts upward from the bottom, so its top slot
    sits at the highest level and its bottom slot at level 1; three levels are used per slot. Set
    ``cassette_levels_increase_downward`` to False for such a cartridge.

    Args:
      cartridge: controller cartridge address, 1-based.
      slot: physical slot counted from the top of the cassette, 1-based.

    Returns:
      The controller level to pass to ``select_position`` and the handling methods.
    """
    if type(cartridge) is not int or not 1 <= cartridge <= self.number_of_cartridges:
      raise ValueError(
        f"level_for_slot: cartridge must be an integer in 1..{self.number_of_cartridges}, "
        f"got {cartridge!r}"
      )
    slots = self.cassette_slots[cartridge]
    if type(slot) is not int or not 1 <= slot <= slots:
      raise ValueError(f"level_for_slot: slot must be an integer in 1..{slots}, got {slot!r}")
    step = self.cassette_level_step[cartridge]
    top = self._top_level(cartridge)
    if self.cassette_levels_increase_downward[cartridge]:
      # Level numbering runs with the slot order: the top slot is the lowest level of the cassette.
      level = top + (slot - 1) * step
    else:
      # Level numbering runs against the slot order: the top slot is the highest level.
      level = top - (slot - 1) * step
    if not 1 <= level <= self.levels_per_cartridge:
      raise ValueError(
        f"level_for_slot: slot {slot} maps to level {level}, outside the configured "
        f"1..{self.levels_per_cartridge} levels"
      )
    return level

  def slot_for_level(self, cartridge: int, level: int) -> int:
    """Translate a controller level to the physical slot it addresses.

    The inverse of ``level_for_slot``, with slots counted from the top of the cassette. A level
    between two physical slots does not address a slot and raises: the shelf pitch leaves gaps in
    the level axis.

    Args:
      cartridge: controller cartridge address, 1-based.
      level: controller level, 1-based.

    Returns:
      The physical slot counted from the top of the cassette, 1-based.
    """
    self._validate_position(cartridge, level, "slot_for_level")
    step = self.cassette_level_step[cartridge]
    slots = self.cassette_slots[cartridge]
    top = self._top_level(cartridge)
    if self.cassette_levels_increase_downward[cartridge]:
      offset = level - top
    else:
      offset = top - level
    if offset % step:
      raise ValueError(
        f"slot_for_level: level {level} does not address a slot on cartridge {cartridge} "
        f"(levels step by {step})"
      )
    slot = offset // step + 1
    if slot > slots:
      raise ValueError(
        f"slot_for_level: level {level} maps to slot {slot}, beyond the {slots} installed slots"
      )
    return slot

  async def _select_cartridge(self, cartridge: int) -> None:
    """Rotate to a cartridge address alone; caller holds the lock and checked the preconditions."""
    selected = await self._read_int(b"RD DM0\r")
    current = await self._read_int(b"RD DM1\r")
    # Skip the write only when the mechanism agrees with the request. A stale DM0 from a previous
    # session can name the target while DM1 still reports another position, and then only a fresh
    # WR DM0 asks the controller to rotate.
    if selected != cartridge or current != cartridge:
      await self._set_register("DM0", cartridge)
      await self._wait_ready("select_cartridge")
      selected = await self._read_int(b"RD DM0\r")
      if selected != cartridge:
        raise CarouselError(
          f"select_cartridge: selected cartridge reads back as {selected}, expected {cartridge}",
          stage="select_cartridge",
        )
    await self._wait_for_current_cartridge(cartridge)

  async def _select_position(self, cartridge: int, level: int) -> None:
    """Rotate and lift to a cartridge level; caller holds the lock and checked the preconditions."""
    if await self._read_int(b"RD DM0\r") != cartridge:
      await self._set_register("DM0", cartridge)
    if await self._read_int(b"RD DM5\r") != level:
      await self._set_register("DM5", level)
    await self._wait_ready("select_position")
    selected = await self._read_int(b"RD DM0\r")
    if selected != cartridge:
      raise CarouselError(
        f"select_position: selected cartridge reads back as {selected}, expected {cartridge}",
        stage="select_position",
      )
    await self._wait_for_current_cartridge(cartridge)

  async def _wait_for_current_cartridge(self, cartridge: int) -> None:
    """Poll the reported current cartridge until the rotation has landed; read-only."""
    while await self._read_int(b"RD DM1\r") != cartridge:
      if await self._read_bool(b"RD 1814\r"):
        code = await self._read_int(b"RD DM200\r")
        raise CarouselError(
          f"select_position: rotation failed, DM200={code}", code=code, stage="select_position"
        )
      await asyncio.sleep(self._POLL_INTERVAL)

  async def _require_sensors(
    self, *, shovel: Optional[bool], station: Optional[bool], stage: str
  ) -> None:
    """Enforce an expected plate position; ``None`` leaves that sensor unchecked."""
    if shovel is not None:
      observed = await self._read_bool(b"RD 1812\r")
      if observed is not shovel:
        state = "a plate" if observed else "no plate"
        raise CarouselError(
          f"{stage}: expected shovel {'occupied' if shovel else 'empty'}, saw {state}", stage=stage
        )
    if station is not None:
      observed = await self._read_bool(b"RD 1813\r")
      if observed is not station:
        state = "a plate" if observed else "no plate"
        raise CarouselError(
          f"{stage}: expected transfer station {'occupied' if station else 'empty'}, saw {state}",
          stage=stage,
        )

  async def _wait_sensor(
    self, command: bytes, expected: bool, *, stage: str, timeout: float = 5.0
  ) -> None:
    """Poll one plate sensor until it reports ``expected``, tolerating release/engage settling."""
    deadline = time.monotonic() + timeout
    observed = not expected
    while time.monotonic() < deadline:
      observed = await self._read_bool(command)
      if observed is expected:
        return
      await asyncio.sleep(self._POLL_INTERVAL)
    state = "a plate" if observed else "no plate"
    raise CarouselError(
      f"{stage}: sensor {command!r} still reports {state} after {timeout:.1f}s",
      stage=stage,
    )

  async def _confirm_delivery(self, stage: str) -> None:
    """Fail loudly when the plate sensors contradict a completed handling action."""
    shovel = await self._read_bool(b"RD 1812\r")
    station = await self._read_bool(b"RD 1813\r")
    if not station or shovel:
      raise CarouselError(
        f"{stage}: handling reports ready but the sensors disagree "
        f"(shovel={'occupied' if shovel else 'empty'}, "
        f"transfer station={'occupied' if station else 'empty'})",
        stage=stage,
      )

  # Lifecycle

  async def setup(self) -> None:
    """Open the serial port and complete the ``CR``/``CC`` handshake; does not initialize.

    A faulted session requires ``stop()`` before another setup. Repeated successful setup is a
    no-op.
    """
    async with self._lock:
      if self._faulted:
        raise RuntimeError("Faulted session: call stop() then setup()")
      if self._ready:
        return
      logger.warning(
        "Tecan carousel driver is not verified against all hardware actions; "
        "confirm the plate positions and the clear motion path before moving anything"
      )
      try:
        self._opened = True
        await self.io.setup()
        await self.io.reset_input_buffer()
        if await asyncio.wait_for(self._exchange(b"CR\r"), timeout=self._EXCHANGE_TIMEOUT) != b"CC":
          raise ValueError("Expected Tecan CC handshake")
        self._ready = True
        logger.info("Tecan carousel connection established; call initialize() before handling")
      except BaseException:
        self._faulted = True
        self._ready = False
        logger.exception("Tecan setup failed")
        await self.io.stop()
        self._opened = False
        raise

  async def stop(self) -> None:
    """Quit the controller conversation and close the serial port; not a physical stop."""
    async with self._lock:
      self._ready = False
      if self._opened:
        if self._bcr_mode:
          # Leaving barcode positioning mode matters even when the session faulted: the handler
          # otherwise stays in a mode where handling commands are not assumed to work.
          await self._leave_bcr_mode(stage="stop")
        if not self._faulted:
          try:
            reply = await asyncio.wait_for(self._exchange(b"CQ\r"), timeout=self._EXCHANGE_TIMEOUT)
            if reply != b"CF":
              logger.warning("Tecan did not confirm communication quit: %r", reply)
          except Exception:
            logger.warning("Tecan communication quit failed; closing serial anyway", exc_info=True)
        await self.io.stop()
        self._opened = False
      self._faulted = False
      logger.info("Tecan serial connection closed (not a physical stop)")

  async def initialize(self, *, timeout: float = 60) -> None:
    """Activate and home the handling (``ST 1801``), unless it is already active.

    Required after a cold start. Calling it on an already active handling does nothing, so the
    method is safe to repeat. A faulted or busy controller is an error rather than an excuse to
    reset.
    """

    async def _initialize() -> None:
      await self._check_ready("initialize")
      if await self._read_bool(b"RD 1801\r"):
        logger.info("Tecan handling is already active; no initialization sent")
        return
      logger.info("Initializing Tecan handling (ST 1801); the carousel and lift will home")
      await self._write(b"ST 1801\r")
      await self._wait_ready("initialize")
      if not await self._read_bool(b"RD 1801\r"):
        raise CarouselError(
          "initialize: handling did not report active afterwards", stage="initialize"
        )

    await self._operation(_initialize, stage="initialize", timeout=timeout)

  async def reset(self, *, timeout: float = 60) -> None:
    """Reset the handling (``ST 1900``), clearing the error flag and ``DM200``.

    The reset re-homes the mechanism and leaves the handling inactive: call ``initialize()``
    afterwards. It is the documented way to clear a latched handling error, not a routine retry.
    """

    async def _reset() -> None:
      logger.warning("Resetting Tecan handling (ST 1900); motion will occur")
      await self._write(b"ST 1900\r")
      await self._wait_ready("reset")

    await self._operation(_reset, stage="reset", timeout=timeout)

  async def soft_reset(self, *, timeout: float = 60) -> None:
    """Recover the current access with ``ST 1800``, keeping the plate where it is.

    This is the documented recovery for an interrupted handling action, in contrast to
    ``reset()``: it does not re-home or clear the plate. It is not a retry of the failed
    operation, and the plate's physical position still needs operator confirmation.
    """

    async def _soft_reset() -> None:
      logger.warning("Soft resetting Tecan handling (ST 1800); motion may occur")
      await self._write(b"ST 1800\r")
      await self._wait_ready("soft_reset")

    await self._operation(_soft_reset, stage="soft_reset", timeout=timeout)

  # Status

  async def is_ready(self) -> bool:
    """Whether handling is ready to accept a command (``RD 1915``)."""
    return await self._operation(lambda: self._read_bool(b"RD 1915\r"), stage="is_ready", timeout=2)

  async def has_error(self) -> bool:
    """Whether the error flag is set (``RD 1814``); reading it does not clear it."""
    return await self._operation(
      lambda: self._read_bool(b"RD 1814\r"), stage="has_error", timeout=2
    )

  async def is_active(self) -> bool:
    """Whether the handling has been activated (``RD 1801``)."""
    return await self._operation(
      lambda: self._read_bool(b"RD 1801\r"), stage="is_active", timeout=2
    )

  async def is_plate_ready(self) -> bool:
    """The plate-ready flag (``RD 1815``), set while a transfer has just completed."""
    return await self._operation(
      lambda: self._read_bool(b"RD 1815\r"), stage="is_plate_ready", timeout=2
    )

  async def is_plate_on_shovel(self) -> bool:
    """Whether the shovel carries a plate (``RD 1812``); not evidence of a delivery."""
    return await self._operation(
      lambda: self._read_bool(b"RD 1812\r"), stage="is_plate_on_shovel", timeout=2
    )

  async def is_plate_at_transfer_station(self) -> bool:
    """Whether a plate sits on the transfer station (``RD 1813``)."""
    return await self._operation(
      lambda: self._read_bool(b"RD 1813\r"), stage="is_plate_at_transfer_station", timeout=2
    )

  async def get_selected_cartridge(self) -> int:
    """The cartridge address written to ``DM0``, not a measured arm position."""
    return await self._operation(
      lambda: self._read_int(b"RD DM0\r"), stage="get_selected_cartridge", timeout=2
    )

  async def get_selected_level(self) -> int:
    """The level written to ``DM5``, not a measured lift position."""
    return await self._operation(
      lambda: self._read_int(b"RD DM5\r"), stage="get_selected_level", timeout=2
    )

  async def get_current_cartridge(self) -> int:
    """The cartridge the controller reports being at (``RD DM1``)."""
    return await self._operation(
      lambda: self._read_int(b"RD DM1\r"), stage="get_current_cartridge", timeout=2
    )

  async def get_process_status(self) -> int:
    """The raw ``DM200`` process word, undecoded by design."""
    return await self._operation(
      lambda: self._read_int(b"RD DM200\r"), stage="get_process_status", timeout=2
    )

  async def get_status(self, *, timeout: float = 10) -> CarouselStatus:
    """Read the whole state snapshot in one serialized exchange."""

    async def _status() -> CarouselStatus:
      return CarouselStatus(
        ready=await self._read_bool(b"RD 1915\r"),
        error=await self._read_bool(b"RD 1814\r"),
        active=await self._read_bool(b"RD 1801\r"),
        plate_ready=await self._read_bool(b"RD 1815\r"),
        plate_on_shovel=await self._read_bool(b"RD 1812\r"),
        plate_at_transfer_station=await self._read_bool(b"RD 1813\r"),
        cartridge=await self._read_int(b"RD DM0\r"),
        level=await self._read_int(b"RD DM5\r"),
        process_status=await self._read_int(b"RD DM200\r"),
      )

    return await self._operation(_status, stage="get_status", timeout=timeout)

  # Positioning

  async def select_cartridge(self, cartridge: int, *, timeout: float = 60) -> None:
    """Rotate to a cartridge without touching the lift (``WR DM0``).

    The manual's Position Carrousel operation sets only the cartridge address, so this leaves
    ``DM5`` exactly as it is. It does not move the lift and does not approach the transfer station,
    so a plate waiting there does not block it. The shovel must be empty and the path clear.

    Args:
      cartridge: controller cartridge address, 1-based.
      timeout: whole-operation deadline in seconds.

    Raises:
      ValueError: Invalid address.
      CarouselError: The controller is busy, has an error, or a plate is on the shovel.
      TimeoutError: The rotation did not report completion before the deadline.
    """
    if type(cartridge) is not int or not 1 <= cartridge <= self.number_of_cartridges:
      raise ValueError(
        f"select_cartridge: cartridge must be an integer in 1..{self.number_of_cartridges}, "
        f"got {cartridge!r}"
      )

    async def _select() -> None:
      await self._check_ready("select_cartridge")
      await self._require_sensors(shovel=False, station=None, stage="select_cartridge")
      await self._select_cartridge(cartridge)

    await self._operation(_select, stage="select_cartridge", timeout=timeout)

  async def select_position(self, cartridge: int, level: int, *, timeout: float = 60) -> None:
    """Rotate and lift to a cartridge level, verifying the reported position.

    This is the carousel's own positioning, used to reach a level without moving a plate. Motion
    occurs: confirm that the shovel and the transfer station are empty and that the path is clear
    before calling it.

    Args:
      cartridge: controller cartridge address, 1-based.
      level: controller level, 1-based from the bottom of the cartridge.
      timeout: whole-operation deadline in seconds.
    """
    self._validate_position(cartridge, level, "select_position")

    async def _select() -> None:
      await self._check_ready("select_position")
      await self._require_sensors(shovel=False, station=False, stage="select_position")
      await self._select_position(cartridge, level)

    await self._operation(_select, stage="select_position", timeout=timeout)

  # Barcode reading

  async def _apply_dm27(self, value: int) -> None:
    """Write the reader-height register and verify the readback; caller holds the lock.

    The controller zero-pads register replies (``00550``), so the readback is compared as an
    integer; a string compare was the bug behind one silently skipped write on 2026-11-03.
    """
    await self._set_register("DM27", value)
    observed = await self._read_int(b"RD DM27\r")
    if observed != value:
      raise CarouselError(
        f"read_barcode: DM27 reads back as {observed} after writing {value}",
        stage="read_barcode",
      )

  async def _restore_dm27(self, original: int) -> bool:
    """Put the reader-height register back; never raises. Caller holds the lock."""
    try:
      await self._apply_dm27(original)
    except Exception:
      logger.warning("Tecan: could not restore DM27 to %d", original, exc_info=True)
      return False
    return True

  def _barcode_aims(
    self, level: int, deltas: tuple[int, ...], pitch: int
  ) -> list[tuple[int, int, int]]:
    """Translate aim offsets into (controller level, DM27, delta) triples.

    Aim height is the requested level's position minus ``DM27``, so a larger ``DM27`` aims lower and
    a negative one cannot express an aim above a level (the firmware stores ``-N`` as ``1300+N``).
    Offsets above a level are therefore reached from the level above it: level ``L-1`` sits one
    pitch higher, and ``DM27 = pitch - delta`` brings it back down to ``delta`` above level ``L``.
    Level 1 has no level above it, so upward offsets cannot be expressed there and are skipped.
    """
    aims: list[tuple[int, int, int]] = []
    seen: set[tuple[int, int]] = set()
    for delta in deltas:
      if delta > 0:
        if level < 2:
          logger.info("read_barcode: no level above level 1 to aim from; skipping aim +%d", delta)
          continue
        if delta >= pitch:
          raise ValueError(
            f"read_barcode: aim +{delta} is at least one level pitch ({pitch}) above the level"
          )
        aim = (level - 1, pitch - delta, delta)
      else:
        aim = (level, -delta, delta)
      key = aim[:2]
      if key in seen:
        continue
      seen.add(key)
      aims.append(aim)
    if not aims:
      raise ValueError("read_barcode: no usable aim offset for this level")
    return aims

  async def read_barcode(
    self,
    cartridge: int,
    level: int,
    *,
    sweep: Optional[Sequence[int]] = None,
    timeout: float = 150,
  ) -> BarcodeRead:
    """Read the barcode of the plate stored at a cartridge level.

    Sequence, measured on an LPT220 EVO: position the carousel (``WR DM0``/``WR DM5``), switch the
    reader on (``ST 1712``), activate two-way communication (``ST 1200``), put the lift into barcode
    positioning (``ST 1910``) and scan (``ST 1710``); ``ST 1713``/``RS 1200``/``RS 1910`` leave the
    mode again.

    **Aim sweep.** The reader's vertical window is narrow and the plate label does not sit exactly
    at the level's nominal read height on this unit: at ``DM27 = 0`` the beam lands 1-2 mm below the
    barcode, and a plate high in its slot was only read 50-100 register units above nominal. A
    single nominal scan is therefore not reliable. ``sweep`` is the ordered list of aim offsets, in
    register units relative to the requested level's nominal position, negative below and positive
    above; the scan stops at the first position that returns a payload. The offsets are applied
    through ``DM27``, which is read before and restored afterwards.

    ``ST 1910`` changes the positioning mode of the whole handler, so this is the one method that
    depends on ``RS 1910`` restoring normal handling. That exit is confirmed through ``DM200``
    (16384 in barcode mode on this unit) rather than assumed: if it still reports barcode mode the
    session is faulted instead of being left in an unknown state. ``DM27`` is likewise verified by
    readback and restored on every path, including failures.

    Args:
      cartridge: controller cartridge address, 1-based.
      level: controller level, 1-based from the bottom of the cartridge.
      sweep: aim offsets in register units, tried in order, stopping at the first read. Defaults to
        nominal first and then 50, 100, 150 and 200 units above it. Pass ``(0,)`` for a single
        nominal scan.
      timeout: whole-operation deadline in seconds. Each scan costs a few seconds, so a wide sweep
        needs a correspondingly larger deadline.

    Returns:
      The scan result. ``BarcodeRead.barcode`` is ``None`` when every aim offset read nothing
      (``??``), which is a normal outcome for an empty slot. ``aim_delta`` reports which offset
      produced the result.

    Raises:
      ValueError: Invalid address, or a sweep the geometry cannot express.
      CarouselError: The controller is busy or faulty, the reply carries no scan frame, ``DM27``
        could not be restored, or the handler could not be confirmed out of barcode positioning
        mode.
      TimeoutError: The operation did not complete before the deadline.
    """
    self._validate_position(cartridge, level, "read_barcode")
    deltas = tuple(_DEFAULT_BCR_SWEEP if sweep is None else sweep)
    if not deltas:
      raise ValueError("read_barcode: sweep must contain at least one aim offset")
    if any(type(delta) is not int for delta in deltas):  # type: ignore[arg-type]
      raise ValueError(f"read_barcode: aim offsets must be integers, got {deltas!r}")

    async def _read() -> BarcodeRead:
      await self._check_ready("read_barcode")
      await self._require_sensors(shovel=False, station=False, stage="read_barcode")
      barcode_mode_at_entry = await self._read_int(b"RD DM200\r") == _BCR_PROCESS_WORD
      original = await self._read_int(b"RD DM27\r")
      pitch = await self._read_int(b"RD DM23\r") if any(delta > 0 for delta in deltas) else 0
      aims = self._barcode_aims(level, deltas, pitch)
      await self._select_position(cartridge, aims[0][0])
      logger.info(
        "Reading barcode for cartridge %d level %d over %d aim position(s); DM27 starts at %d",
        cartridge,
        level,
        len(aims),
        original,
      )
      result: Optional[BarcodeRead] = None
      positions: list[str] = []
      clean = True
      restored = True
      dm27 = original
      positioned = aims[0][0]
      try:
        await self._scan_command(b"ST 1712\r", stage="read_barcode: reader on")
        await self._scan_command(b"ST 1200\r", stage="read_barcode: two-way on")
        self._bcr_mode = True
        for requested_level, offset, delta in aims:
          if offset != dm27:
            await self._apply_dm27(offset)
            dm27 = offset
          if requested_level != positioned:
            await self._select_position(cartridge, requested_level)
            positioned = requested_level
          await self._scan_command(b"ST 1910\r", stage="read_barcode: bcr positioning")
          await self._wait_ready("read_barcode: bcr positioning")
          frame = await self._scan_command(
            b"ST 1710\r", stage="read_barcode: scan", expect_scan=True
          )
          position, payload = _parse_scan_frame(frame)
          if position is None:
            raise CarouselError(
              "read_barcode: the scan reply carried no SPS position field, so no scan is evident",
              stage="read_barcode",
            )
          if payload is None:
            # A position without a payload segment is an incomplete read, not a verdict about the
            # slot: try the next aim offset rather than abandoning the slot.
            logger.warning(
              "read_barcode: scan at aim %+d returned position %r without a barcode payload",
              delta,
              position,
            )
            positions.append(position)
            continue
          result = BarcodeRead(
            cartridge=cartridge,
            level=level,
            position=position,
            payload=payload,
            barcode=None if payload == "??" else payload[:-1],
            aim_delta=delta,
          )
          if result.barcode is not None:
            logger.info(
              "Barcode at cartridge %d level %d (aim %+d): %s",
              cartridge,
              level,
              delta,
              result.barcode,
            )
            break
      finally:
        restored = await self._restore_dm27(original)
        clean = await self._leave_bcr_mode(
          stage="read_barcode", barcode_mode_at_entry=barcode_mode_at_entry
        )
      if not restored:
        raise CarouselError(
          f"read_barcode: DM27 could not be restored to {original}; the reader height is left "
          "changed, check it before the next scan",
          stage="read_barcode",
        )
      if not clean:
        raise CarouselError(
          "read_barcode: the scan is back but the handler could not be confirmed out of barcode "
          "positioning mode"
          + (f": {self._bcr_exit_problem}" if self._bcr_exit_problem else "")
          + "; do not send handling commands",
          stage="read_barcode",
        )
      if result is None:
        raise CarouselError(
          "read_barcode: no scan frame carried a barcode payload at any of "
          f"{len(aims)} aim position(s); last positions reported: {positions}",
          stage="read_barcode",
        )
      if result.barcode is None:
        logger.info(
          "No barcode at cartridge %d level %d over %d aim position(s)",
          cartridge,
          level,
          len(aims),
        )
      return result

    return await self._operation(_read, stage="read_barcode", timeout=timeout)

  # Handling

  async def export_plate(self, cartridge: int, level: int, *, timeout: float = 60) -> None:
    """Retrieve the plate from a cartridge level onto the transfer station (``ST 1905``).

    The carousel rotates and lifts, picks the plate with the shovel and places it on the transfer
    station. Nothing is retried: an error, a deadline or a sensor contradiction raises and leaves
    the physical state to be confirmed by the operator.
    """
    self._validate_position(cartridge, level, "export_plate")

    async def _export() -> None:
      await self._check_ready("export_plate")
      await self._require_sensors(shovel=False, station=False, stage="export_plate")
      await self._select_position(cartridge, level)
      logger.info("Exporting plate from cartridge %d level %d", cartridge, level)
      await self._write(b"ST 1905\r")
      await self._wait_ready("export_plate")
      await self._confirm_delivery("export_plate")
      logger.info("Plate from cartridge %d level %d is on the transfer station", cartridge, level)

    await self._operation(_export, stage="export_plate", timeout=timeout)

  async def import_plate(self, cartridge: int, level: int, *, timeout: float = 60) -> None:
    """Move the plate from the transfer station into a cartridge level (``ST 1904``)."""
    self._validate_position(cartridge, level, "import_plate")

    async def _import() -> None:
      await self._check_ready("import_plate")
      await self._require_sensors(shovel=False, station=True, stage="import_plate")
      await self._select_position(cartridge, level)
      logger.info(
        "Importing plate from the transfer station to cartridge %d level %d", cartridge, level
      )
      await self._write(b"ST 1904\r")
      await self._wait_ready("import_plate")
      if await self._read_bool(b"RD 1813\r"):
        raise CarouselError(
          "import_plate: handling reports ready but the transfer station still senses a plate",
          stage="import_plate",
        )
      logger.info("Plate stored in cartridge %d level %d", cartridge, level)

    await self._operation(_import, stage="import_plate", timeout=timeout)

  async def put_plate_on_transfer_station(
    self, cartridge: int, level: int, *, timeout: float = 60
  ) -> None:
    """Place the plate already on the shovel onto the transfer station (``ST 1906``).

    This is the dedicated way to release a plate left on the shovel; it does not start a new
    cartridge access. The transfer station must be empty and the shovel occupied.
    """
    self._validate_position(cartridge, level, "put_plate_on_transfer_station")

    async def _put() -> None:
      await self._check_ready("put_plate_on_transfer_station")
      await self._require_sensors(shovel=True, station=False, stage="put_plate_on_transfer_station")
      await self._set_register("DM0", cartridge)
      await self._set_register("DM5", level)
      await self._wait_ready("put_plate_on_transfer_station")
      logger.info("Placing the shovel plate on the transfer station")
      await self._write(b"ST 1906\r")
      await self._wait_ready("put_plate_on_transfer_station")
      await self._confirm_delivery("put_plate_on_transfer_station")
      logger.info("Plate is on the transfer station")

    await self._operation(_put, stage="put_plate_on_transfer_station", timeout=timeout)

  async def get_plate_from_transfer_station(
    self, cartridge: int, level: int, *, timeout: float = 60
  ) -> None:
    """Pick the plate up from the transfer station (``ST 1907``), leaving it on the shovel."""
    self._validate_position(cartridge, level, "get_plate_from_transfer_station")

    async def _get() -> None:
      await self._check_ready("get_plate_from_transfer_station")
      await self._require_sensors(
        shovel=False, station=True, stage="get_plate_from_transfer_station"
      )
      await self._set_register("DM0", cartridge)
      await self._set_register("DM5", level)
      await self._wait_ready("get_plate_from_transfer_station")
      logger.info("Taking the plate from the transfer station")
      await self._write(b"ST 1907\r")
      await self._wait_ready("get_plate_from_transfer_station")
      if not await self._read_bool(b"RD 1812\r"):
        raise CarouselError(
          "get_plate_from_transfer_station: no plate is sensed on the shovel afterwards",
          stage="get_plate_from_transfer_station",
        )

    await self._operation(_get, stage="get_plate_from_transfer_station", timeout=timeout)

  async def pick_plate(self, cartridge: int, level: int, *, timeout: float = 60) -> None:
    """Pick a plate from a cartridge level onto the shovel (``ST 1908``).

    The shovel must be empty. A plate on the transfer station is not affected.
    """
    self._validate_position(cartridge, level, "pick_plate")

    async def _pick() -> None:
      await self._check_ready("pick_plate")
      await self._require_sensors(shovel=False, station=None, stage="pick_plate")
      await self._select_position(cartridge, level)
      logger.info("Picking plate from cartridge %d level %d", cartridge, level)
      await self._write(b"ST 1908\r")
      await self._wait_ready("pick_plate")
      if not await self._read_bool(b"RD 1812\r"):
        raise CarouselError(
          "pick_plate: no plate is sensed on the shovel afterwards", stage="pick_plate"
        )

    await self._operation(_pick, stage="pick_plate", timeout=timeout)

  async def place_plate(self, cartridge: int, level: int, *, timeout: float = 60) -> None:
    """Place the plate carried on the shovel into a cartridge level (``ST 1909``)."""
    self._validate_position(cartridge, level, "place_plate")

    async def _place() -> None:
      await self._check_ready("place_plate")
      await self._require_sensors(shovel=True, station=None, stage="place_plate")
      await self._select_position(cartridge, level)
      logger.info("Placing plate into cartridge %d level %d", cartridge, level)
      await self._write(b"ST 1909\r")
      await self._wait_ready("place_plate")
      if await self._read_bool(b"RD 1812\r"):
        raise CarouselError(
          "place_plate: the shovel still senses a plate after the placement", stage="place_plate"
        )

    await self._operation(_place, stage="place_plate", timeout=timeout)

  async def move_plate(
    self,
    cartridge: int,
    level: int,
    to_cartridge: int,
    to_level: int,
    *,
    timeout: float = 120,
  ) -> None:
    """Move a plate between two cartridge levels inside the carousel (``ST 1908`` + ``ST 1909``).

    The transfer station is not used and must be empty. Both the pick and the place are verified
    before the next step, so a failed pick never turns into a placement into an unknown position.
    """
    self._validate_position(cartridge, level, "move_plate")
    self._validate_position(to_cartridge, to_level, "move_plate")

    async def _move() -> None:
      await self._check_ready("move_plate")
      await self._require_sensors(shovel=False, station=False, stage="move_plate")
      await self._select_position(cartridge, level)
      logger.info("Moving plate from cartridge %d level %d", cartridge, level)
      await self._write(b"ST 1908\r")
      await self._wait_ready("move_plate")
      try:
        await self._wait_sensor(b"RD 1812\r", True, stage="move_plate: pick")
      except CarouselError as error:
        raise CarouselError(
          f"move_plate: nothing was picked from cartridge {cartridge} level {level}",
          stage="move_plate",
        ) from error
      await self._select_position(to_cartridge, to_level)
      await self._write(b"ST 1909\r")
      await self._wait_ready("move_plate")
      await self._wait_sensor(b"RD 1812\r", False, stage="move_plate: place")
      logger.info("Plate stored in cartridge %d level %d", to_cartridge, to_level)

    await self._operation(_move, stage="move_plate", timeout=timeout)
