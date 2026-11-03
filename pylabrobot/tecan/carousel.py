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
from typing import Awaitable, Callable, Mapping, Optional, TypeVar

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

  Not implemented, because their effect on this installation is not established: door and gate
  control (``ST 1901``, ``ST 1902``), terminate access (``ST 1903``), barcode reading positions
  (``ST 1910`` and its still-unresolved mode exit), short import/export, and the climate and shaker
  commands of incubator variants. ``ST 1910`` is deliberately absent: it changes the positioning
  mode of the whole handler, and no documented exit has been verified.
  """

  _EXCHANGE_TIMEOUT = 1.0
  _SETTLE_DELAY = 0.3
  _POLL_INTERVAL = 0.2

  def __init__(
    self,
    port: str,
    *,
    number_of_cartridges: int = 10,
    levels_per_cartridge: int = 22,
    cassette_slots: Optional[Mapping[int, int]] = None,
    cassette_level_step: Optional[Mapping[int, int]] = None,
    cassette_levels_increase_downward: Optional[Mapping[int, bool]] = None,
  ) -> None:
    """Configure an explicit operator-selected serial port without opening it.

    Args:
      port: serial device, chosen by the operator rather than discovered automatically.
      number_of_cartridges: cartridges on the rotating plate. The default is the manual's maximum
        of ten; the installed count is an installation detail.
      levels_per_cartridge: plate positions per cartridge for the installed cassettes. The default
        of 22 is a standard cassette; half-height and deep-well cassettes hold fewer.
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

  # Transport

  async def _exchange(self, command: bytes) -> bytes:
    """Send one CR-terminated frame and read one CRLF-terminated reply; caller holds the lock."""
    logger.debug("Tecan request %r", command)
    await self.io.write(command)
    reply = bytearray()
    for _ in range(16):
      chunk = await self.io.read(1)
      if not chunk:
        raise TimeoutError(f"Incomplete Tecan reply to {command!r}: {bytes(reply)!r}")
      reply.extend(chunk)
      if chunk == b"\n":
        break
    raw = bytes(reply)
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
    max_level = min(
      self.levels_per_cartridge,
      1 + (self.cassette_slots[cartridge] - 1) * self.cassette_level_step[cartridge],
    )
    if type(level) is not int or not 1 <= level <= max_level:
      raise ValueError(f"{stage}: level must be an integer in 1..{max_level}, got {level!r}")

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
    if self.cassette_levels_increase_downward[cartridge]:
      # Controller level 1 is the topmost slot: counting from the top runs with the level axis.
      level = 1 + (slot - 1) * step
    else:
      # Controller level 1 is the bottommost slot: the topmost slot sits at the highest level.
      level = 1 + (slots - 1) * step - (slot - 1) * step
    if level > self.levels_per_cartridge:
      raise ValueError(
        f"level_for_slot: slot {slot} maps to level {level}, beyond the configured "
        f"{self.levels_per_cartridge} levels"
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
    if self.cassette_levels_increase_downward[cartridge]:
      offset = level - 1
    else:
      offset = 1 + (slots - 1) * step - level
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
