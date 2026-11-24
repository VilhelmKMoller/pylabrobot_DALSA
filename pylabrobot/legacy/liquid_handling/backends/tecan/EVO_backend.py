import asyncio
import logging
import math
from abc import ABCMeta, abstractmethod
from dataclasses import dataclass, field
from typing import (
  Dict,
  List,
  Optional,
  Sequence,
  Tuple,
  TypeVar,
  Union,
  cast,
)

from pylabrobot.io.usb import USB
from pylabrobot.legacy.liquid_handling.backends.backend import (
  LiquidHandlerBackend,
)
from pylabrobot.legacy.liquid_handling.backends.tecan.errors import (
  TecanLihaError,
  TecanTipsNotFetchedError,
  TecanError,
  error_code_to_exception,
)
from pylabrobot.legacy.liquid_handling.backends.tecan import liha, mca
from pylabrobot.legacy.liquid_handling.backends.tecan.liha_firmware import LihaFirmware
from pylabrobot.legacy.liquid_handling.backends.tecan.liha_firmware_commands import (
  COMMANDS as LIHA_COMMANDS,
)
from pylabrobot.legacy.liquid_handling.errors import ChannelizedError
from pylabrobot.resources.errors import HasTipError, NoTipError
from pylabrobot.resources.itemized_resource import ItemizedResource
from pylabrobot.legacy.liquid_handling.standard import (
  Drop,
  DropTipRack,
  MultiHeadAspirationContainer,
  MultiHeadAspirationPlate,
  MultiHeadDispenseContainer,
  MultiHeadDispensePlate,
  Pickup,
  PickupTipRack,
  ResourceDrop,
  ResourceMove,
  ResourcePickup,
  SingleChannelAspiration,
  SingleChannelDispense,
)
from pylabrobot.resources import (
  Coordinate,
  Resource,
  TecanPlateCarrier,
  TecanTip,
  Tip,
  TipRack,
  TipSpot,
  Trash,
)

T = TypeVar("T")

LIHA_DEFAULT_CLASS = "Water free dispense"
UNKNOWN = "unknown"  # LiHa channel contents after an interrupted liquid step
# LiHa errors after which the tips are raised straight up (PAZ to the Z range, Z only): liquid
# detection / clot / DiTi errors, where the arm itself is fine. EVOware then shows the operator a
# dialog (retry, continue at z_max, or abort = PAZ to the Z range); PLR neither retries nor
# continues. Not after a crash (10), on an uninitialized arm (1, 7), with the door open (250), or
# for a refused command (2, 3): the arm stays where it is (read positions before recovering).
LIHA_RAISE_Z_AFTER = frozenset({9, 11, 12, 17, 18, 19, 20, 21, 24, 25, 26, 27})


@dataclass(frozen=True)
class _LihaTarget:
  """A container for ``liha_mix``, shaped like a liquid-handling op for ``_liha_target``."""

  resource: Resource
  offset: Coordinate = field(default_factory=Coordinate.zero)


class TecanLiquidHandler(LiquidHandlerBackend, metaclass=ABCMeta):
  """
  Abstract base class for Tecan liquid handling robot backends.
  """

  @abstractmethod
  def __init__(
    self,
    packet_read_timeout: int = 120,
    read_timeout: int = 300,
    write_timeout: int = 300,
  ):
    """

    Args:
      packet_read_timeout: The timeout for reading packets from the Tecan machine in seconds.
      read_timeout: The timeout for reading from the Tecan machine in seconds.
    """

    super().__init__()
    self.io = USB(
      human_readable_device_name="Tecan EVO",
      packet_read_timeout=packet_read_timeout,
      read_timeout=read_timeout,
      write_timeout=write_timeout,
      id_vendor=0x0C47,
      id_product=0x4000,
    )

    self._cache: Dict[str, List[Optional[int]]] = {}

  def _assemble_command(self, module: str, command: str, params: List[Optional[int]]) -> str:
    """Assemble a firmware command to the Tecan machine.

    Args:
      module: 2 character module identifier (C5 for LiHa, ...)
      command: 3 character command identifier
      params: list of integer parameters

    Returns:
      A string containing the assembled command.
    """

    cmd = module + command + ",".join(str(a) if a is not None else "" for a in params)
    return f"\02{cmd}\00"

  def parse_response(self, resp: bytes) -> Dict[str, Union[str, int, List[Union[int, str]]]]:
    """Parse a machine response string

    Args:
      resp: The response string to parse.

    Returns:
      A dictionary containing the parsed values.
    """

    # Preserve the existing prefix/module/status/body/suffix layout. The local
    # device command manuals do not specify USB delimiter or status-bit rules.
    if len(resp) < 5:
      raise ValueError("Tecan response is truncated: expected at least 5 bytes")

    module = resp[1:3].decode("utf-8")
    ret = int(resp[3]) ^ (1 << 7)
    if ret != 0:
      raise error_code_to_exception(module, ret)

    data: List[Union[int, str]] = []
    for x in resp[4:-1].decode("utf-8").split(","):
      if len(x) == 0:  # Skip empty values
        continue
      # RoMa(C1) and LiHa(C5) should only have integer values
      # MCA can return both integers and strings
      data.append(int(x) if x.lstrip("-").isdigit() else x)

    return {"module": module, "data": data}

  async def send_command(
    self,
    module: str,
    command: str,
    params: Optional[List[Optional[int]]] = None,
    write_timeout: Optional[int] = None,
    read_timeout: Optional[int] = None,
    wait=True,
  ):
    """Send a firmware command to the Tecan machine. Caches `set` commands and ignores if
    redundant.

    Args:
      module: 2 character module identifier (C5 for LiHa, ...)
      command: 3 character command identifier
      params: list of integer parameters
      write_timeout: write timeout in seconds. If None, `self.write_timeout` is used.
      read_timeout: read timeout in seconds. If None, `self.read_timeout` is used.
      wait: If True, wait for a response. If False, return `None` immediately after sending.

    Returns:
      A dictionary containing the parsed response, or None if no response was read within `timeout`.
    """

    if command[0] == "S" and params is not None:
      k = module + command
      if k in self._cache and self._cache[k] == params:
        return
      self._cache[k] = params

    cmd = self._assemble_command(module, command, [] if params is None else params)

    await self.io.write(cmd.encode(), timeout=write_timeout)
    if not wait:
      return None

    resp = await self.io.read(timeout=read_timeout)
    if len(resp) >= 5 and resp[1:3] != module.encode("utf-8"):
      raise ValueError(f"Expected response module {module}, got {resp[1:3]!r}")
    return self.parse_response(resp)

  async def setup(self):
    await super().setup()
    await self.io.setup()

  async def stop(self):
    await self.io.stop()


class EVOBackend(TecanLiquidHandler):
  """
  Interface for the Tecan Freedom EVO series
  """

  LIHA = "C5"
  ROMA = "C1"
  MCA = "W1"
  PNP = "W2"

  # (site, column, row, tips, Z) after a successful liha_mix; any other LiHa operation clears it
  _liha_mixed: Optional[tuple] = None
  _liha_last_stride: int = 1

  def __init__(
    self,
    diti_count: int = 0,
    packet_read_timeout: int = 12,
    read_timeout: int = 120,
    write_timeout: int = 60,
  ):
    """Create a new EVO interface.

    Args:
      packet_read_timeout: timeout in seconds for reading a single packet.
      read_timeout: timeout in seconds for reading a full response.
      write_timeout: timeout in seconds for writing a command.
    """

    super().__init__(
      packet_read_timeout=packet_read_timeout,
      read_timeout=read_timeout,
      write_timeout=write_timeout,
    )

    self._num_channels: Optional[int] = None
    self.diti_count = diti_count
    # channels [num_channels - diti_count, num_channels) configured for disposable tips

    self._liha_connected: Optional[bool] = None
    self._roma_connected: Optional[bool] = None
    self._pnp_connected: Optional[bool] = None
    self._mca_connected: Optional[bool] = None
    self._mca_operation_config = None
    self._mca_operation_lock = asyncio.Lock()
    # LiHa hardware options for the firmware layer (``liha_firmware.HARDWARE_OPTIONS``); unknown
    # until probed on the instrument, so none.
    self.liha_options: Tuple[str, ...] = ()
    # LiHa X move parameters (SFX EndSpeed, Accel) for every LiHa plan when set: replaces the
    # planners' SFX values and is sent before each plan, so all LiHa X moves use it. None = as
    # EVOware (the planners' values).
    self.liha_x_move: Optional[Tuple[int, int]] = None
    # Operator-measured LiHa (dX, dY) corrections in 1/10 mm per worktable site (liha.LihaGeometry).
    self.liha_site_offsets: Dict["liha.LihaSite", Tuple[int, int]] = {}
    # Operator-measured LiHa Z corrections in 1/10 mm per site (negative = lower).
    self.liha_site_z_offsets: Dict["liha.LihaSite", int] = {}

    self._z_traversal_height = 210  # mm, the default value for SHZ command
    self._z_roma_traversal_height = (
      68.7  # mm, is what was used to develop this but possibly too low
    )

  async def move_x_only(self, planner, name, target):
    """Experimental X-only planning; nonempty moves are blocked, not dispatched.

    Reads the configured arms using the already-open transport. This does not
    open USB, validate homing or geometry, or protect any existing movement path.
    The lock and fault latch apply only to this backend instance and this entry point.
    """
    from .x_backend_planner import BackendXPlanner

    if not hasattr(self, "_x_backend_planner"):
      self._x_backend_planner = BackendXPlanner(self, planner)
    return await self._x_backend_planner.request(planner, name, target)

  def x_coordinator(self, **kwargs):
    """The backend's X collision-avoidance coordinator (created on first use).

    Keyword arguments are passed to ``EvoXCoordinator`` on first creation only.
    Use ``await backend.x_coordinator().move_x("mca", 9100)``.
    """
    from .x_coordinator import EvoXCoordinator
    from .x_layout import EVO_Z_TRAVEL, evo_planner

    if getattr(self, "_x_coordinator", None) is None:
      planner = kwargs.pop("planner", None) or evo_planner()
      kwargs.setdefault("z_travel", EVO_Z_TRAVEL)
      self._x_coordinator = EvoXCoordinator(self, planner, **kwargs)
    elif kwargs:
      raise ValueError("x_coordinator already created; settings cannot be changed")
    return self._x_coordinator

  @property
  def num_channels(self) -> int:
    """The number of pipette channels present on the robot."""

    if self._num_channels is None:
      raise RuntimeError("has not loaded num_channels, forgot to call `setup`?")
    return self._num_channels

  @property
  def liha_connected(self) -> bool:
    """Whether LiHa arm is present on the robot."""

    if self._liha_connected is None:
      raise RuntimeError("liha_connected not set, forgot to call `setup`?")
    return self._liha_connected

  @property
  def roma_connected(self) -> bool:
    """Whether RoMa arm is present on the robot."""

    if self._roma_connected is None:
      raise RuntimeError("roma_connected not set, forgot to call `setup`?")
    return self._roma_connected

  @property
  def pnp_connected(self) -> bool:
    """Whether PnP arm is present on the robot."""

    if self._pnp_connected is None:
      raise RuntimeError("pnp_connected not set, forgot to call `setup`?")
    return self._pnp_connected

  @property
  def mca_connected(self) -> bool:
    """Whether MCA arm is present on the robot."""

    if self._mca_connected is None:
      raise RuntimeError("mca_connected not set, forgot to call `setup`?")
    return self._mca_connected

  async def door_lock(self):
    """Request both locks; a successful response does not verify physical engagement.

    Failure stops the sequence and can leave the door only partially locked.
    """
    for lock in (1, 2):
      try:
        await self.send_command("O1", "ALO", [lock, 1])
      except TecanError as exc:
        if exc.module == "O1" and exc.error_code == 18:
          logging.getLogger(__name__).warning(
            "Door lock rejected (O1 error 18): door may be open; close the door and "
            "check both locks. The lock state may be partial or unknown."
          )
        raise

  async def door_unlock(self):
    """Request both locks release, not physical opening; failures may leave partial state."""
    await self.send_command("O1", "ALO", [1, 0])
    await self.send_command("O1", "ALO", [2, 0])

  def serialize(self) -> dict:
    return {**super().serialize(), **self.io.serialize()}

  async def setup(self, skip_initialization: bool = False):
    """Connect and initialize all arms (homing, parking, LiHa flush, MCA plunger).

    Args:
      skip_initialization: only connect and read the arm configuration, without
        any motion. For an instrument that is already initialized since power-on.
    """

    await super().setup()

    if skip_initialization:
      await self._connect_without_initialization()
      return
    await self.initialize()

  async def initialize(self):
    """Full EVO initialization on an open connection: all arms homed and parked,
    MCA plunger initialized, LiHa plungers initialized and flushed."""
    self._liha_connected = await self.setup_arm(EVOBackend.LIHA)
    self._mca_connected = await self.setup_arm(EVOBackend.MCA)
    self._roma_connected = await self.setup_arm(EVOBackend.ROMA)

    if self.roma_connected:  # position_initialization_x in reverse order from setup_arm
      self.roma = RoMa(self, EVOBackend.ROMA)
      await self.roma.position_initialization_x()
      # move to home position (TBD) after initialization
      await self._park_roma()

    self._head96_installed = self.mca_connected
    if self.mca_connected:
      self.mca = Mca(self, EVOBackend.MCA)
      # move to home position (TBD) after initialization
      await self._park_mca()
      # PIA does not initialize the plunger (MCA manual p.34); EVOware runs PID here.
      await self.mca_initialize_head_drive()

    if self.liha_connected:
      self.liha = LiHa(self, EVOBackend.LIHA)
      await self.liha.position_initialization_x()



    self._num_channels = await self.liha.report_number_tips()
    self._x_range = await self.liha.report_x_param(5)
    self._y_range = (await self.liha.report_y_param(5))[0]
    self._z_range = (await self.liha.report_z_param(5))[0]

    # Initialize plungers. Assumes wash station assigned at rail 1.
    await self.liha.set_z_travel_height([self._z_range] * self.num_channels)
    await self.liha.position_absolute_all_axis(45, 1031, 90, [1200] * self.num_channels)
    await self.liha.initialize_plunger(self._bin_use_channels(list(range(self.num_channels))))
    await self.liha.position_valve_logical([1] * self.num_channels)
    await self.liha.move_plunger_relative([100] * self.num_channels)
    await self.liha.position_valve_logical([0] * self.num_channels)
    await self.liha.set_end_speed_plunger([1800] * self.num_channels)
    await self.liha.move_plunger_relative([-100] * self.num_channels)
    await self.liha.position_absolute_all_axis(45, 1031, 90, [self._z_range] * self.num_channels)

  async def _arm_present(self, module) -> bool:
    try:
      await self.send_command(module, command="RFV", params=[0])
    except TecanError as e:
      if e.error_code == 5:  # device not implemented
        return False
      raise
    return True

  async def _connect_without_initialization(self):
    """Detect arms and read ranges; sends report commands only."""
    self._liha_connected = await self._arm_present(EVOBackend.LIHA)
    self._mca_connected = await self._arm_present(EVOBackend.MCA)
    self._roma_connected = await self._arm_present(EVOBackend.ROMA)
    self._head96_installed = self.mca_connected
    if self.roma_connected:
      self.roma = RoMa(self, EVOBackend.ROMA)
    if self.mca_connected:
      self.mca = Mca(self, EVOBackend.MCA)
    if self.liha_connected:
      self.liha = LiHa(self, EVOBackend.LIHA)
      self._num_channels = await self.liha.report_number_tips()
      self._x_range = await self.liha.report_x_param(5)
      self._y_range = (await self.liha.report_y_param(5))[0]
      self._z_range = (await self.liha.report_z_param(5))[0]
    else:
      self._num_channels = 0

  async def setup_arm(self, module):
    try:
      if module == EVO.MCA:
        await self.send_command(module, command="PIB")

      await self.send_command(module, command="PIA")
    except TecanError as e:
      if e.error_code == 5:
        return False
      raise e

    if module != EVO.MCA:
      await self.send_command(module, command="BMX", params=[2])

    return True

  async def _park_liha(self):
    await self.liha.set_z_travel_height([self._z_range] * self.num_channels)
    await self.liha.position_absolute_all_axis(45, 1031, 90, [self._z_range] * self.num_channels)

  async def _park_roma(self):
    await self.roma.set_vector_coordinate_position(1, 9000, 2000, 2464, 1800, None, 1, 0)
    await self.roma.action_move_vector_coordinate_position()

  async def _park_mca(self):
    """Run the existing MCA parking command sequence."""
    await self.send_command(EVO.MCA, command="PIA")
    await self.send_command(EVO.MCA, command="PAA", params=[6000, 1000, None])
    await self.send_command(EVO.MCA, command="BMA", params=[0, 0, 0])


  # ============== LiquidHandlerBackend methods: LiHa ==============
  #
  # EVOware's LiHa steps, planned by ``liha.py`` (replayed against complete EVOware runs in
  # liha_planner_tests.py) from the EVOware configuration and liquid classes. Every plan is
  # validated command by command by the firmware layer (``LihaFirmware``) before the first command
  # is sent, and every X move is preceded by ``x_coordinator().clear_path("liha", x)``.
  #
  # Supported, as on the REFINES deck: DiTis fetched from a DiTi rack and dropped into the DiTi
  # waste chute; aspirate with liquid detection and dispense (free or wet contact) of the same
  # volume on all channels, with consecutive channels over consecutive rows of one column.
  # EVOware "Mix" with a mix liquid class (``liha_mix``); an aspirate right after it in the same
  # wells follows on without moving or taking an air gap, as EVOware does.
  # DiTis go into the chute or back into a rack (EVOware "Set DiTis Back").
  # Not supported yet (raises): ``mix`` in aspirate/dispense ops,
  # multi-dispense, partial dispense, flow rates / liquid heights / blow-out other than the liquid
  # class's.
  #
  # On an instrument error the tips are raised straight up when the arm is fine (liquid detection
  # and DiTi errors, ``LIHA_RAISE_Z_AFTER``), otherwise the arm is left where it stopped. Nothing
  # is retried (EVOware asks its operator). The contents of the channels involved become unknown:
  # they can only drop their DiTis.

  def _liha_firmware(self) -> LihaFirmware:
    fw = getattr(self, "_liha_fw", None)
    if fw is None or fw.num_tips != self.num_channels:

      async def sender(module: str, command: str, params: List[Optional[int]]):
        return await self._liha_send(command, params or None)

      fw = LihaFirmware(sender, EVOBackend.LIHA, self.liha_options, self.num_channels)
      self._liha_fw = fw
    return fw

  async def _liha_run(self, plan: "liha.Plan") -> list:
    """Validate the whole plan, then send it; returns the replies (None for host waits)."""
    fw = self._liha_firmware()
    if self.liha_x_move is not None:
      x_move: List[Optional[int]] = list(self.liha_x_move)
      plan = liha.Plan(
        [("SFX", x_move)]
        + [(c, x_move if c == "SFX" else p) for c, p in plan.commands]
      )
    for command, params in plan.commands:
      if command == "WAIT":
        continue
      fw.check_wire(command, params)
      if LIHA_COMMANDS[command]["moves_x"] and (command != "PAA" or params[0] is None):
        raise RuntimeError(f"LiHa {command} moves X outside an absolute PAA; refused")
    replies: list = []
    for command, params in plan.commands:
      if command == "WAIT":
        await asyncio.sleep((params[0] or 0) / 1000)
        replies.append(None)
        continue
      if command == "PAA":
        assert params[0] is not None  # checked above
        await self.x_coordinator().clear_path("liha", params[0])
      replies.append(await fw.execute_wire(command, params))
    return replies

  async def _liha_report(self, command: str, selector: int) -> List[int]:
    """One value per channel from RPZ/RPP (e.g. RPZ0 = current Z, RPP0 = plunger steps)."""
    data = (await self._liha_firmware().execute_wire(command, [selector]))["data"]
    if len(data) != self.num_channels or any(type(v) is not int for v in data):
      raise TecanError(f"unexpected LiHa {command}{selector} response {data!r}", EVOBackend.LIHA, 0)
    return list(data)

  def _liha_contents(self) -> Dict[int, object]:
    """Per channel: None (only air), ``(volume, liquid class name)``, or ``UNKNOWN``."""
    contents: Optional[Dict[int, object]] = getattr(self, "_liha_contents_state", None)
    if contents is None:
      contents = {}
      self._liha_contents_state = contents
    return contents

  def _liha_lc(self, name: Optional[str], volume: float):
    library = getattr(self, "liquid_classes", None)
    if library is None:
      raise RuntimeError("LiHa liquid classes need the EVOware configuration: call load_evoware_config()")
    return library.liha(name or LIHA_DEFAULT_CLASS, volume)

  def _liha_channels(self, use_channels: List[int]) -> List[int]:
    if not self.liha_connected:
      raise RuntimeError("no LiHa on this instrument")
    if sorted(set(use_channels)) != list(use_channels):
      raise ValueError(f"use_channels must be increasing without repeats, got {use_channels}")
    first_diti = self.num_channels - self.diti_count
    if not use_channels or use_channels[0] < first_diti or use_channels[-1] >= self.num_channels:
      raise ValueError(
        f"DiTis are configured on channels {first_diti}..{self.num_channels - 1} (diti_count"
        f" {self.diti_count}), got {use_channels}"
      )
    return [c + 1 for c in use_channels]

  def _liha_site(self, resource: Resource) -> "liha.LihaSite":
    """The EVOware worktable position of ``resource`` (or of the labware it belongs to).

    A labware that carries its own LiHa correction (``liha_offset`` = (dX, dY, dZ) in 1/10 mm, e.g.
    ``DiTi_1000ul_LiHa_box``) has it registered for this site, so every LiHa position on it uses it.
    """
    geometry = self.mca_geometry  # PLR resource -> EVOware placement (not MCA-specific)
    lw = geometry.labware_resource(resource)
    placement = geometry.placement(lw)
    holder = lw.parent
    assert holder is not None and holder.parent is not None
    site = liha.LihaSite(
      placement.grid, placement.site_number - 1, mca.evoware_name(holder.parent), mca.evoware_name(lw)
    )
    correction = getattr(lw, "liha_offset", None)
    if correction is not None:
      dx, dy, dz = correction
      self.liha_site_offsets[site] = (dx, dy)
      self.liha_site_z_offsets[site] = dz
    return site

  def _liha_target(
    self, ops: Sequence, use_channels: List[int], trough_column: int = 1, row_stride: bool = False
  ) -> Tuple["liha.LihaSite", int, int, List[int]]:
    """EVOware site, column and row for tip ``tips[0]`` of ops on consecutive channels.

    Wells/tip spots must be consecutive rows of one column, channel order = row order; a channel
    left out leaves its row out (tips 1, 2, 4 over rows 1, 2, 4; the planner decides whether that is
    supported). With ``row_stride`` the tips may go over every k-th row instead (tips 1-4 over rows
    2, 4, 6, 8); ``self._liha_last_stride`` is set to k. On a
    one-well labware (trough) EVOware's own tip positions are used: ``trough_column`` (1-based, of
    the columns EVOware addresses, 12 for "Trough 300ml MCA96") with tip n over row n; the Y offsets
    LiquidHandler computes to spread channels over a single well are therefore ignored, and other
    offsets are refused.
    """
    tips = self._liha_channels(use_channels)
    if len(ops) != len(use_channels):
      raise ValueError("one operation per channel")
    geometry = self.mca_geometry  # PLR resource -> EVOware placement (not MCA-specific)
    labware = {geometry.labware_resource(op.resource) for op in ops}
    if len(labware) != 1:
      raise NotImplementedError("LiHa operations on more than one labware at a time")
    lw = labware.pop()
    site = self._liha_site(lw)
    items = getattr(lw, "num_items", 0)
    if items == 1:
      for op in ops:
        if op.offset.x != 0 or op.offset.z != 0:
          raise NotImplementedError(f"offsets are not supported on {lw.name}, got {op.offset}")
      self._liha_last_stride = 1
      return site, trough_column, tips[0], tips
    for op in ops:
      if op.offset != Coordinate.zero():
        raise NotImplementedError(f"offsets are not supported, got {op.offset}")
    if not isinstance(lw, ItemizedResource):
      raise ValueError(f"{lw.name} has no wells or tip spots")
    places = []
    for op in ops:
      index = lw.index_of_item(op.resource)
      if index is None:
        raise ValueError(f"{op.resource.name} is not an item of {lw.name}")
      places.append(divmod(index, lw.num_items_y))  # (column, row), 0-based, column-major
    column, row = places[0]
    stride = 1
    if row_stride and len(places) > 1 and tips[1] - tips[0] == 1:
      stride = max(1, places[1][1] - row)
    for (c, r), tip in zip(places, tips):
      if c != column or r - row != stride * (tip - tips[0]):
        raise NotImplementedError(
          "LiHa channels must be over consecutive rows of one column, in channel order"
          + (" (or every k-th row)" if row_stride else "")
        )
    self._liha_last_stride = stride
    return site, column + 1, row + 1, tips

  def _liha_errors(self, exc: TecanLihaError, use_channels: List[int], requested: int = 0):
    """A ChannelizedError naming the failing channels (REE per tip, or RTS for DiTi errors), or
    None when the instrument did not say which channels failed."""
    errors: Dict[int, Exception] = {}
    if exc.tip_errors:
      for channel, code in exc.tip_errors.items():
        if channel in use_channels:
          errors[channel] = error_code_to_exception(EVOBackend.LIHA, code)
    elif exc.mounted_tips is not None and exc.error_code in (25, 26):
      for channel in use_channels:
        if not exc.mounted_tips >> channel & 1:
          errors[channel] = error_code_to_exception(EVOBackend.LIHA, exc.error_code)
    elif exc.mounted_tips is not None and exc.error_code == 27:
      for channel in use_channels:
        if exc.mounted_tips >> channel & 1 and not requested >> channel & 1:
          errors[channel] = error_code_to_exception(EVOBackend.LIHA, 27)
    return ChannelizedError(errors, error=str(exc)) if errors else None

  async def _liha_execute(self, plan: "liha.Plan", use_channels: List[int], requested: int = 0) -> list:
    try:
      return await self._liha_run(plan)
    except TecanLihaError as exc:
      if exc.error_code in LIHA_RAISE_Z_AFTER:
        await self._liha_raise_z(exc)
      channelized = self._liha_errors(exc, use_channels, requested)
      if channelized is not None:
        raise channelized from exc
      raise

  async def _liha_raise_z(self, exc: Exception) -> None:
    """After ``exc``: all tips straight up to the Z range (PAZ; X and Y stay). If that fails too,
    the arm stays where it is and the failure is noted on ``exc``."""
    try:
      await self._liha_firmware().execute_wire("PAZ", [self._z_range] * self.num_channels)
    except Exception as raise_exc:  # the original error matters more; keep it, note this one
      exc.add_note(f"raising the LiHa tips after the error failed too: {raise_exc!r}")
    else:
      exc.add_note(f"the LiHa tips were raised to Z {self._z_range}")

  async def _liha_require_tips(self, use_channels: List[int], mounted: bool):
    """RTS before a DiTi or liquid step: DiTis must (not) be on ``use_channels``."""
    tips = await self.liha_mounted_tips()
    wrong = [c for c in use_channels if bool(tips >> c & 1) != mounted]
    if wrong:
      error = NoTipError if mounted else HasTipError
      what = "has no DiTi" if mounted else "already has a DiTi"
      raise ChannelizedError({c: error(f"LiHa channel {c} {what} (RTS {tips:08b})") for c in wrong})

  @staticmethod
  def _liha_diti(ops: Sequence) -> str:
    """EVOware DiTi labware of the mounted tips (all channels the same)."""
    return mca.tip_rack_name(EVOBackend._mca_tip_model([op.tip for op in ops]))

  @staticmethod
  def _liha_check_ops(ops: Sequence) -> float:
    """The one volume of the ops; refuses settings the liquid class decides on EVOware."""
    for op in ops:
      unsupported = {
        "flow_rate": op.flow_rate,
        "liquid_height": op.liquid_height,
        "blow_out_air_volume": op.blow_out_air_volume,
        "mix": op.mix,
      }
      given = [k for k, v in unsupported.items() if v is not None]
      if given:
        raise NotImplementedError(
          f"LiHa {', '.join(given)}: not supported, the EVOware liquid class decides (leave None)"
        )
    volumes = {op.volume for op in ops}
    if len(volumes) != 1:
      raise NotImplementedError(f"LiHa needs the same volume on all channels, got {sorted(volumes)}")
    volume = float(volumes.pop())
    if not (math.isfinite(volume) and volume > 0):
      raise ValueError(f"volume must be positive, got {volume}")
    return volume

  async def pick_up_tips(self, ops: List[Pickup], use_channels: List[int], slow_x: bool = True):
    """EVOware "Get DiTis": fetch DiTis from a rack, one column, consecutive rows.

    Args:
      slow_x: move X with EVOware's slow settings around the fetch (default, as EVOware does on
        instruments with a RoMa; see ``liha.plan_get_ditis``).
    Raises ChannelizedError (NoTipError) for channels that did not get a DiTi.
    """
    for op in ops:
      if not isinstance(op.resource, TipSpot):
        raise ValueError(f"{op.resource.name} is not a tip spot")
    site, column, row, tips = self._liha_target(ops, use_channels)
    diti = self._liha_diti(ops)
    if diti != site.labware:
      raise ValueError(f"tip {diti!r} does not belong to rack {site.labware!r}")
    self._liha_mixed = None
    plan = liha.plan_get_ditis(self._liha_geometry(), site, column, row, tips, slow_x)
    await self._liha_require_tips(use_channels, mounted=False)
    requested = liha.tip_mask(tips)
    replies = await self._liha_execute(plan, use_channels, requested)
    mounted = replies[-1]["data"][0]
    missing = [c for c in use_channels if not mounted >> c & 1]
    if missing:
      raise ChannelizedError(
        {c: NoTipError(f"LiHa channel {c} did not get a DiTi (RTS {mounted:08b})") for c in missing}
      )
    for op, channel in zip(ops, use_channels):
      self._liha_contents()[channel] = None
      self._liha_ditis()[channel] = diti
      self._liha_conductive()[channel] = getattr(op.tip, "is_conductive", None)

  def _liha_conductive(self) -> Dict[int, Optional[bool]]:
    """Per channel: whether the mounted DiTi is conductive (``Tip.is_conductive``; None = unknown),
    recorded by ``pick_up_tips``."""
    state: Optional[Dict[int, Optional[bool]]] = getattr(self, "_liha_conductive_state", None)
    if state is None:
      state = {}
      self._liha_conductive_state = state
    return state

  def _liha_require_conductive(self, use_channels: List[int], what: str):
    """Refuse a step with liquid level detection on channels whose DiTis are known to be
    non-conductive: the LiHa detects capacitively, so the tips would go down to z_max and fail."""
    state = self._liha_conductive()
    bad = [c for c in use_channels if state.get(c) is False]
    if bad:
      raise ValueError(
        f"LiHa {what} detects the liquid, which needs conductive DiTis; channels {bad} carry"
        " non-conductive ones"
      )

  def _liha_ditis(self) -> Dict[int, str]:
    """EVOware DiTi labware mounted per channel (recorded by ``pick_up_tips``)."""
    ditis: Optional[Dict[int, str]] = getattr(self, "_liha_ditis_state", None)
    if ditis is None:
      ditis = {}
      self._liha_ditis_state = ditis
    return ditis

  async def drop_tips(self, ops: List[Drop], use_channels: List[int], slow_x: bool = True):
    """EVOware "Drop DiTis" into the DiTi waste chute, or "Set DiTis Back" into a DiTi rack.

    Into a ``Trash``: it must be the chute on the deck, a ``Washstation_2Grid_DiTi_Waste`` on a
    ``DiTi_Waste_3_Trough_Wash`` carrier at grid 1, PLR site 0 (``liha.DITI_WASTE``). EVOware's
    position for it is fixed, so offsets (e.g. those ``discard_tips`` spreads over the trash) are
    ignored.

    Into tip spots (e.g. ``LiquidHandler.return_tips``): ``liha.plan_set_ditis_back``, consecutive
    rows of one column of a rack of the mounted DiTi type, channel order = row order. The channels
    must hold only air (dispense first; DiTis with liquid or unknown contents go to the chute).
    ``slow_x`` as in ``pick_up_tips``. Pick them up again with ``pick_up_tips``.

    Raises ChannelizedError (HasTipError) for channels that still have their DiTi afterwards.
    """
    self._liha_mixed = None
    if all(isinstance(op.resource, TipSpot) for op in ops):
      site, column, row, tips = self._liha_target(ops, use_channels)
      diti = self._liha_diti(ops)
      if diti != site.labware:
        raise ValueError(f"tip {diti!r} does not belong to rack {site.labware!r}")
      contents = self._liha_contents()
      held = [c for c in use_channels if contents.get(c) is not None]
      if held:
        raise RuntimeError(
          f"LiHa channels {held} hold liquid or unknown contents; dispense first, or drop the DiTis"
          " into the chute"
        )
      plan = liha.plan_set_ditis_back(self._liha_geometry(), site, column, row, tips, slow_x)
    elif all(isinstance(op.resource, Trash) for op in ops):
      for trash in {op.resource for op in ops}:
        site = self._liha_site(trash)
        if site != liha.DITI_WASTE:
          raise ValueError(f"{trash.name} is at {site}, not the LiHa DiTi waste {liha.DITI_WASTE}")
      tips = self._liha_channels(use_channels)
      if len(ops) != len(use_channels):
        raise ValueError("one operation per channel")
      plan = liha.plan_drop_ditis(self._liha_geometry(), tips)
    else:
      raise NotImplementedError(
        "LiHa DiTis go either into the DiTi waste (a Trash) or back into a DiTi rack (tip spots)"
      )
    await self._liha_require_tips(use_channels, mounted=True)
    replies = await self._liha_execute(plan, use_channels)
    after = next(r for (c, _), r in zip(reversed(plan.commands), reversed(replies)) if c == "RTS")
    still = [c for c in use_channels if after["data"][0] >> c & 1]
    if still:
      raise ChannelizedError(
        {c: HasTipError(f"LiHa channel {c} still has its DiTi (RTS {after['data'][0]:08b})") for c in still}
      )
    for channel in use_channels:
      self._liha_contents()[channel] = None
      self._liha_ditis().pop(channel, None)
      self._liha_conductive().pop(channel, None)

  async def aspirate(
    self,
    ops: List[SingleChannelAspiration],
    use_channels: List[int],
    liquid_class: Optional[str] = None,
    trough_column: int = 1,
  ):
    """EVOware "Aspirate" with liquid level detection, single mode, the same volume on all channels.

    Args:
      liquid_class: EVOware LiHa liquid class (default "Water free dispense").
      trough_column: column EVOware addresses on a one-well labware (see ``_liha_target``).
    The channels must hold only air (dispense first). The liquid class's air gaps are added.
    From a trough pass its well once per channel with ``spread="custom"``: EVOware positions the
    tips itself, and PLR cannot spread 8 channels over one 80 mm well.
    Right after ``liha_mix`` in the same wells with the same channels, the tips aspirate where they
    are and take no air gap (the mix's is still in the tips), as EVOware does.
    """
    mixed, self._liha_mixed = self._liha_mixed, None
    volume = self._liha_check_ops(ops)
    site, column, row, tips = self._liha_target(ops, use_channels, trough_column)
    contents = self._liha_contents()
    full = [c for c in use_channels if contents.get(c) is not None]
    if full:
      raise RuntimeError(f"LiHa channels {full} already hold liquid or unknown contents; dispense first")
    lc = self._liha_lc(liquid_class, volume)
    self._liha_require_conductive(use_channels, "aspirate")
    geo = self._liha_geometry()
    diti = self._liha_diti(ops)
    await self._liha_require_tips(use_channels, mounted=True)
    plunger = await self._liha_report("RPP", 0)
    after_mix_z = None
    if mixed is not None and mixed[:4] == (site, column, row, tips):
      after_mix_z = await self._liha_still_at(geo.tips(site, column, row, tips[0]), tips, mixed[4])
    plan = liha.plan_aspirate(
      geo, lc, site, column, row, tips, volume, diti, plunger, after_mix_z=after_mix_z
    )
    for channel in use_channels:
      contents[channel] = UNKNOWN
    await self._liha_execute(plan.search, use_channels)
    detected = await self._liha_report("RPZ", 0)
    await self._liha_execute(plan.finish(detected), use_channels)
    for channel in use_channels:
      contents[channel] = (volume, lc.name)

  async def dispense(
    self, ops: List[SingleChannelDispense], use_channels: List[int], trough_column: int = 1
  ):
    """EVOware "Dispense", single mode: free or wet contact as the aspiration's liquid class says.

    The channels may go over every k-th row of one column (e.g. channels 0-3 over rows B, D, F, H;
    EVOware's checker pattern), with the tip spacing k times the row pitch.

    Dispenses everything the channels aspirated (EVOware dispenses all but the system trailing air
    gap), so ``volume`` must equal the aspirated volume.
    """
    self._liha_mixed = None
    volume = self._liha_check_ops(ops)
    site, column, row, tips = self._liha_target(ops, use_channels, trough_column, row_stride=True)
    stride = self._liha_last_stride
    contents = self._liha_contents()
    held = {contents.get(c) for c in use_channels}
    if len(held) != 1 or None in held or UNKNOWN in held:
      raise RuntimeError(f"LiHa channels {use_channels} do not hold one known aspiration: {held}")
    aspirated, name = cast(Tuple[float, str], held.pop())
    if not math.isclose(volume, aspirated, abs_tol=1e-6):
      raise NotImplementedError(
        f"LiHa dispenses the whole aspiration ({aspirated} uL); partial dispense is not supported"
      )
    lc = self._liha_lc(name, volume)
    if lc.dispense_lld:
      self._liha_require_conductive(use_channels, f"dispense ({lc.name}, wet contact)")
    geo = self._liha_geometry()
    diti = self._liha_diti(ops)
    await self._liha_require_tips(use_channels, mounted=True)
    plunger = await self._liha_report("RPP", 0)
    plan = liha.plan_dispense(geo, lc, site, column, row, tips, diti, plunger, stride)
    for channel in use_channels:
      contents[channel] = UNKNOWN
    await self._liha_execute(plan.first, use_channels)
    if plan.finish_args is not None:
      detected = await self._liha_report("RPZ", 0)
      await self._liha_execute(plan.finish(detected), use_channels)
    for channel in use_channels:
      contents[channel] = None

  async def liha_mix(
    self,
    containers: Sequence[Resource],
    volume: float,
    cycles: int,
    use_channels: List[int],
    liquid_class: str,
    trough_column: int = 1,
  ):
    """EVOware "Mix", single mode: ``cycles`` times aspirate and dispense ``volume`` uL in each of
    ``containers`` (one per channel: consecutive rows of one column, or a trough's single well).

    ``liquid_class`` must be a mix class: one that mixes at the labware's z_max after detecting the
    liquid (Aspirate/LLD position 0), such as the instrument's ">> Water Mix Gentle <<\t11"
    (see ``liha.plan_mix``; the volume is not calibrated). The channels must hold only air. The
    class's leading air gap stays in the tips; an ``aspirate`` right after, in the same wells with
    the same channels, follows on from it.
    Raises ChannelizedError (TecanLihaNoLiquidError) for channels that found no liquid.
    """
    self._liha_mixed = None
    if not (math.isfinite(volume) and volume > 0):
      raise ValueError(f"volume must be positive, got {volume}")
    if isinstance(cycles, bool) or not isinstance(cycles, int) or cycles < 1:
      raise ValueError(f"cycles must be a positive integer, got {cycles!r}")
    targets = [_LihaTarget(c) for c in containers]
    site, column, row, tips = self._liha_target(targets, use_channels, trough_column)
    contents = self._liha_contents()
    full = [c for c in use_channels if contents.get(c) is not None]
    if full:
      raise RuntimeError(
        f"LiHa channels {full} already hold liquid or unknown contents; dispense first"
      )
    ditis = {self._liha_ditis().get(c) for c in use_channels}
    if len(ditis) != 1 or None in ditis:
      raise RuntimeError(f"LiHa channels {use_channels} need DiTis picked up by pick_up_tips")
    diti = cast(str, ditis.pop())
    lc = self._liha_lc(liquid_class, volume)
    self._liha_require_conductive(use_channels, "mix")
    geo = self._liha_geometry()
    # refuse an unsuitable class before anything is sent (the plunger only changes the air gap)
    liha.plan_mix(geo, lc, site, column, row, tips, volume, cycles, diti, [0] * self.num_channels)
    await self._liha_require_tips(use_channels, mounted=True)
    plunger = await self._liha_report("RPP", 0)
    plan = liha.plan_mix(geo, lc, site, column, row, tips, volume, cycles, diti, plunger)
    for channel in use_channels:
      contents[channel] = UNKNOWN
    await self._liha_execute(plan.search, use_channels)
    detected = await self._liha_report("RPZ", 0)
    await self._liha_execute(plan.finish(detected), use_channels)
    for channel in use_channels:
      contents[channel] = None
    self._liha_mixed = (site, column, row, tips, plan.retract_z)

  async def liha_detect_liquid(
    self,
    containers: Sequence[Resource],
    use_channels: List[int],
    liquid_class: Optional[str] = None,
    trough_column: int = 1,
  ) -> Dict[int, int]:
    """Detect the liquid surface in ``containers`` (one per channel, as for ``liha_mix``) and
    return the detected Z per channel (firmware units, 1/10 mm; lower = deeper).

    Not EVOware's "Detect Liquid" (never logged here): ``liha.plan_detect_liquid``, the verified
    detection of an aspirate without air gap, plunger or submerge, then straight up to z_travel.
    ``liquid_class`` (default "Water free dispense") gives the detection settings. The channels
    must carry conductive DiTis holding only air. Raises ChannelizedError
    (TecanLihaNoLiquidError) for channels that found no liquid; the tips are then raised.
    """
    self._liha_mixed = None
    targets = [_LihaTarget(c) for c in containers]
    site, column, row, tips = self._liha_target(targets, use_channels, trough_column)
    contents = self._liha_contents()
    full = [c for c in use_channels if contents.get(c) is not None]
    if full:
      raise RuntimeError(f"LiHa channels {full} hold liquid or unknown contents; dispense first")
    ditis = {self._liha_ditis().get(c) for c in use_channels}
    if len(ditis) != 1 or None in ditis:
      raise RuntimeError(f"LiHa channels {use_channels} need DiTis picked up by pick_up_tips")
    lc = self._liha_lc(liquid_class, 100)
    self._liha_require_conductive(use_channels, "liquid detection")
    search, retract = liha.plan_detect_liquid(
      self._liha_geometry(), lc, site, column, row, tips, cast(str, ditis.pop())
    )
    await self._liha_require_tips(use_channels, mounted=True)
    await self._liha_execute(search, use_channels)
    detected = await self._liha_report("RPZ", 0)
    await self._liha_execute(retract, use_channels)
    return {c: detected[c] for c in use_channels}

  async def _liha_still_at(self, pos: "liha.Position", tips: List[int], z: int) -> Optional[int]:
    """``z`` if the arm is still where ``liha_mix`` left it (X, Y, spacing and the tips' Z, read
    with RPX0/RPY0/RPZ0), else None. Another arm's move may have pushed the LiHa aside."""
    fw = self._liha_firmware()
    x = (await fw.execute_wire("RPX", [0]))["data"]
    y = (await fw.execute_wire("RPY", [0]))["data"]
    zs = await self._liha_report("RPZ", 0)
    here = list(x[:1]) == [pos.x] and list(y[:2]) == [pos.y, pos.y_spacing]
    if here and all(zs[t - 1] == z for t in tips):
      return z
    return None

  # ============== LiHa wash ==============
  #
  # Command sequences come from ``liha.py`` (reproducing EVOware run logs). The
  # XY move is preceded by ``x_coordinator().clear_path("liha", x)``.

  def _liha_geometry(self) -> "liha.LihaGeometry":
    config = getattr(self, "evoware_config", None)
    if config is None:
      raise RuntimeError("LiHa geometry needs the EVOware configuration: call load_evoware_config()")
    if getattr(self, "_z_range", None) is None:
      raise RuntimeError("LiHa Z range unknown, forgot to call `setup`?")
    deck = getattr(self, "_deck", None)
    if deck is not None:  # labware carrying its own LiHa correction (e.g. DiTi_1000ul_LiHa_box)
      for resource in deck.get_all_resources():
        if getattr(resource, "liha_offset", None) is not None:
          self._liha_site(resource)
    return liha.LihaGeometry(
      config, self._z_range, self.liha_site_offsets, self.liha_site_z_offsets
    )

  async def _liha_send(self, command: str, params=None):
    # EVOware re-sends S-commands (SEP, SPP, SHZ, SSM) every time; bypass the set-command cache.
    self._cache.pop(EVOBackend.LIHA + command, None)
    return await self.send_command(EVOBackend.LIHA, command=command, params=params)

  async def liha_mounted_tips(self) -> int:
    """Mounted disposable tips as a channel bit mask (RTS; 0 = none, bit 0 = channel 1)."""
    data = (await self._liha_send("RTS"))["data"]
    if not data or type(data[0]) is not int:
      raise TecanError(f"unexpected LiHa RTS response {data!r}", EVOBackend.LIHA, 0)
    return data[0]

  async def liha_wash_tips(
    self,
    waste: "liha.LihaSite" = liha.WASH_WASTE,
    waste_volume_ml: float = 12.0,
    waste_delay_ms: int = 50,
    air_gap_ul: float = 10,
    air_gap_speed: float = 70,
    use_channels: Optional[List[int]] = None,
  ):
    """Flush the bare DiTi adapters with system liquid over the wash station waste,
    as EVOware's "Wash Tips" (``flushLiHa.esc``): ``waste_volume_ml`` per tip in
    full dilutor strokes, then a system trailing air gap of ``air_gap_ul``.

    Refused while disposable tips are mounted (RTS not 0): the system liquid would
    go into the tips. Defaults are those of ``flushLiHa.esc`` on this instrument.
    """
    if not self.liha_connected:
      raise RuntimeError("no LiHa on this instrument")
    self._liha_mixed = None
    channels = list(range(self.num_channels)) if use_channels is None else use_channels
    plan = liha.plan_wash(
      self._liha_geometry(), waste, waste_volume_ml, waste_delay_ms, air_gap_ul, air_gap_speed, channels
    )
    mounted = await self.liha_mounted_tips()
    if mounted:
      raise RuntimeError(f"LiHa has disposable tips mounted (RTS {mounted}); drop them before washing")
    await self._liha_run(plan)

  # ============== MCA 96-channel head ==============
  #
  # Command sequences come from ``mca.py`` (reproducing EVOware captures). Every
  # XY move is preceded by ``x_coordinator().clear_path("mca", x)`` so LiHa and
  # RoMa are pushed aside first, and Z is at travel height before any XY move.
  # The backend tracks the plunger; on any error its state becomes unknown and
  # the next liquid operation is refused until tips are picked up again.

  def load_evoware_config(self, database_dir):
    """Load the instrument's EVOware ``database`` folder: Carrier.cfg and
    evoware.inf (MCA positions, computed exactly as EVOware does) and the liquid
    classes DefaultLCs.XML / CustomLCs.XML."""
    from pylabrobot.legacy.liquid_handling.backends.tecan import evoware_config
    from pylabrobot.legacy.liquid_handling.backends.tecan.evoware_liquid_classes import (
      LiquidClassLibrary,
    )

    self.evoware_config = evoware_config.load(database_dir)
    self._mca_geometry_obj = mca.McaGeometry(self.evoware_config)
    self.liquid_classes = LiquidClassLibrary.from_database(database_dir)

  @property
  def mca_geometry(self) -> "mca.McaGeometry":
    geo = getattr(self, "_mca_geometry_obj", None)
    if geo is None:
      raise RuntimeError("MCA geometry needs the EVOware configuration: call load_evoware_config()")
    return geo

  def _mca_liquid_class(self, tip_model: str, volume: float, name: Optional[str]):
    library = getattr(self, "liquid_classes", None)
    if library is None:
      raise RuntimeError("MCA liquid classes need the EVOware configuration: call load_evoware_config()")
    return library.mca(name or mca.DEFAULT_CLASS[mca.tip_rack_name(tip_model)], volume)

  def _mca_plunger(self) -> "mca.PlungerState":
    if getattr(self, "_mca_plunger_state", None) is None:
      self._mca_plunger_state = mca.PlungerState()
    return self._mca_plunger_state

  async def _mca_send(self, command: str, params=None):
    # EVOware re-sends S-commands (SLP, SSM, SPA) every time; bypass the set-command cache.
    self._cache.pop(EVOBackend.MCA + command, None)
    return await self.send_command(EVOBackend.MCA, command=command, params=params)

  async def mca_position(self) -> Tuple[int, int, int, int]:
    """Current MCA X, Y, Z, G (firmware units, RPA 0)."""
    data = (await self._mca_send("RPA", [0]))["data"]
    if len(data) < 4 or any(type(v) is not int for v in data[:4]):
      raise TecanError(f"unexpected MCA RPA response {data!r}", EVOBackend.MCA, 0)
    return (data[0], data[1], data[2], data[3])

  async def _mca_execute(self, plan: "mca.Plan"):
    """Send a plan in order; verify plunger checks; clear X before XY moves."""
    plunger = self._mca_plunger()
    try:
      for index, (command, params) in enumerate(plan.commands):
        if command == "WAIT":
          await asyncio.sleep(params[0] / 1000)
          continue
        if command == "PAA" and params and params[0] is not None:
          await self.x_coordinator().clear_path("mca", int(params[0]))
        resp = await self._mca_send(command, params)
        if index in plan.checks:
          got = (resp or {}).get("data", [None])[0]
          if type(got) is not int or abs(got - plan.checks[index]) > mca.PLUNGER_TOLERANCE:
            raise TecanError(
              f"MCA plunger at {got}, expected {plan.checks[index]}; stopping", EVOBackend.MCA, 0
            )
    except BaseException:
      plunger.reset(None)
      raise

  async def _mca_z_speed(self) -> int:
    return int((await self._mca_send("RPA", [2]))["data"][2])

  @staticmethod
  def _mca_tip_model(tips) -> str:
    models = {getattr(t, "model", None) for t in tips if t is not None}
    if len(models) != 1 or None in models:
      raise ValueError(f"MCA needs one known tip type on all channels, got {models}")
    return models.pop()

  def _mca_head_tip_model(self) -> str:
    head = self.head96 or {}
    return self._mca_tip_model([t.get_tip() for t in head.values() if t.has_tip])

  @staticmethod
  def _mix_spec(mix) -> Optional["mca.MixSpec"]:
    if mix is None:
      return None
    return mca.MixSpec(mix.volume, mix.repetitions, mix.flow_rate)

  async def pick_up_tips96(self, pickup: PickupTipRack):
    here = await self.mca_position()
    plan = mca.plan_pick_up(self.mca_geometry, pickup.resource, pickup.offset, here[:3], self._mca_plunger())
    try:
      await self._mca_execute(plan)
    except TecanTipsNotFetchedError:
      # ATF leaves the head at Zend; raise it before reporting (as EVOware does)
      await self._mca_send("PAA", [None, None, mca.Z_TRAVEL])
      raise

  async def drop_tips96(self, drop: DropTipRack, allow_liquid: bool = False):
    """Return tips into a tip rack. Refused while the tips hold liquid (ATD would
    push it into the rack) unless ``allow_liquid``."""
    if not isinstance(drop.resource, TipRack):
      raise NotImplementedError("MCA tip drop is supported into a tip rack (return) only")
    here = await self.mca_position()
    plan = mca.plan_drop(self.mca_geometry, drop.resource, drop.offset, here[:3], self._mca_plunger(),
                         allow_liquid=allow_liquid)
    await self._mca_execute(plan)

  @staticmethod
  def _mca_liquid_resource(op) -> Resource:
    if isinstance(op, (MultiHeadAspirationPlate, MultiHeadDispensePlate)):
      return op.wells[0].parent
    return op.container

  async def aspirate96(
    self,
    aspiration: Union[MultiHeadAspirationPlate, MultiHeadAspirationContainer],
    liquid_class: Optional[str] = None,
    multi_dispense: bool = False,
  ):
    """Aspirate with all 96 channels using an EVOware liquid class.

    Args:
      liquid_class: EVOware liquid class name (default: "Water free dispense" for
        200 uL MCA96 tips, "Water wet contact" for 50 uL).
      multi_dispense: use the class's multi-pipetting parameters (excess volume) so
        the liquid can be dispensed in several portions; empty the excess with
        ``mca_empty_tips``.
    ``aspiration.mix`` mixes in the source before aspirating (EVOware "mix before aspirate").
    Further aspirations into tips that already hold liquid are allowed.
    """
    tip = self._mca_tip_model(aspiration.tips)
    lc = self._mca_liquid_class(tip, aspiration.volume, liquid_class)
    target = mca.LiquidTarget(
      self.mca_geometry, self._mca_liquid_resource(aspiration), aspiration.offset, tip,
      aspiration.liquid_height,
    )
    here = await self.mca_position()
    z_speed = await self._mca_z_speed()
    plan = mca.plan_aspirate(
      target, aspiration.volume, lc, here[:3], self._mca_plunger(),
      flow_rate=aspiration.flow_rate, blow_out_air_volume=aspiration.blow_out_air_volume,
      current_z_speed=z_speed, mix=self._mix_spec(aspiration.mix), multi=multi_dispense,
    )
    await self._mca_execute(plan)

  async def dispense96(self, dispense: Union[MultiHeadDispensePlate, MultiHeadDispenseContainer]):
    """Dispense with all 96 channels using the liquid class of the aspiration.

    Partial volumes are allowed. Dispensing everything in single mode empties the
    tips completely, as EVOware does. ``dispense.mix`` mixes after a full dispense.
    """
    tip = self._mca_tip_model(dispense.tips)
    target = mca.LiquidTarget(
      self.mca_geometry, self._mca_liquid_resource(dispense), dispense.offset, tip,
      dispense.liquid_height,
    )
    here = await self.mca_position()
    z_speed = await self._mca_z_speed()
    plan = mca.plan_dispense(
      target, dispense.volume, here[:3], self._mca_plunger(),
      flow_rate=dispense.flow_rate, current_z_speed=z_speed, mix=self._mix_spec(dispense.mix),
    )
    await self._mca_execute(plan)

  async def mca_empty_tips(self, resource: Resource, offset: Coordinate = Coordinate.zero()):
    """Expel everything left in the tips (e.g. multi-pipetting excess) into ``resource``."""
    tip = self._mca_head_tip_model()
    target = mca.LiquidTarget(self.mca_geometry, resource, offset, tip)
    here = await self.mca_position()
    await self._mca_execute(mca.plan_empty(target, here[:3], self._mca_plunger()))

  async def mix96(
    self,
    resource: Resource,
    volume: float,
    repetitions: int,
    flow_rate: Optional[float] = None,
    offset: Coordinate = Coordinate.zero(),
    liquid_height: Optional[float] = None,
    liquid_class: Optional[str] = None,
  ):
    """EVOware MCAMix: air gap, immerse, ``repetitions`` cycles of ``volume`` uL
    without leaving the liquid, then raise to travel height.

    ``resource`` is a plate or trough. ``flow_rate`` (uL/s) sets both strokes;
    None keeps the liquid class speeds (EVOware: 50 uL/s in, 400 uL/s out).
    """
    tip = self._mca_head_tip_model()
    lc = self._mca_liquid_class(tip, volume, liquid_class)
    target = mca.LiquidTarget(self.mca_geometry, resource, offset, tip, liquid_height)
    here = await self.mca_position()
    plan = mca.plan_mix(
      target, volume, repetitions, lc, here[:3], self._mca_plunger(), flow_rate=flow_rate
    )
    await self._mca_execute(plan)

  # MCA96 plate gripper (optional hardware; NOT tested: this instrument has none).
  async def mca_pick_up_plate(self, resource: Resource, z_grip: int, grip_width: int, open_width: int):
    """Grip ``resource`` with the MCA96 gripper (firmware units: Z, G widths in 1/10 mm).
    Fails with TecanError 9 "Plate not fetched" if no plate is gripped."""
    here = await self.mca_position()
    await self._mca_execute(mca.plan_grip(self.mca_geometry, resource, here[:3], z_grip, grip_width, open_width))

  async def mca_drop_plate(self, destination: Resource, z_release: int, open_width: int):
    """Put a gripped plate down over ``destination`` and open the gripper."""
    here = await self.mca_position()
    await self._mca_execute(mca.plan_release(self.mca_geometry, destination, here[:3], z_release, open_width))

  async def mca_initialize_head_drive(self):
    """Initialize the MCA plunger (EVOware sequence: G open, PID, ASH0 x2, PPA ready).

    Needed once after power-on: PLR ``setup()`` sends ``PIA``, which does not
    initialize the plunger. Requires the head at travel height and no tips.
    """
    here = await self.mca_position()
    if here[2] != mca.Z_TRAVEL:
      raise RuntimeError(f"MCA must be at travel height Z {mca.Z_TRAVEL} for head drive init (Z={here[2]})")
    if self.head96 and any(t.has_tip for t in self.head96.values()):
      raise RuntimeError("remove MCA tips before initializing the head drive")
    await self._mca_execute(mca.plan_initialize_head_drive(self._mca_plunger()))

  async def mca_move_xyz(self, x: Optional[int] = None, y: Optional[int] = None, z: Optional[int] = None):
    """Move the MCA in firmware units. XY moves happen at Z travel height."""
    here = await self.mca_position()
    plan = mca.Plan()
    if x is not None or y is not None:
      mca.plan_move_xy(plan, here[0] if x is None else x, here[1] if y is None else y, here[:3])
    if z is not None:
      plan.add("PAA", None, None, z)
    await self._mca_execute(plan)

  async def mca_move_to(
    self, resource: Resource, offset: Coordinate = Coordinate.zero(), z: Optional[int] = None
  ):
    """Move the MCA (channel A1) over ``resource``; Z stays at travel height unless given."""
    x, y = self.mca_geometry.xy(resource, offset)
    await self.mca_move_xyz(x, y, z)

  async def pick_up_resource(self, pickup: ResourcePickup):
    # TODO: implement PnP for moving tubes
    assert self.roma_connected

    z_range = await self.roma.report_z_param(5)
    x, y, z = self._roma_positions(
      pickup.resource, pickup.resource.get_location_wrt(self.deck), z_range
    )
    h = int(pickup.resource.get_absolute_size_y() * 10)

    # move to resource
    await self.roma.set_smooth_move_x(1)
    await self.roma.set_fast_speed_x(10000)
    await self.roma.set_fast_speed_y(5000, 1500)
    await self.roma.set_fast_speed_z(1300)
    await self.roma.set_fast_speed_r(5000, 1500)
    await self.roma.set_vector_coordinate_position(1, x, y, z["safe"], 900, None, 1, 0)
    await self.roma.action_move_vector_coordinate_position()
    await self.roma.set_smooth_move_x(0)

    # pick up resource
    await self.roma.position_absolute_g(900)  # TODO: verify
    await self.roma.set_target_window_class(1, 0, 0, 0, 135, 0)
    await self.roma.set_vector_coordinate_position(1, x, y, z["travel"], 900, None, 1, 1)
    # TODO verify z param
    await self.roma.set_vector_coordinate_position(1, x, y, z["end"], 900, None, 1, 0)
    await self.roma.action_move_vector_coordinate_position()
    await self.roma.set_fast_speed_y(3500, 1000)
    await self.roma.set_fast_speed_r(2000, 600)
    await self.roma.set_gripper_params(100, 75)
    await self.roma.grip_plate(h - 100)

  async def move_picked_up_resource(self, move: ResourceMove):
    raise NotImplementedError()

  async def drop_resource(self, drop: ResourceDrop):
    """Drop a resource like a plate or a lid using the integrated robotic arm."""

    z_range = await self.roma.report_z_param(5)
    x, y, z = self._roma_positions(
      drop.resource, drop.resource.get_location_wrt(self.deck), z_range
    )
    xt, yt, zt = self._roma_positions(drop.resource, drop.destination, z_range)

    # move to target
    await self.roma.set_target_window_class(1, 0, 0, 0, 135, 0)
    await self.roma.set_target_window_class(2, 0, 0, 0, 53, 0)
    await self.roma.set_target_window_class(3, 0, 0, 0, 55, 0)
    await self.roma.set_target_window_class(4, 45, 0, 0, 0, 0)
    await self.roma.set_vector_coordinate_position(1, x, y, z["end"], 900, None, 1, 1)
    await self.roma.set_vector_coordinate_position(2, x, y, z["travel"], 900, None, 1, 2)
    await self.roma.set_vector_coordinate_position(3, x, y, z["safe"], 900, None, 1, 3)
    await self.roma.set_vector_coordinate_position(4, xt, yt, zt["safe"], 900, None, 1, 4)
    await self.roma.set_vector_coordinate_position(5, xt, yt, zt["travel"], 900, None, 1, 3)
    await self.roma.set_vector_coordinate_position(6, xt, yt, zt["end"], 900, None, 1, 0)
    await self.roma.action_move_vector_coordinate_position()

    # release resource
    await self.roma.position_absolute_g(900)
    await self.roma.set_fast_speed_y(5000, 1500)
    await self.roma.set_fast_speed_r(5000, 1500)
    await self.roma.set_vector_coordinate_position(1, xt, yt, zt["end"], 900, None, 1, 1)
    await self.roma.set_vector_coordinate_position(2, xt, yt, zt["travel"], 900, None, 1, 2)
    await self.roma.set_vector_coordinate_position(3, xt, yt, zt["safe"], 900, None, 1, 0)
    await self.roma.action_move_vector_coordinate_position()
    await self.roma.set_fast_speed_y(3500, 1000)
    await self.roma.set_fast_speed_r(2000, 600)

  def can_pick_up_tip(self, channel_idx: int, tip: Tip) -> bool:
    if not isinstance(tip, TecanTip):
      return False
    return True

  def _first_valid(self, lst: List[Optional[T]]) -> Tuple[Optional[T], int]:
    """Returns first item in list that is not None"""

    for i, v in enumerate(lst):
      if v is not None:
        return v, i
    return None, -1

  def _bin_use_channels(self, use_channels: List[int]) -> int:
    """Converts use_channels to a binary coded tip representation."""

    b = 0
    for channel in use_channels:
      b += 1 << channel
    return b

  def _roma_positions(
    self, resource: Resource, offset: Coordinate, z_range: int
  ) -> Tuple[int, int, Dict[str, int]]:
    """Creates x, y, and z positions used by RoMa ops."""

    parent = resource.parent  # PlateHolder
    if parent is None:
      raise ValueError(f"Operation is not supported by resource {resource}.")
    parent = parent.parent  # PlateCarrier
    # TODO: this is probably the current plate carrier, not the destination.
    # Also, we should just support any coordinate as the destination.
    if not isinstance(parent, TecanPlateCarrier):
      raise ValueError(f"Operation is not supported by resource {parent}.")

    if (
      parent.roma_x is None
      or parent.roma_y is None
      or parent.roma_z_safe is None
      or parent.roma_z_end is None
    ):
      raise ValueError(f"Operation is not supported by resource {parent}.")
    x_position = int((offset.x - 100) * 10 + parent.roma_x)
    y_position = int((347.1 - (offset.y + resource.get_absolute_size_y())) * 10 + parent.roma_y)
    z_positions = {
      "safe": z_range - int(parent.roma_z_safe),
      "travel": int(self._z_roma_traversal_height * 10),
      "end": z_range - int(parent.roma_z_end - offset.z * 10),
    }

    return x_position, y_position, z_positions


class EVOArm:
  """
  Provides firmware commands for EVO arms. Caches arm positions.
  """

  _pos_cache: Dict[str, int] = {}

  def __init__(self, backend: EVOBackend, module: str):
    self.backend = backend
    self.module = module

  async def position_initialization_x(self):
    """Reinitializes X-axis of the arm."""

    await self.backend.send_command(module=self.module, command="PIX")

  async def report_x_param(self, param: int) -> int:
    """Report current parameter for x-axis.

    Args:
      param: 0 - current position, 5 - actual machine range
    """

    resp: List[int] = (
      await self.backend.send_command(module=self.module, command="RPX", params=[param])
    )["data"]
    return resp[0]

  async def report_y_param(self, param: int) -> List[int]:
    """Report current parameters for y-axis.

    Args:
      param: 0 - current position, 5 - actual machine range
    """

    resp: List[int] = (
      await self.backend.send_command(module=self.module, command="RPY", params=[param])
    )["data"]
    return resp


class LiHa(EVOArm):
  """
  Provides firmware commands for the LiHa
  """

  async def initialize_plunger(self, tips):
    """Initializes plunger and valve drive

    Args:
      tips: binary coded tip select
    """
    await self.backend.send_command(module=self.module, command="PID", params=[tips])

  async def report_z_param(self, param: int) -> List[int]:
    """Report current parameters for z-axis.

    Param:
    0   Report current position in 1/10 mm.
    1   Report acceleration in 1/10 mm/s².
    2   Report fast speed in 1/10 mm/s.
    3   Report initialization speed in 1/10 mm.
    4   Report initialization offset in 1/10 mm.
    5   Report actual machine range in 1/10 mm.
    6   Report deviation in encoder increments.
    7   Report every time 0.
    8   Report scale adjust factor.
    9   Report slow speed in 1/10 mm/s.
    10  Report axis scale factor.
    11  Report target position in 1/10 mm.
    12  Report travel position in 1/10 mm.
    """

    resp: List[int] = (
      await self.backend.send_command(module=self.module, command="RPZ", params=[param])
    )["data"]
    return resp

  async def report_number_tips(self) -> int:
    """Report number of tips on arm."""

    resp: List[int] = (
      await self.backend.send_command(module=self.module, command="RNT", params=[1])
    )["data"]
    return resp[0]

  async def position_absolute_all_axis(self, x: int, y: int, ys: int, z: List[int]):
    """Position absolute for all LiHa axes.

    Args:
      x: absolute x position in 1/10 mm, must be in allowed machine range
      y: absolute y position in 1/10 mm, must be in allowed machine range
      ys: absolute y spacing in 1/10 mm, must be between 90 and 380
      z: absolute z position in 1/10 mm for each channel, must be in
         allowed machine range

    Raises:
      TecanError: if moving to the target position causes a collision
    """

    cur_x = EVOArm._pos_cache.setdefault(self.module, await self.report_x_param(0))
    for module, pos in EVOArm._pos_cache.items():
      if module == self.module:
        continue
      if cur_x < x and cur_x < pos < x:  # moving right
        raise TecanError("Invalid command (collision)", self.module, 2)
      if cur_x > x and cur_x > pos > x:
        raise TecanError("Invalid command (collision)", self.module, 2)
      if abs(pos - x) < 1500:
        raise TecanError("Invalid command (collision)", self.module, 2)

    await self.backend.send_command(module=self.module, command="PAA", params=list([x, y, ys] + z))

    EVOArm._pos_cache[self.module] = x

  async def position_valve_logical(self, param: List[Optional[int]]):
    """Position valve logical for each channel.

    Args:
      param: 0 - outlet, 1 - inlet, 2 - bypass
    """

    await self.backend.send_command(module=self.module, command="PVL", params=param)

  async def set_end_speed_plunger(self, speed: List[Optional[int]]):
    """Set end speed for plungers.

    Args:
      speed: speed for each plunger in half step per second, must be between
             5 and 6000
    """

    await self.backend.send_command(module=self.module, command="SEP", params=speed)

  async def move_plunger_relative(self, rel: List[Optional[int]]):
    """Move plunger relative upwards (dispense) or downards (aspirate).

    Args:
      rel: relative position for each plunger in full steps, must be between
           -3150 and 3150
    """

    await self.backend.send_command(module=self.module, command="PPR", params=rel)

  async def set_detection_mode(self, proc: int, sense: int):
    """Set liquid detection mode.

    Args:
      proc: detection procedure (7 for double detection sequential with
            retract and extra submerge)
      sense: conductivity (1 for high)
    """

    await self.backend.send_command(module=self.module, command="SDM", params=[proc, sense])

  async def set_search_speed(self, speed: List[Optional[int]]):
    """Set search speed for liquid search commands.

    Args:
      speed: speed for each channel in 1/10 mm/s, must be between 1 and 1500
    """

    await self.backend.send_command(module=self.module, command="SSL", params=speed)

  async def set_search_retract_distance(self, dist: List[Optional[int]]):
    """Set z-axis retract distance for liquid search commands.

    Args:
      dist: retract distance for each channel in 1/10 mm, must be in allowed
            machine range
    """

    await self.backend.send_command(module=self.module, command="SDL", params=dist)

  async def set_search_submerge(self, dist: List[Optional[int]]):
    """Set submerge for liquid search commands.

    Args:
      dist: submerge distance for each channel in 1/10 mm, must be between
            -1000 and max z range
    """

    await self.backend.send_command(module=self.module, command="SBL", params=dist)

  async def set_search_z_start(self, z: List[Optional[int]]):
    """Set z-start for liquid search commands.

    Args:
      z: start height for each channel in 1/10 mm, must be in allowed machine range
    """

    await self.backend.send_command(module=self.module, command="STL", params=z)

  async def set_search_z_max(self, z: List[Optional[int]]):
    """Set z-max for liquid search commands.

    Args:
      z: max for each channel in 1/10 mm, must be in allowed machine range
    """

    await self.backend.send_command(module=self.module, command="SML", params=z)

  async def set_z_travel_height(self, z):
    """Set z-travel height.

    Args:
      z: travel heights in absolute 1/10 mm for each channel, must be in allowed
         machine range + 20
    """
    await self.backend.send_command(module=self.module, command="SHZ", params=z)

  async def move_detect_liquid(self, channels: int, zadd: List[Optional[int]]):
    """Move tip, detect liquid, submerge.

    Args:
      channels: binary coded tip select
      zadd: required distance to travel downwards in 1/10 mm for each channel,
            must be between 0 and z-start - z-max
    """

    await self.backend.send_command(
      module=self.module,
      command="MDT",
      params=[channels] + [None] * 3 + zadd,
    )

  async def set_slow_speed_z(self, speed: List[Optional[int]]):
    """Set slow speed for z.

    Args:
      speed: speed in 1/10 mm/s for each channel, must be between 1 and 4000
    """

    await self.backend.send_command(module=self.module, command="SSZ", params=speed)

  async def set_tracking_distance_z(self, rel: List[Optional[int]]):
    """Set z-axis relative tracking distance used by dispense and aspirate.

    Args:
      rel: relative value in 1/10 mm for each channel, must be between
            -2100 and 2100
    """

    await self.backend.send_command(module=self.module, command="STZ", params=rel)

  async def move_tracking_relative(self, rel: List[Optional[int]]):
    """Move tracking relative. Starts the z-drives and dilutors simultaneously
        to achieve a synchronous tracking movement.

    Args:
      rel: relative position for each plunger in full steps, must be between
           -3150 and 3150
    """

    await self.backend.send_command(module=self.module, command="MTR", params=rel)

  async def move_absolute_z(self, z: List[Optional[int]]):
    """Position absolute with slow speed z-axis

    Args:
      z: absolute position in 1/10 mm for each channel, must be in
         allowed machine range
    """

    await self.backend.send_command(module=self.module, command="MAZ", params=z)

  async def set_stop_speed_plunger(self, speed: List[Optional[int]]):
    """Set stop speed for plungers

    Args:
      speed: speed for each plunger in half step per second, must be between
             50 and 2700
    """

    await self.backend.send_command(module=self.module, command="SPP", params=speed)

  async def get_disposable_tip(self, tips, z_start, z_search):
    """Picks up tips

    Args:
      tips: binary coded tip select
      z_start: position in 1/10 mm where searching begins
      z_search: search distance in 1/10 mm, range within a tip must be found
    """

    await self.backend.send_command(
      module=self.module,
      command="AGT",
      params=[tips, z_start, z_search, 0],
    )

  async def discard_disposable_tip_high(self, tips):
    """Drops tips
    Discards at the Z-axes initialization height
    Args:
      tips: binary coded tip select
    """

    await self.backend.send_command(module=self.module, command="ADT", params=[tips])

  async def _drop_disposable_tip(self, tips, discard_height):
    """Drops tips
    Discards at a variable Z-axis initialization height

    Args:
      tips: binary coded tip select
      discard_height: binary. 0 above tip rack, 1 in tip rack
    """

    await self.backend.send_command(
      module=self.module, command="AST", params=[tips, discard_height]
    )


class Mca(EVOArm):
  """Provides firmware commands for the MCA 96-channel arm."""

  async def report_axis_parameter(self, selector: int) -> List[int]:
    """Report axis parameter for all axes (X, Y, Z, G).

    Args:
      selector: Parameter selector:
        0 - current position in 0.1 mm
        1 - acceleration in 0.1 mm/s^2
        2 - fast speed in 0.1 mm/s
        3 - initialization speed in 0.1 mm/s
        4 - initialization offset in 0.1 mm
        5 - upper limit (range) in 0.1 mm
        6 - initialization deviation in encoder increments
        7 - displacement offset in 0.1 mm
        8 - scale adjust factor
        10 - lower limit in 0.1 mm
        11 - target position in 0.1 mm
        12 - scale factor

    Returns:
      List of [X, Y, Z, G] values for the selected parameter.
    """
    resp = await self.backend.send_command(
      module=self.module, command="RPA", params=[selector]
    )
    data = resp["data"]
    # MCA RPA returns list of ints for X, Y, Z, G
    data_int = [int(v) for v in data]  # type: ignore[return-value]
    if len(data_int) < 4:
      raise ValueError(f"Unexpected RPA response length: {len(data_int)}")
    return data_int[:4]  # X, Y, Z, G

  async def report_x_param(self, param: int) -> int:
    """Report current parameter for X-axis.

    Args:
      param: 0 - current position, 5 - actual machine range
    """
    if param == 0:
      return (await self.report_axis_parameter(0))[0]
    elif param == 5:
      return (await self.report_axis_parameter(5))[0]
    else:
      raise ValueError(f"Unsupported X parameter selector: {param}")

  async def report_y_param(self, param: int) -> List[int]:
    """Report current parameter for Y-axis.

    Args:
      param: 0 - current position, 5 - actual machine range
    """
    if param == 0:
      return [(await self.report_axis_parameter(0))[1]]
    elif param == 5:
      return [(await self.report_axis_parameter(5))[1]]
    else:
      raise ValueError(f"Unsupported Y parameter selector: {param}")

  async def report_z_param(self, param: int) -> List[int]:
    """Report current parameter for Z-axis.

    Args:
      param: 0 - current position, 5 - actual machine range
    """
    if param == 0:
      return [(await self.report_axis_parameter(0))[2]]
    elif param == 5:
      return [(await self.report_axis_parameter(5))[2]]
    else:
      raise ValueError(f"Unsupported Z parameter selector: {param}")

  async def report_g_param(self, param: int) -> int:
    """Report current parameter for G-axis (gripper).

    Args:
      param: 0 - current position, 5 - actual machine range
    """
    if param == 0:
      return (await self.report_axis_parameter(0))[3]
    elif param == 5:
      return (await self.report_axis_parameter(5))[3]
    else:
      raise ValueError(f"Unsupported G parameter selector: {param}")

  async def position_initialization_x(self):
    """Reinitialize X-axis. MCA uses PIA for full initialization."""
    await self.backend.send_command(module=self.module, command="PIA")

  async def position_absolute_all_axis(
    self, x: Optional[int], y: Optional[int], z: Optional[int], g: Optional[int] = None
  ):
    """Position absolute for MCA axes.

    Args:
      x: absolute X position in 1/10 mm, or None to keep current
      y: absolute Y position in 1/10 mm, or None to keep current
      z: absolute Z position in 1/10 mm, or None to keep current
      g: absolute G position in 1/10 mm, or None to keep current
    """
    params = [x, y, z, g]
    # Filter trailing None values
    while params and params[-1] is None:
      params.pop()
    await self.backend.send_command(module=self.module, command="PAA", params=params)

  async def break_all_movement(self):
    """Stop all axis movement."""
    await self.backend.send_command(module=self.module, command="BMA", params=[0, 0, 0, 0])

  async def report_head_parameter(self, selector: int) -> List[int]:
    """Report head parameter.

    Args:
      selector: Parameter selector (device-specific)
    """
    resp = await self.backend.send_command(
      module=self.module, command="RHP", params=[selector]
    )
    data = resp["data"]
    if data is None:
      return []
    return [int(v) for v in data]  # type: ignore[return-value]


class RoMa(EVOArm):
  """
  Provides firmware commands for the RoMa plate robot
  """

  async def report_z_param(self, param: int) -> int:
    """Report current parameter for z-axis.

    Args:
      param: 0 - current position, 5 - actual machine range
    """

    resp: List[int] = (
      await self.backend.send_command(module=self.module, command="RPZ", params=[param])
    )["data"]
    return resp[0]

  async def report_r_param(self, param: int) -> int:
    """Report current parameter for r-axis.

    Args:
      param: 0 - current position, 5 - actual machine range
    """

    resp: List[int] = (
      await self.backend.send_command(module=self.module, command="RPR", params=[param])
    )["data"]
    return resp[0]

  async def report_g_param(self, param: int) -> int:
    """Report current parameter for g-axis.

    Args:
      param: 0 - current position, 5 - actual machine range
    """

    resp: List[int] = (
      await self.backend.send_command(module=self.module, command="RPG", params=[param])
    )["data"]
    return resp[0]

  async def set_smooth_move_x(self, mode: int):
    """Sets X-axis smooth move.

    Args:
      mode: 0 - active, 1 - normal acceleration and speed used
    """

    await self.backend.send_command(module=self.module, command="SSM", params=[mode])

  async def set_fast_speed_x(self, speed: Optional[int], accel: Optional[int] = None):
    """Set fast speed and acceleration for X-axis.

    Args:
      speed: fast speed in 1/10 mm/s
      accel: acceleration in 1/10 mm/s^2
    """

    await self.backend.send_command(module=self.module, command="SFX", params=[speed, accel])

  async def set_fast_speed_y(self, speed: Optional[int], accel: Optional[int] = None):
    """Set fast speed and acceleration for Y-axis.

    Args:
      speed: fast speed in 1/10 mm/s
      accel: acceleration in 1/10 mm/s^2
    """

    await self.backend.send_command(module=self.module, command="SFY", params=[speed, accel])

  async def set_fast_speed_z(self, speed: Optional[int], accel: Optional[int] = None):
    """Set fast speed and acceleration for Z-axis.

    Args:
      speed: fast speed in 1/10 mm/s
      accel: acceleration in 1/10 mm/s^2
    """

    await self.backend.send_command(module=self.module, command="SFZ", params=[speed, accel])

  async def set_fast_speed_r(self, speed: Optional[int], accel: Optional[int] = None):
    """Set fast speed and acceleration for R-axis.

    Args:
      speed: fast speed in 1/10 dg/s
      accel: acceleration in 1/10 dg/s^2
    """

    await self.backend.send_command(module=self.module, command="SFR", params=[speed, accel])

  async def set_vector_coordinate_position(
    self,
    v: int,
    x: int,
    y: int,
    z: int,
    r: int,
    g: Optional[int],
    speed: int,
    tw: int = 0,
  ):
    """Sets vector coordinate positions into table.

    Args:
      v: vector to be defined, must be between 1 and 100
      x: absolute x position in 1/10 mm
      y: absolute y position in 1/10 mm
      z: absolute z position in 1/10 mm
      r: absolute r position in 1/10 mm
      g: absolute g position in 1/10 mm
      speed: speed select, 0 - slow, 1 - fast
      tw: target window class, set with STW

    Raises:
      TecanError: if moving to the target position causes a collision
    """

    cur_x = EVOArm._pos_cache.setdefault(self.module, await self.report_x_param(0))
    for module, pos in EVOArm._pos_cache.items():
      if module == self.module:
        continue
      if cur_x < x and cur_x < pos < x:  # moving right
        raise TecanError("Invalid command (collision)", self.module, 2)
      if cur_x > x and cur_x > pos > x:  # moving left
        raise TecanError("Invalid command (collision)", self.module, 2)
      if abs(pos - x) < 1500:
        raise TecanError("Invalid command (collision)", self.module, 2)

    await self.backend.send_command(
      module=self.module,
      command="SAA",
      params=[v, x, y, z, r, g, speed, 0, tw],
    )

  async def action_move_vector_coordinate_position(self):
    """Starts coordinate movement, built by vector coordinate table."""

    await self.backend.send_command(module=self.module, command="AAC")

    EVOArm._pos_cache[self.module] = await self.report_x_param(0)

  async def position_absolute_g(self, g: int):
    """Position absolute for G-axis

    Args:
      g: absolute position in 1/10 mm
    """

    await self.backend.send_command(module=self.module, command="PAG", params=[g])

  async def set_gripper_params(self, speed: int, pwm: int, cur: Optional[int] = None):
    """Set gripper parameters

    Args:
      speed: gripper search speed in 1/10 mm/s
      pwm: pulse width modification limit
      cur: maximal allowed current
    """

    await self.backend.send_command(module=self.module, command="SGG", params=[speed, pwm, cur])

  async def grip_plate(self, pos: int):
    """Grips plate at current X/Y/Z/R-position

    Args:
      pos: target position, plate must be fetched within current and target position
    """

    await self.backend.send_command(module=self.module, command="AGR", params=[pos])

  async def set_target_window_class(self, wc: int, x: int, y: int, z: int, r: int, g: int):
    """Sets drive parameters for the AAC command.

    Args:
      wc: window class, must be between 1 and 100
      x: target window for x-axis in 1/10 mm
      y: target window for y-axis in 1/10 mm
      z: target window for z-axis in 1/10 mm
      r: target window for r-axis in 1/10 deg
      g: target window for g-axis in 1/10 mm
    """

    await self.backend.send_command(module=self.module, command="STW", params=[wc, x, y, z, r, g])


# Deprecated alias with warning # TODO: remove mid May 2025 (giving people 1 month to update)
# https://github.com/PyLabRobot/pylabrobot/issues/466


class EVO(EVOBackend):
  def __init__(self, *args, **kwargs):
    raise RuntimeError(
      "`EVO` is deprecated. Please use `EVOBackend` instead.",
    )
