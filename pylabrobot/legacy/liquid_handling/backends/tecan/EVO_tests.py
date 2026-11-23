import unittest
import unittest.mock
from unittest.mock import call

from pylabrobot.legacy.liquid_handling.backends.tecan.EVO_backend import (
  EVOBackend,
  LiHa,
  RoMa,
)
from pylabrobot.legacy.liquid_handling.standard import (
  GripDirection,
  ResourceDrop,
  ResourcePickup,
)
from pylabrobot.resources import (
  Coordinate,
  DeepWell_96_Well,
  DiTi_100ul_Te_MO,
  DiTi_SBS_3_Pos_MCA96,
  EVO150Deck,
  MP_3Pos_PCR,
)
from pylabrobot.resources.rotation import Rotation


class EVOTests(unittest.IsolatedAsyncioTestCase):
  """Test that the EVO backend is calling `send_command` correctly."""

  def setUp(self) -> None:
    super().setUp()

    # mock the EVO
    self.evo = EVOBackend(diti_count=8)
    self.evo.send_command = unittest.mock.AsyncMock()  # type: ignore[method-assign]

    async def send_command(module, command, params=None):
      if command == "RPX":  # report_x_param
        return {"data": [9000]}  # park position roma
      if command == "RPY":  # report_y_param
        return {"data": [90]}  # park position roma
      if command == "RPZ":  # report_z_param
        return {"data": [2000]}  # dummy value
      return {"data": None}

    self.evo.send_command.side_effect = send_command  # type: ignore[method-assign]

    self.deck = EVO150Deck()
    self.evo.set_deck(self.deck)

    # setup
    self.evo.setup = unittest.mock.AsyncMock()  # type: ignore[method-assign]
    self.evo._num_channels = 8
    self.evo._x_range = 2000  # TODO: override report_x_param
    self.evo._y_range = 2000
    self.evo._z_range = 2000
    self.evo._roma_connected = True
    self.evo._liha_connected = True
    self.evo.liha = LiHa(self.evo, EVOBackend.LIHA)
    self.evo.roma = RoMa(self.evo, EVOBackend.ROMA)

    # deck setup
    self.tr_carrier = DiTi_SBS_3_Pos_MCA96(name="tip_rack_carrier")
    try:
      self.tr_carrier[0] = self.tr = DiTi_100ul_Te_MO(name="tip_rack")
    except NotImplementedError as exc:
      self.skipTest(str(exc))
    self.deck.assign_child_resource(self.tr_carrier, rails=10)

    self.plate_carrier = MP_3Pos_PCR(name="plate_carrier")
    self.plate_carrier[0] = self.plate = DeepWell_96_Well(name="plate")
    self.deck.assign_child_resource(self.plate_carrier, rails=16)

    self.evo.send_command.reset_mock()

  # The upstream LiHa pick_up_tips/aspirate/dispense tests were removed with the upstream LiHa
  # logic (operator decision 2026-11-21): LiHa operations are tested in EVO_liha_backend_tests.py.

  async def test_move_resource(self):
    pickup = ResourcePickup(
      resource=self.plate,
      offset=Coordinate.zero(),
      pickup_distance_from_top=13.2,
      direction=GripDirection.FRONT,
    )
    drop = ResourceDrop(
      resource=self.plate,
      destination=self.plate_carrier[0].get_location_wrt(self.deck),
      destination_absolute_rotation=Rotation(0, 0, 0),
      offset=Coordinate.zero(),
      pickup_distance_from_top=13.2,
      pickup_direction=GripDirection.FRONT,
      direction=GripDirection.FRONT,
      rotation=0,
    )

    await self.evo.pick_up_resource(pickup)
    await self.evo.drop_resource(drop)

    self.evo.send_command.assert_has_calls(  # type: ignore[attr-defined]
      [
        call(module="C1", command="RPZ", params=[5]),
        call(module="C1", command="SSM", params=[1]),
        call(module="C1", command="SFX", params=[10000, None]),
        call(module="C1", command="SFY", params=[5000, 1500]),
        call(module="C1", command="SFZ", params=[1300, None]),
        call(module="C1", command="SFR", params=[5000, 1500]),
        call(module="C1", command="RPX", params=[0]),
        call(
          module="C1",
          command="SAA",
          params=[1, 5561, 2368, 1054, 900, None, 1, 0, 0],
        ),
        call(module="C1", command="AAC"),
        call(module="C1", command="RPX", params=[0]),
        call(module="C1", command="SSM", params=[0]),
        call(module="C1", command="PAG", params=[900]),
        call(module="C1", command="STW", params=[1, 0, 0, 0, 135, 0]),
        call(module="C1", command="RPX", params=[0]),
        call(
          module="C1",
          command="SAA",
          params=[1, 5561, 2368, 687, 900, None, 1, 0, 1],
        ),
        call(module="C1", command="RPX", params=[0]),
        call(
          module="C1",
          command="SAA",
          params=[1, 5561, 2368, 59, 900, None, 1, 0, 0],
        ),
        call(module="C1", command="AAC"),
        call(module="C1", command="RPX", params=[0]),
        call(module="C1", command="SFY", params=[3500, 1000]),
        call(module="C1", command="SFR", params=[2000, 600]),
        call(module="C1", command="SGG", params=[100, 75, None]),
        call(module="C1", command="AGR", params=[754]),
        call(module="C1", command="RPZ", params=[5]),
        call(module="C1", command="STW", params=[1, 0, 0, 0, 135, 0]),
        call(module="C1", command="STW", params=[2, 0, 0, 0, 53, 0]),
        call(module="C1", command="STW", params=[3, 0, 0, 0, 55, 0]),
        call(module="C1", command="STW", params=[4, 45, 0, 0, 0, 0]),
        call(module="C1", command="RPX", params=[0]),
        call(
          module="C1",
          command="SAA",
          params=[1, 5561, 2368, 59, 900, None, 1, 0, 1],
        ),
        call(module="C1", command="RPX", params=[0]),
        call(
          module="C1",
          command="SAA",
          params=[2, 5561, 2368, 687, 900, None, 1, 0, 2],
        ),
        call(module="C1", command="RPX", params=[0]),
        call(
          module="C1",
          command="SAA",
          params=[3, 5561, 2368, 1054, 900, None, 1, 0, 3],
        ),
        call(module="C1", command="RPX", params=[0]),
        call(
          module="C1",
          command="SAA",
          params=[4, 5561, 2368, 1054, 900, None, 1, 0, 4],
        ),
        call(module="C1", command="RPX", params=[0]),
        call(
          module="C1",
          command="SAA",
          params=[5, 5561, 2368, 687, 900, None, 1, 0, 3],
        ),
        call(module="C1", command="RPX", params=[0]),
        call(
          module="C1",
          command="SAA",
          params=[6, 5561, 2368, 59, 900, None, 1, 0, 0],
        ),
        call(module="C1", command="AAC"),
        call(module="C1", command="RPX", params=[0]),
        call(module="C1", command="PAG", params=[900]),
        call(module="C1", command="SFY", params=[5000, 1500]),
        call(module="C1", command="SFR", params=[5000, 1500]),
        call(module="C1", command="RPX", params=[0]),
        call(
          module="C1",
          command="SAA",
          params=[1, 5561, 2368, 59, 900, None, 1, 0, 1],
        ),
        call(module="C1", command="RPX", params=[0]),
        call(
          module="C1",
          command="SAA",
          params=[2, 5561, 2368, 687, 900, None, 1, 0, 2],
        ),
        call(module="C1", command="RPX", params=[0]),
        call(
          module="C1",
          command="SAA",
          params=[3, 5561, 2368, 1054, 900, None, 1, 0, 0],
        ),
        call(module="C1", command="AAC"),
        call(module="C1", command="RPX", params=[0]),
        call(module="C1", command="SFY", params=[3500, 1000]),
        call(module="C1", command="SFR", params=[2000, 600]),
      ]
    )
