from typing import Dict, Optional

from pylabrobot.resources.carrier import (
  Carrier,
  ResourceHolder,
  create_resources,
)
from pylabrobot.resources.coordinate import Coordinate
from pylabrobot.resources.tecan.tecan_resource import TecanResource
from pylabrobot.resources.trash import Trash


class TecanWashStation(Carrier, TecanResource):
  """Base class for Tecan tip carriers."""

  def __init__(
    self,
    name: str,
    size_x: float,
    size_y: float,
    size_z: float,
    off_x: float,
    off_y: float,
    sites: Optional[Dict[int, ResourceHolder]] = None,
    category="tecan_wash_station",
    model: Optional[str] = None,
  ):
    super().__init__(
      name,
      size_x,
      size_y,
      size_z,
      sites,
      category=category,
      model=model,
    )

    self.off_x: float = off_x
    self.off_y: float = off_y


def Wash_Station(name: str) -> TecanWashStation:
  """Tecan part no. 10613001"""
  return TecanWashStation(
    name=name,
    size_x=25.0,
    size_y=390.0,
    size_z=0.0,
    off_x=12.5,
    off_y=24.7,
    sites=create_resources(
      klass=ResourceHolder,
      locations=[
        Coordinate(12.2, 106.7, 0.0),
        Coordinate(11.0, 180.7, 0.0),
        Coordinate(12.2, 281.7, 0.0),
      ],
      resource_size_x=[
        12.0,
        12.0,
        12.0,
      ],
      resource_size_y=[
        73.0,
        100.0,
        73.0,
      ],
    ),
    model="Wash_Station",
  )


def Wash_Station_Waste(name: str) -> Trash:
  return Trash(name=name, size_x=12.0, size_y=100.0, size_z=140.0)


def Wash_Station_Cleaner_shallow(name: str) -> Trash:
  return Trash(name=name, size_x=12.0, size_y=73.0, size_z=140.0)


def Wash_Station_Cleaner_deep(name: str) -> Trash:
  return Trash(name=name, size_x=12.0, size_y=73.0, size_z=140.0)


def Washstation_2Grid_DiTi_Waste(name: str) -> Trash:
  """EVOware labware "Washstation 2Grid DiTi Waste": the LiHa DiTi waste chute of carrier
  "DiTi Waste 3 Trough + Wash" (site 7).

  Outline only (the site's 50 x 93 mm; EVOware's labware record has no usable size). The LiHa
  backend drops DiTis at EVOware's fixed chute position, not at this resource's PLR location.
  """
  return Trash(
    name=name, size_x=50.0, size_y=93.0, size_z=40.0, model="Washstation_2Grid_DiTi_Waste"
  )


def DiTi_Waste_3_Trough_Wash(name: str) -> TecanWashStation:
  """EVOware carrier "DiTi Waste 3 Trough + Wash" (Carrier.cfg id 322), two grids wide.

  Sites from Carrier.cfg (Tecan units / 10; PLR y = carrier depth - site y - site depth):

  ========  ===========  =================================  =========================
  PLR idx   EVOware      position (x, y, z) mm, size mm     REFINES worktable
  ========  ===========  =================================  =========================
  0         site 7       (0, -15, 6), 50 x 93               DiTi waste chute
  1         site 6       (25.5, 94.6, 16), 24 x 84          Trough 100ml
  2         site 5       (25.5, 182, 16), 24 x 84           Trough 100ml
  3         site 4       (25.5, 269.4, 16), 24 x 84         Trough 100ml
  4         site 3       (0, 111, 6), 25 x 71               Wash Station Cleaner shallow
  5         site 2       (0, 184, 6), 25 x 92               Wash Station Waste
  6         site 1       (0, 278, 6), 25 x 71               Wash Station Cleaner shallow
  ========  ===========  =================================  =========================

  PLR index = 7 - EVOware site, as for the other Tecan carriers here (EVOware's numbering is not
  strictly back to front on this carrier, so do not infer it from Y). Carrier.cfg lists an eighth,
  empty site (0 x 0 at 0/0/0); it holds nothing and is left out. The chute site reaches 15 mm past
  the carrier's front edge.
  """
  sites = [  # EVOware sites 7..1: (x, y, z), (width, depth) in mm
    ((0.0, -15.0, 6.0), (50.0, 93.0)),
    ((25.5, 94.6, 16.0), (24.0, 84.0)),
    ((25.5, 182.0, 16.0), (24.0, 84.0)),
    ((25.5, 269.4, 16.0), (24.0, 84.0)),
    ((0.0, 111.0, 6.0), (25.0, 71.0)),
    ((0.0, 184.0, 6.0), (25.0, 92.0)),
    ((0.0, 278.0, 6.0), (25.0, 71.0)),
  ]
  return TecanWashStation(
    name=name,
    size_x=50.0,
    size_y=352.0,
    size_z=6.0,
    off_x=12.5,
    off_y=14.5,
    sites=create_resources(
      klass=ResourceHolder,
      locations=[Coordinate(*xyz) for xyz, _ in sites],
      resource_size_x=[w for _, (w, _) in sites],
      resource_size_y=[d for _, (_, d) in sites],
      name_prefix=name,
    ),
    model="DiTi_Waste_3_Trough_Wash",
  )
