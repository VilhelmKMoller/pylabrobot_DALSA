from pylabrobot.resources.carrier import TubeCarrier, PlateCarrier, ShakerCarrier, TipCarrier, MFXCarrier, create_homogeneous_carrier_sites, Coordinate

# dummy CSV_Shaker0/vacume system
def CVS_Shaker_DTU(name: str) -> ShakerCarrier:
  """ Hamilton cat. no.: costum by vikmol """
  return ShakerCarrier(
    name=name,
    size_x=157.5,
    size_y=497.0,
    size_z=90.0, # original 90.0
    sites=create_homogeneous_carrier_sites([
        # Coordinate(1.1, 167.5, 25.5), # Site.1.(X,Y,Z) !!!
        # Coordinate(14.6, 172.1, 77), # Site.2.(X,Y,Z)
        # Coordinate(1.2, 48.5, 25.5), # Site.3.(X,Y,Z) !!!
        # Coordinate(14.6, 53, 77),    # Site.4.(X,Y,Z)
        # Coordinate(14.6, 53.0, 20.5),# Site.5.(X,Y,Z) ???
        Coordinate(14.6, 286.9, 77),  # Site.7.(X,Y,Z) orignal z=75
        Coordinate(14.6, 392.1, 77), # Site.6.(X,Y,Z)  orignal z=75
      ],
      site_size_x=127.0, # original
      site_size_y=86.0, # original ¨
      # site_size_x2=154 # Alternative coordinates
      # site_size_y2=95 # Alternative coordinates
    ),
    model="CVS_Shaker_DTU"
  )



# attempt to build own tip tip settup original z-value = 29.0
def TIP_CAR_NTR_A00(name: str) -> TipCarrier:
  """ Carrier with 5 nestable tip rack positions """
  return TipCarrier(
    name=name,
    size_x=135.0,
    size_y=497.0,
    size_z=130.0, # 200.0 # 130
    sites=create_homogeneous_carrier_sites([
        Coordinate(6.2, 10.0, 82.4), # original z=29 63.4 all, change with -26 from 108.4
        Coordinate(6.2, 106.0, 82.4),
        Coordinate(6.2, 202.0, 82.4),
        Coordinate(6.2, 298.0, 82.4),
        Coordinate(6.2, 394.0, 82.4)
      ],
      site_size_x=122.4,
      site_size_y=82.6,
    ),
    model="TIP_CAR_NTR_A00"
  )

# MFXCarrier
def MFX_CAR_DTU(name: str) -> MFXCarrier:
  """ Hamilton cat. no.: costum by vikmol """
  return MFXCarrier(
    name=name,
    size_x=135.0,
    size_y=497.0,
    size_z=130.0, # original 18.195
    sites=create_homogeneous_carrier_sites([
        Coordinate(x=6.3, y=10.2, z=131.0),   # Site.3.(X,Y,Z) original z=77.0
        Coordinate(x=6.3, y=106.2, z=131.0),  # Site.2.(X,Y,Z) original z=77.0
        Coordinate(x=6.3, y=202.2, z=114.7), # Site.1.(X,Y,Z)
        Coordinate(x=6.3, y=298.2, z=114.7), # Site.5.(X,Y,Z)
        Coordinate(x=6.3, y=394.2, z=114.7), # Site.4.(X,Y,Z)
      ],
      site_size_x=122.4, # original 135.0
      site_size_y=82.6, # original 94.0
    ),
    model="MFX_CAR_DTU"
  )


# Reagent reservar 1:
def RGT_CAR_3R_A01(name: str) -> TubeCarrier:
  """ Hamilton cat. no.: costum by vikmol """
  return TubeCarrier(
    name=name,
    size_x = 22.5, # Dim.Dx
    size_y = 497, # Dim.Dy
    size_z = 136.0, # Dim.Dz
    sites=create_homogeneous_carrier_sites([
        Coordinate(x=0.75, y=9.1, z=18.5), # Site.1.(X,Y,Z)
        Coordinate(x=0.55, y=172.5, z=18.5),   # Site.3.(X,Y,Z)
        Coordinate(x=0.75, y=335.9, z=18.5),  # Site.2.(X,Y,Z)
      ],
      site_size_x= 21, # Site.1.Dx122.4
      site_size_y= 142  # Site.1.Dy82.6
    ),
    model="RGT_CAR_3R_A01_1"
  )


# Reagent reservar 2:
def RGT_CAR_5R_A00(name: str) -> TubeCarrier:
  """ Hamilton cat. no.: costum by vikmol """
  return TubeCarrier(
    name=name,
    size_x = 22.5, # Dim.Dx
    size_y = 497, # Dim.Dy
    size_z = 82.0, # Dim.Dz
    sites=create_homogeneous_carrier_sites([
          Coordinate(x=1.25, y=6.5, z=18.5),   # Site.3.(X,Y,Z)
          Coordinate(x=1.25, y=102.5, z=18.5),  # Site.2.(X,Y,Z)
          Coordinate(x=1.25, y=198.5, z=18.5), # Site.1.(X,Y,Z)
          Coordinate(x=1.25, y=294.5, z=18.5),   # Site.5.(X,Y,Z)
          Coordinate(x=1.25, y=390.5, z=18.5),   # Site.4.(X,Y,Z)
      ],
      site_size_x= 20, # Site.1.Dx122.4
      site_size_y= 90  # Site.1.Dy82.6
    ),
    model="RGT_CAR_5R_A00"
  )


# Coolers
def COOLER(name: str) -> TubeCarrier:
  """ Hamilton cat. no.: costum by vikmol """
  return TubeCarrier(
    name=name,
    size_x = 135, # Dim.Dx
    size_y = 497, # Dim.Dy
    size_z = 130, # Dim.Dz
    sites=create_homogeneous_carrier_sites([
          Coordinate(x=4, y=8.5, z=81.5),  # Site.2.(X,Y,Z)
          Coordinate(x=4, y=104.5, z=81.5), # Site.1.(X,Y,Z)
          Coordinate(x=4, y=200.5, z=81.5), # Site.5.(X,Y,Z)
          Coordinate(x=4, y=296.5, z=81.5), # Site.4.(X,Y,Z)
          Coordinate(x=4, y=392.5, z=81.5),   # Site.3.(X,Y,Z)
      ],
      site_size_x= 127, # Site.1.Dx122.4
      site_size_y= 86  # Site.1.Dy82.6
    ),
    model="COOLER"
  )

# Reagent reservar 3:
name = "my_SMP_CAR_24_A00"
# Dim.Dx22.5Dim.Dy497Dim.Dz140