import msgspec


class Coords(msgspec.Struct, array_like=True):
    lat_deg_e7: int
    lon_deg_e7: int
    alt_mm: int
