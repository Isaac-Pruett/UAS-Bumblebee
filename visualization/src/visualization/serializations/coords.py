import msgspec
import zenoh
from foxglove.messages import LocationFix

from visualization.foxglove_zenoh_adapter import sample_timestamp


class Coords(msgspec.Struct, array_like=True):
    lat_deg_e7: int
    lon_deg_e7: int
    alt_mm: int


def coords_to_location(
    coords: Coords,
    sample: zenoh.Sample,
) -> LocationFix:
    return LocationFix(
        timestamp=sample_timestamp(sample),
        frame_id="gps",
        latitude=coords.lat_deg_e7 / 1e7,
        longitude=coords.lon_deg_e7 / 1e7,
        altitude=coords.alt_mm / 1000.0,
    )
