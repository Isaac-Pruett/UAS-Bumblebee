import msgspec


class Vec3(msgspec.Struct, array_like=True):
    x: float
    y: float
    z: float
