import msgspec


class Vec3(msgspec.Struct, array_like=True):
    w: float
    x: float
    y: float
    z: float
