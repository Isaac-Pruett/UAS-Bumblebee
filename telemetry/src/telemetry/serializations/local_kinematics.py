import msgspec

from .Vec3 import Vec3


class LocalKinematics(msgspec.Struct, array_like=True):
    pos: Vec3
    velo: Vec3
    acc: Vec3
