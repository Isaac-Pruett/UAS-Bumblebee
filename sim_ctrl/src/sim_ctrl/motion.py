import math
from dataclasses import asdict, dataclass

import msgpack
import msgspec


class Movement(msgspec.Struct, array_like=True):
    x: float
    y: float
    z: float

    vx: float
    vy: float
    vz: float
    afx: float
    afy: float
    afz: float

    force_en: bool

    yaw: float
    yaw_rate: float


def bitnan(value: float) -> int:
    return 0 if math.isnan(value) else 1


def ned_all(_: Movement) -> None:
    pass


def enu_all(m: Movement) -> None:
    t = Movement(**asdict(m))

    m.x = t.y
    m.y = t.x
    m.z = -t.z

    m.vx = t.vy
    m.vy = t.vx
    m.vz = -t.vz

    m.afx = t.afy
    m.afy = t.afx
    m.afz = -t.afz


def nan_vel(m: Movement) -> None:
    m.vx = math.nan
    m.vy = math.nan
    m.vz = math.nan


def nan_acc(m: Movement) -> None:
    m.afx = math.nan
    m.afy = math.nan
    m.afz = math.nan


def nan_pos(m: Movement) -> None:
    m.x = math.nan
    m.y = math.nan
    m.z = math.nan


def enu_pos(m: Movement) -> None:
    enu_all(m)
    nan_vel(m)
    nan_acc(m)
