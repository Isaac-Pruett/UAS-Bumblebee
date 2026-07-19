import math
import os
from ast import Subscript
from base64 import decode
from time import sleep
from types import NoneType

import msgpack
import msgspec
import msgspec.msgpack
import zenoh
from pynput import keyboard

from sim_ctrl.motion import Movement

pressed = set()


def on_press(key):
    try:
        pressed.add(key.char)
    except AttributeError:
        pass


def on_release(key):
    try:
        pressed.discard(key.char)
    except AttributeError:
        pass


def collect_pos(sub: zenoh.Subscriber) -> tuple[float, float, float] | None:
    recvpack = sub.handler.try_recv()
    if recvpack is not None:
        pl = recvpack.payload.to_bytes()
        return msgspec.msgpack.decode(pl)


def main():
    try:
        fp = os.environ.get("ZENOH_CONFIG")
        if fp is None:
            raise zenoh.ZError("ZENOH_CONFIG not set")
        cfg = zenoh.Config().from_file(fp)

    except zenoh.ZError:
        cfg = zenoh.Config()

    session = zenoh.open(cfg)

    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release,
    )
    listener.start()

    pub = session.declare_publisher("mavlink/control/move/rfu/raw")

    land_pub = session.declare_publisher("mavlink/control/move/land")

    sub = session.declare_subscriber("mavlink/telemetry/pos/local/enu")

    sleep(0.5)

    try:
        while True:
            vx = 0.0
            vy = 0.0
            yr = 0.0

            if "l" in pressed:
                cmd = {
                    "height_m": 10.0,
                    "yaw": 0.0,
                }

                print("Querying launch")
                payload = msgpack.packb(cmd, use_bin_type=True)

                replies = session.get(
                    "mavlink/control/move/launch",
                    payload=payload,
                    timeout=1.0,
                )

                errors = [
                    msgspec.msgpack.decode(reply.err.payload.to_bytes())
                    for reply in replies
                    if reply.err is not None
                ]

                if 0b1 not in errors:  # CantFlyError
                    optional_tuple = collect_pos(sub)
                    x, y, z = (0, 0, 0)
                    while (optional_tuple is None) or z < (10.0 - 0.3):
                        optional_tuple = collect_pos(sub)
                        if optional_tuple is not None:
                            x, y, z = optional_tuple
                            print(f"x = {x}, y = {y}, z = {z}")

                        pass

            if "o" in pressed:
                cmd = {
                    "coords": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": -2.0,
                    },
                    "descend_speed_m_s": 3.0,
                    "acceptance_rad_m": 2.0,
                    "yaw": 0.0,
                }

            if "w" in pressed:
                vy += 20.0
            if "a" in pressed:
                vx -= 20.0
            if "s" in pressed:
                vy -= 20.0
            if "d" in pressed:
                vx += 20.0

            if "q" in pressed:
                yr -= 1.0

            if "e" in pressed:
                yr += 1.0

            pack = Movement(
                x=math.nan,
                y=math.nan,
                z=math.nan,
                vx=vx,
                vy=vy,
                vz=0.0,
                afx=math.nan,
                afy=math.nan,
                afz=math.nan,
                force_en=False,
                yaw=math.nan,
                yaw_rate=yr,
            )
            pub.put(msgspec.msgpack.encode(pack))

            # print(f"pos packet recv: {msgpack.unpackb(pl, raw=False)}")

            sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        listener.stop()
        session.close()


if __name__ == "__main__":
    main()
