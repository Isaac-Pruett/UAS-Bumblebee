import math
import os
import time
from time import sleep
from types import NoneType

import msgpack
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

    pub = session.declare_publisher("mavlink/control/move/flu/raw")

    sub = session.declare_subscriber("mavlink/telemetry/pos/local/enu")

    sleep(0.5)

    try:
        while True:
            vx = 0.0
            vy = 0.0
            yr = 0.0

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
                print("q")
            if "e" in pressed:
                yr += 1.0
                print("e")

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
            pub.put(pack.to_msgpack())

            recvpack = sub.handler.try_recv()
            if recvpack is not None:
                pl = recvpack.payload.to_bytes()

                # print(f"pos packet recv: {msgpack.unpackb(pl, raw=False)}")

            sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        listener.stop()
        session.close()


if __name__ == "__main__":
    main()
