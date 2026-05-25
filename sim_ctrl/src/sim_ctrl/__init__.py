import math
import os
import time
from time import sleep

import zenoh
from pynput import keyboard

from sim_ctrl.motion import Movement

pressed = set()

NAME = os.environ.get("DRONE_NAME", "bingus")


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
        session = zenoh.open(zenoh.Config().from_env())
    except zenoh.ZError:
        session = zenoh.open(zenoh.Config())

    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release,
    )
    listener.start()

    pub = session.declare_publisher(f"{NAME}/mavlink/control/move/enu/raw")

    sleep(0.5)

    try:
        while True:
            vx = 0.0
            vy = 0.0
            yr = 0.0

            if "w" in pressed:
                vy += 1.0
            if "a" in pressed:
                vx -= 1.0
            if "s" in pressed:
                vy -= 1.0
            if "d" in pressed:
                vx += 1.0

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
            pub.put(pack.to_msgpack())

            sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        listener.stop()
        session.close()


if __name__ == "__main__":
    main()
