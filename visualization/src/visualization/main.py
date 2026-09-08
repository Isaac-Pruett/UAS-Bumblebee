import time
from time import sleep

import foxglove
import msgspec.msgpack
import zenoh
from foxglove.messages import (
    Color,
    Pose,
    Quaternion,
    SceneEntity,
    SceneUpdate,
    SpherePrimitive,
    Vector3,
)

from visualization.foxglove_zenoh_adapter import (
    MsgPackFoxgloveAdapter,
    MsgPackFoxgloveLogger,
    sample_timestamp,
)
from visualization.serializations.coords import Coords, coords_to_location
from visualization.serializations.Vec3 import Vec3

foxglove.start_server()


def vec3_to_ball(
    v: Vec3,
    sample: zenoh.Sample,
    color: Color | None = None,
) -> SceneUpdate:
    if color is None:
        color = Color(
            r=1.0,
            g=0.0,
            b=0.0,
            a=0.8,
        )
    sphere = SpherePrimitive(
        pose=Pose(
            position=Vector3(
                x=v.x,
                y=v.y,
                z=v.z,
            ),
            orientation=Quaternion(
                x=0.0,
                y=0.0,
                z=0.0,
                w=1.0,
            ),
        ),
        size=Vector3(
            x=0.25,
            y=0.25,
            z=0.25,
        ),
        color=color,
    )

    name = str(sample.key_expr).split("/", maxsplit=1)[0]

    entity = SceneEntity(
        timestamp=sample_timestamp(sample),
        frame_id="enu",
        id=f"vehicle_{name}",
        spheres=[sphere],
    )

    return SceneUpdate(
        entities=[entity],
        deletions=[],
    )


def identity_collector(
    sample: zenoh.Sample,
):
    pass


def main():
    try:
        session = zenoh.open(zenoh.Config().from_env())
    except zenoh.ZError:
        session = zenoh.open(zenoh.Config())

    pos_adapter = MsgPackFoxgloveAdapter(
        packet_type=Vec3,
        render=vec3_to_ball,
    )
    poshandler = MsgPackFoxgloveLogger(pos_adapter)

    pos_sub = session.declare_subscriber(
        "**/mavlink/telemetry/pos/local/enu", handler=poshandler.log
    )

    coords_adapter = MsgPackFoxgloveAdapter(
        packet_type=Coords,
        render=coords_to_location,
    )
    coords_handler = MsgPackFoxgloveLogger(coords_adapter)

    coords_sub = session.declare_subscriber(
        "**/mavlink/telemetry/pos/global/int", handler=coords_handler.log
    )

    origin_setter = session.declare_publisher("**/mavlink/origin/set")

    # coords_sub_for_origin_setting = session.declare_subscriber(
    #     "**/mavlink/telemetry/pos/global/int",
    # )

    # identity_getter = session.declare_querier(
    #     "**/identity/whoami",
    # )

    # Wait for subscribers to be ready
    sleep(0.5)

    origin = Coords(337_489_970, -843_879_850, 300)
    origin_setter.put(msgspec.msgpack.encode(origin))

    while True:
        foxglove.log("/sim_alive", {"time": time.time()})
        time.sleep(0.3)

    print("Python done!")
    session.close()


if __name__ == "__main__":
    main()
