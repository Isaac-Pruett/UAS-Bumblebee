import time
from time import sleep

import foxglove
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
from visualization.serializations.Vec3 import Vec3

foxglove.start_server()


def vec3_to_ball(
    v: Vec3,
    sample: zenoh.Sample,
) -> SceneUpdate:
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
        color=Color(
            r=1.0,
            g=0.0,
            b=0.0,
            a=1.0,
        ),
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


def main():
    try:
        session = zenoh.open(zenoh.Config().from_env())
    except zenoh.ZError:
        session = zenoh.open(zenoh.Config())

    adapter = MsgPackFoxgloveAdapter(
        packet_type=Vec3,
        render=vec3_to_ball,
    )
    handler = MsgPackFoxgloveLogger(adapter)

    sub = session.declare_subscriber(
        "**/mavlink/telemetry/pos/local/enu", handler=handler.log
    )

    # Wait for subscribers to be ready
    sleep(0.5)

    # /luke/mavlink/telemetry/pos/local/enu
    #

    while True:
        foxglove.log("/sim_alive", {"time": time.time()})
        time.sleep(0.03)

    print("Python done!")
    session.close()


if __name__ == "__main__":
    main()
