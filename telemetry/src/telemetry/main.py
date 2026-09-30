import logging
import os
import time

import foxglove
import zenoh
from foxglove.messages import (
    Color,
    Duration,
    Pose,
    Quaternion,
    SceneEntity,
    SceneUpdate,
    SpherePrimitive,
    Vector3,
)

from telemetry.foxglove_zenoh_adapter import (
    MsgPackFoxgloveAdapter,
    MsgPackFoxgloveLogger,
    msgpack_to_json,
    sample_timestamp,
)
from telemetry.serializations.coords import Coords, coords_to_location
from telemetry.serializations.local_kinematics import LocalKinematics


def local_kinematics_to_ball(
    kinematics: LocalKinematics,
    sample: zenoh.Sample,
    color: Color | None = None,
) -> SceneUpdate:
    position = kinematics.pos
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
                x=position.x,
                y=position.y,
                z=position.z,
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
        lifetime=Duration(sec=2, nsec=0),
        spheres=[sphere],
    )

    return SceneUpdate(
        entities=[entity],
        deletions=[],
    )


def main():
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)
    # Do not inherit a drone's namespaced ZENOH_CONFIG from nix develop.
    config_path = os.environ.get("TELEMETRY_ZENOH_CONFIG")
    if config_path:
        config = zenoh.Config.from_file(config_path)
    else:
        config = zenoh.Config()
        config.insert_json5("mode", '"client"')
        config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
        config.insert_json5("scouting/multicast/enabled", "false")

    logger.info(
        "Connecting telemetry to Zenoh using %s", config_path or "tcp/127.0.0.1:7447"
    )
    try:
        session = zenoh.open(config)
    except zenoh.ZError:
        logger.exception("Telemetry could not connect to its configured Zenoh router")
        raise

    server = foxglove.start_server()

    # The MAVLink bridge emits this stream only after confirming the FC's EKF
    # origin. Preserve those local coordinates; never project or add offsets here.
    pos_adapter = MsgPackFoxgloveAdapter(
        packet_type=LocalKinematics,
        render=local_kinematics_to_ball,
    )
    poshandler = MsgPackFoxgloveLogger(pos_adapter)
    pos_sub = session.declare_subscriber(
        "**/mavlink/telemetry/state/local/enu", handler=poshandler.log
    )

    coords_adapter = MsgPackFoxgloveAdapter(
        packet_type=Coords,
        render=coords_to_location,
    )
    coords_handler = MsgPackFoxgloveLogger(coords_adapter)

    coords_sub = session.declare_subscriber(
        "**/mavlink/telemetry/pos/global/int", handler=coords_handler.log
    )

    all_topics_collector = MsgPackFoxgloveLogger(
        msgpack_to_json,
        topic=lambda sample: f"/raw/{sample.key_expr}",
    )
    all_topics_sub = session.declare_subscriber("**", handler=all_topics_collector.log)

    # Keep server/subscriber handles alive for the entire bridge lifetime.
    try:
        while True:
            foxglove.log("/gcs_telem_alive", {"time": time.time()})
            time.sleep(0.3)
    except KeyboardInterrupt:
        pass
    finally:
        session.close()
        server.stop()


if __name__ == "__main__":
    main()
