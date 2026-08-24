import os
import time
from collections.abc import Callable

import foxglove
import zenoh
from foxglove.messages import FoxgloveMessage, Log, LogLevel

server = foxglove.start_server()


def foxglove_logger(
    message_callback: Callable[[zenoh.Sample], FoxgloveMessage],
) -> Callable[[zenoh.Sample], None]:

    def log_to_foxglove(sample: zenoh.Sample) -> None:
        keyexpr = str(sample.key_expr)

        if not keyexpr.startswith("/"):
            keyexpr = "/" + keyexpr

        message: FoxgloveMessage = message_callback(sample)
        foxglove.log(keyexpr, message)

    return log_to_foxglove


def main():
    if os.environ.get("ZENOH_CONFIG"):
        session = zenoh.open(zenoh.Config.from_file(os.environ["ZENOH_CONFIG"]))
    else:
        session = zenoh.open(zenoh.Config())

    logger = foxglove_logger(
        lambda sample: Log(
            level=LogLevel.Info,
            message="Hello to Foxglove!",
        )
    )

    # Everything logged with foxglove.log() while this context is active
    # will also be recorded to this MCAP file.
    # with foxglove.open_mcap("example.mcap"):
    universal_recv = session.declare_subscriber(
        "**",
        handler=logger,
    )

    try:
        while True:
            time.sleep(0.033)
    finally:
        universal_recv.undeclare()
        session.close()


if __name__ == "__main__":
    main()
