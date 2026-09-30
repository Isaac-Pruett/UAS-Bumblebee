from __future__ import annotations

import logging
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import foxglove
import msgspec
import zenoh
from foxglove.messages import FoxgloveMessage, Timestamp


def msgpack_to_json(sample: zenoh.Sample) -> dict[str, Any]:
    packet = msgspec.msgpack.decode(sample.payload.to_bytes())
    # Normalize MessagePack values (including binary data) to JSON-compatible values.
    return {"data": msgspec.json.decode(msgspec.json.encode(packet))}


@dataclass(frozen=True)
class MsgPackFoxgloveAdapter[T]:
    packet_type: type[T]
    render: Callable[[T, zenoh.Sample], FoxgloveMessage]

    def __call__(self, sample: zenoh.Sample) -> FoxgloveMessage:
        packet = msgspec.msgpack.decode(
            sample.payload.to_bytes(),
            type=self.packet_type,
        )

        return self.render(packet, sample)


class MsgPackFoxgloveLogger:
    def __init__(
        self,
        converter: Callable[[zenoh.Sample], FoxgloveMessage | dict[str, Any]],
        topic: Callable[[zenoh.Sample], str] | None = None,
    ):
        self.converter = converter
        self.topic = topic or (lambda sample: str(sample.key_expr))

    def log(self, sample: zenoh.Sample) -> None:
        if sample.kind != sample.kind.PUT:
            return

        try:
            message = self.converter(sample)
        except (msgspec.DecodeError, TypeError, ValueError) as exc:
            logging.getLogger(__name__).warning(
                "Skipping invalid telemetry on %s: %s", sample.key_expr, exc
            )
            return

        foxglove.log(
            self.topic(sample),
            message,
        )


def sample_timestamp(sample: zenoh.Sample) -> Timestamp:
    if sample.timestamp is None:
        ns = time.time_ns()
    else:
        dt = sample.timestamp.get_time()
        ns = int(dt.timestamp() * 1_000_000_000)

    return Timestamp(
        sec=ns // 1_000_000_000,
        nsec=ns % 1_000_000_000,
    )
