from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass

import foxglove
import msgspec
import zenoh
from foxglove.messages import FoxgloveMessage, Timestamp


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
        converter: Callable[[zenoh.Sample], FoxgloveMessage],
        topic: Callable[[zenoh.Sample], str] | None = None,
    ):
        self.converter = converter
        self.topic = topic or (lambda sample: str(sample.key_expr))

    def log(self, sample: zenoh.Sample) -> None:
        if sample.kind != sample.kind.PUT:
            return

        foxglove.log(
            self.topic(sample),
            self.converter(sample),
        )


def sample_timestamp(sample: zenoh.Sample) -> Timestamp:
    ns = (
        time.time_ns()
        if sample.timestamp is None
        else sample.timestamp.get_time_as_ntp64().as_nanos()
    )

    return Timestamp(
        sec=ns // 1_000_000_000,
        nsec=ns % 1_000_000_000,
    )
