"""Reserved hardware transport boundary.

The v0.1 milestone intentionally exercises UDP/SITL first. The UART endpoint will
reuse the same adapter contract after bench validation on the Jetson Orin Nano.
"""


class SerialTransport:
    def __init__(self, endpoint: str, baud: int) -> None:
        self.endpoint = endpoint
        self.baud = baud

    def open(self) -> None:
        raise NotImplementedError("serial transport is deferred until the SITL milestone passes")
