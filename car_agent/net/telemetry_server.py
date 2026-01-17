from __future__ import annotations

import socket
from typing import Tuple

from .protocol import dumps




def _parse_host_port(s: str) -> Tuple[str, int]:
    host, port = s.rsplit(":", 1)
    return host.strip(), int(port)


class UdpTelemetryClient:
    def __init__(self, peer: str) -> None:
        self.peer = _parse_host_port(peer)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, obj: dict) -> None:
        try:
            self.sock.sendto(dumps(obj), self.peer)
        except Exception:
            pass

    def close(self) -> None:
        try:
            self.sock.close()
        except Exception:
            pass
