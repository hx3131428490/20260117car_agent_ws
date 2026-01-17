from __future__ import annotations

import socket
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

from .protocol import loads


def _parse_host_port(s: str) -> Tuple[str, int]:
    host, port = s.rsplit(":", 1)
    return host.strip(), int(port)


@dataclass
class CmdSnapshot:
    seq: int = 0
    t: float = 0.0
    vx: float = 0.0
    wz: float = 0.0
    mode: str = "idle"
    rx_time: float = 0.0


class UdpCmdServer:
    def __init__(self, car_id: str, listen: str) -> None:
        self.car_id = car_id
        self.listen = listen

        self._sock: Optional[socket.socket] = None
        self._th: Optional[threading.Thread] = None
        self._stop = threading.Event()

        self._lock = threading.Lock()
        self._latest = CmdSnapshot()

        self.rx_count = 0
        self.parse_err = 0
        self.last_sender: Optional[Tuple[str, int]] = None

    def start(self) -> None:
        if self._th is not None and self._th.is_alive():
            return

        host, port = _parse_host_port(self.listen)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((host, port))
        sock.settimeout(0.2)

        self._sock = sock
        self._stop.clear()
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def _run(self) -> None:
        assert self._sock is not None
        while not self._stop.is_set():
            try:
                data, addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except Exception:
                continue

            self.last_sender = addr
            try:
                msg = loads(data)
                if msg.get("type") != "cmd":
                    continue

                dst = str(msg.get("car_id", ""))
                if dst not in ("", "broadcast", self.car_id):
                    continue

                snap = CmdSnapshot(
                    seq=int(msg.get("seq", 0)),
                    t=float(msg.get("t", 0.0)),
                    vx=float(msg.get("vx", 0.0)),
                    wz=float(msg.get("wz", 0.0)),
                    mode=str(msg.get("mode", "auto")),
                    rx_time=time.time(),
                )
                with self._lock:
                    self._latest = snap
                self.rx_count += 1
            except Exception:
                self.parse_err += 1

    def get_latest(self) -> CmdSnapshot:
        with self._lock:
            return self._latest

    def stop(self) -> None:
        self._stop.set()
        if self._th is not None:
            self._th.join(timeout=1.0)
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
        self._th = None
        self._sock = None
