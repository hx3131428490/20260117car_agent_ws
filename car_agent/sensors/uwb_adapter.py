from __future__ import annotations

import time
from dataclasses import dataclass
from multiprocessing import Process, shared_memory
from typing import Optional

import numpy as np

from . import uwb_serial_io


@dataclass
class UwbState:
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    stamp: float = 0.0
    err: int = 1
    rx_age_s: float = 1e9


class UwbAdapter:
    def __init__(self, serial_port: str, baudrate: int = 921600) -> None:
        self.serial_port = serial_port
        self.baudrate = int(baudrate)

        self._shm: Optional[shared_memory.SharedMemory] = None
        self._arr: Optional[np.ndarray] = None
        self._proc: Optional[Process] = None

    def start(self) -> None:
        if self._proc is not None and self._proc.is_alive():
            return

        arr, shm = uwb_serial_io.init_UWB_shm()
        self._arr = arr
        self._shm = shm

        self._proc = Process(
            target=uwb_serial_io.read_UWB,
            args=(self._arr.shape, self._arr.dtype, self._shm.name, self.serial_port, self.baudrate),
            daemon=True,
        )
        self._proc.start()

    def is_alive(self) -> bool:
        return self._proc is not None and self._proc.is_alive()

    def get_latest(self) -> UwbState:
        if self._arr is None:
            return UwbState()

        a = self._arr.copy()
        x, y, vx, vy, stamp, err = float(a[0]), float(a[1]), float(a[2]), float(a[3]), float(a[4]), int(a[5])

        age = 1e9
        now = time.time()
        if stamp > 1e-6:
            age = max(0.0, now - stamp)

        return UwbState(x=x, y=y, vx=vx, vy=vy, stamp=stamp, err=err, rx_age_s=age)

    def stop(self) -> None:
        if self._proc is not None and self._proc.is_alive():
            self._proc.terminate()
            self._proc.join(timeout=1.0)
        self._proc = None

        if self._shm is not None:
            try:
                self._shm.close()
            finally:
                try:
                    self._shm.unlink()
                except Exception:
                    pass
        self._shm = None
        self._arr = None
