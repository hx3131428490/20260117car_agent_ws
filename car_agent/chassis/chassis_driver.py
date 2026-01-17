from __future__ import annotations

import time
from dataclasses import dataclass
from multiprocessing import Process, shared_memory
from typing import Optional, Tuple

import numpy as np

from .shm_layout import ShmLayout
from . import wheeltec_serial_io


@dataclass
class ChassisState:
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    wx: float = 0.0
    wy: float = 0.0
    wz: float = 0.0
    err: int = 0
    stamp: float = 0.0


class ChassisDriver:
    def __init__(self, serial_port: str, baudrate: int = 115200, control_hz: float = 50.0) -> None:
        self.serial_port = serial_port
        self.baudrate = int(baudrate)
        self.control_hz = float(control_hz)

        self.layout = ShmLayout()
        self._shm: Optional[shared_memory.SharedMemory] = None
        self._arr: Optional[np.ndarray] = None
        self._proc: Optional[Process] = None

        self._last_cmd_ts: float = 0.0

    def start(self) -> None:
        if self._proc is not None and self._proc.is_alive():
            return

        arr, shm = wheeltec_serial_io.init_CAR_shm()
        self._arr = arr
        self._shm = shm

        self._proc = Process(
            target=wheeltec_serial_io.read_CAR,
            args=(self._arr.shape, self._arr.dtype, self._shm.name, self.serial_port),
            daemon=True,
        )
        self._proc.start()

    def is_alive(self) -> bool:
        return self._proc is not None and self._proc.is_alive()

    def set_cmd(self, vx: float, vy: float, wz: float) -> None:
        if self._arr is None:
            return
        if abs(vy) > 1e-4:
            now = time.time()
            if now - self._vy_warn_last > 2.0:
                print(f"[chassis] vy command ignored on differential drive: vy={vy:.4f}")
                self._vy_warn_last = now
        self._arr[self.layout.cmd_slice] = [float(vx), 0.0, float(wz)]
        self._last_cmd_ts = time.time()

    def get_state(self) -> ChassisState:
        if self._arr is None:
            return ChassisState(err=1, stamp=time.time())

        s = self._arr.copy()
        vx, vy, vz, ax, ay, az, wx, wy, wz = [float(x) for x in s[self.layout.state_slice]]
        err = int(s[self.layout.err_idx])

        if not self.is_alive():
            err = 1

        return ChassisState(
            vx=vx, vy=vy, vz=vz,
            ax=ax, ay=ay, az=az,
            wx=wx, wy=wy, wz=wz,
            err=err,
            stamp=time.time(),
        )

    def stop(self) -> None:
        if self._proc is not None and self._proc.is_alive():
            self._proc.terminate()
            self._proc.join(timeout=1)

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
