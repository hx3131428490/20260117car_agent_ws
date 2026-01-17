from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


SHM_LEN: int = 13

CMD_SLICE = slice(0, 3)      # vx, vy, wz command
STATE_SLICE = slice(3, 12)   # vx, vy, vz, ax, ay, az, wx, wy, wz
ERR_IDX: int = 12            # error flag


@dataclass(frozen=True)
class ShmLayout:
    length: int = SHM_LEN
    cmd_slice: slice = CMD_SLICE
    state_slice: slice = STATE_SLICE
    err_idx: int = ERR_IDX

    @property
    def cmd_dim(self) -> int:
        return self.cmd_slice.stop - self.cmd_slice.start

    @property
    def state_dim(self) -> int:
        return self.state_slice.stop - self.state_slice.start
