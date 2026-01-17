from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from typing import Any, Dict


def dumps(obj: Dict[str, Any]) -> bytes:
    return json.dumps(obj, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def loads(data: bytes) -> Dict[str, Any]:
    return json.loads(data.decode("utf-8"))


@dataclass
class Cmd:
    car_id: str
    seq: int
    t: float
    vx: float
    wz: float
    mode: str = "auto"

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["type"] = "cmd"
        return d


@dataclass
class Telemetry:
    car_id: str
    t: float
    seq: int
    state: Dict[str, Any]
    health: Dict[str, Any]

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["type"] = "telemetry"
        return d
