from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Any, Dict

import yaml

from car_agent.chassis.chassis_driver import ChassisDriver


def load_yaml(path: str) -> Dict[str, Any]:
    p = Path(path).expanduser().resolve()
    with p.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=str, default="car_agent/config/default.yaml")
    ap.add_argument("--vx", type=float, default=0.10)     # 线速度测试值，单位按你底盘协议
    ap.add_argument("--wz", type=float, default=0.30)     # 角速度测试值
    ap.add_argument("--t", type=float, default=1.5)       # 每段持续时间
    return ap.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_yaml(args.config)

    serial_port = str(cfg.get("chassis", {}).get("serial_port", "/dev/ttyCH343USB0"))
    baudrate = int(cfg.get("chassis", {}).get("baudrate", 115200))
    hz = float(cfg.get("chassis", {}).get("control_hz", 50.0))

    v_max = float(cfg.get("safety", {}).get("v_max", 0.3))
    w_max = float(cfg.get("safety", {}).get("w_max", 0.6))

    vx = clamp(float(args.vx), -v_max, v_max)
    wz = clamp(float(args.wz), -w_max, w_max)
    dt = 1.0 / max(1.0, hz)

    chassis = ChassisDriver(serial_port=serial_port, baudrate=baudrate, control_hz=hz)
    chassis.start()
    print(f"[test] chassis started: alive={chassis.is_alive()} serial={serial_port} hz={hz}")

    def hold_cmd(vx_: float, vy_: float, wz_: float, duration: float) -> None:
        t0 = time.time()
        last = 0.0
        while time.time() - t0 < duration:
            chassis.set_cmd(vx_, vy_, wz_)
            now = time.time()
            if now - last >= 0.2:
                st = chassis.get_state()
                print(
                    f"[telemetry] v=({st.vx:.3f},{st.vy:.3f},{st.vz:.3f}) "
                    f"a=({st.ax:.3f},{st.ay:.3f},{st.az:.3f}) "
                    f"w=({st.wx:.3f},{st.wy:.3f},{st.wz:.3f}) err={st.err} alive={chassis.is_alive()}"
                )
                last = now
            time.sleep(dt)

    try:
        print("[test] warmup 1.0s, cmd=0")
        hold_cmd(0.0, 0.0, 0.0, 1.0)

        print(f"[test] forward {args.t:.2f}s, cmd vx={vx:.3f}")
        hold_cmd(vx, 0.0, 0.0, float(args.t))

        print("[test] stop 1.0s, cmd=0")
        hold_cmd(0.0, 0.0, 0.0, 1.0)

        print(f"[test] rotate {args.t:.2f}s, cmd wz={wz:.3f}")
        hold_cmd(0.0, 0.0, wz, float(args.t))

        print("[test] stop 1.0s, cmd=0")
        hold_cmd(0.0, 0.0, 0.0, 1.0)

    finally:
        chassis.set_cmd(0.0, 0.0, 0.0)
        time.sleep(0.1)
        chassis.stop()
        print("[test] done")


if __name__ == "__main__":
    main()
