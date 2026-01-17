from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import yaml

from car_agent.chassis.chassis_driver import ChassisDriver
from car_agent.net.cmd_server import UdpCmdServer
from car_agent.net.protocol import Telemetry
from car_agent.net.telemetry_server import UdpTelemetryClient
from car_agent.safety.limits import clamp
from car_agent.sensors.uwb_adapter import UwbAdapter


@dataclass
class AppConfig:
    raw: Dict[str, Any]

    @property
    def car_id(self) -> str:
        return str(self.raw.get("car_id", "car_unknown"))

    @property
    def cmd_listen(self) -> str:
        return str(self.raw.get("net", {}).get("cmd_listen", "0.0.0.0:31001"))

    @property
    def telemetry_peer(self) -> str:
        return str(self.raw.get("net", {}).get("telemetry_peer", "192.168.10.1:32001"))

    @property
    def control_hz(self) -> float:
        return float(self.raw.get("loop", {}).get("control_hz", 50.0))

    @property
    def telemetry_hz(self) -> float:
        return float(self.raw.get("loop", {}).get("telemetry_hz", 20.0))

    @property
    def chassis_serial(self) -> str:
        return str(self.raw.get("chassis", {}).get("serial_port", ""))

    @property
    def chassis_baudrate(self) -> int:
        return int(self.raw.get("chassis", {}).get("baudrate", 115200))

    @property
    def chassis_hz(self) -> float:
        return float(self.raw.get("chassis", {}).get("control_hz", 50))

    @property
    def cmd_timeout_s(self) -> float:
        return float(self.raw.get("safety", {}).get("cmd_timeout_s", 0.2))

    @property
    def v_max(self) -> float:
        return float(self.raw.get("safety", {}).get("v_max", 0.3))

    @property
    def w_max(self) -> float:
        return float(self.raw.get("safety", {}).get("w_max", 0.6))

    @property
    def uwb_enabled(self) -> bool:
        return bool(self.raw.get("sensors", {}).get("uwb", {}).get("enabled", False))

    @property
    def uwb_port(self) -> str:
        return str(self.raw.get("sensors", {}).get("uwb", {}).get("serial_port", "/dev/ttyCH343USB1"))

    @property
    def uwb_baudrate(self) -> int:
        return int(self.raw.get("sensors", {}).get("uwb", {}).get("baudrate", 921600))



def load_config(path: str) -> AppConfig:
    p = Path(path).expanduser().resolve()
    with p.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return AppConfig(raw=data)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=str, default="car_agent/config/default.yaml")
    return ap.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_config(args.config)
    print(f"[car_agent] boot ok, car_id={cfg.car_id}, config={args.config}")

    chassis = ChassisDriver(
        serial_port=cfg.chassis_serial,
        baudrate=cfg.chassis_baudrate,
        control_hz=cfg.chassis_hz,
    )
    chassis.start()
    print(f"[car_agent] chassis started, alive={chassis.is_alive()}, serial={cfg.chassis_serial}")

    cmd_server = UdpCmdServer(car_id=cfg.car_id, listen=cfg.cmd_listen)
    cmd_server.start()
    print(f"[car_agent] cmd server listen={cfg.cmd_listen}")

    uwb = None
    if cfg.uwb_enabled:
        uwb = UwbAdapter(serial_port=cfg.uwb_port, baudrate=cfg.uwb_baudrate)
        uwb.start()
        print(f"[car_agent] uwb started, alive={uwb.is_alive()}, serial={cfg.uwb_port}")


    telem = UdpTelemetryClient(peer=cfg.telemetry_peer)
    print(f"[car_agent] telemetry peer={cfg.telemetry_peer}")

    dt = 1.0 / max(1.0, cfg.control_hz)
    telem_dt = 1.0 / max(1.0, cfg.telemetry_hz)

    last_telem = 0.0
    last_print = 0.0

    try:
        while True:
            now = time.time()
            cmd = cmd_server.get_latest()

            stale = (now - cmd.rx_time) > cfg.cmd_timeout_s
            if stale:
                vx_cmd = 0.0
                wz_cmd = 0.0
                mode = "idle"
            else:
                vx_cmd = clamp(cmd.vx, -cfg.v_max, cfg.v_max)
                wz_cmd = clamp(cmd.wz, -cfg.w_max, cfg.w_max)
                mode = cmd.mode

            chassis.set_cmd(vx_cmd, 0.0, wz_cmd)

            if now - last_telem >= telem_dt:
                st = chassis.get_state()

                uwb_state = None
                if uwb is not None:
                    u = uwb.get_latest()
                    uwb_state = {
                        "x": u.x, "y": u.y,
                        "vx": u.vx, "vy": u.vy,
                        "age_s": u.rx_age_s,
                        "err": u.err,
                        "alive": uwb.is_alive(),
                    }

                pkt = Telemetry(
                    car_id=cfg.car_id,
                    t=now,
                    seq=cmd.seq,
                    state={
                        "vx": st.vx, "vy": st.vy, "vz": st.vz,
                        "ax": st.ax, "ay": st.ay, "az": st.az,
                        "wx": st.wx, "wy": st.wy, "wz": st.wz,
                        "err": st.err,
                        "uwb": uwb_state,
                    },
                    health={
                        "alive": chassis.is_alive(),
                        "cmd_rx_count": cmd_server.rx_count,
                        "cmd_parse_err": cmd_server.parse_err,
                        "cmd_stale": stale,
                        "mode": mode,
                    },
                )
                telem.send(pkt.to_dict())
                last_telem = now

            if now - last_print >= 1.0:
                st = chassis.get_state()
                print(
                    f"[status] mode={mode} stale={stale} "
                    f"cmd(vx={vx_cmd:.3f},wz={wz_cmd:.3f}) "
                    f"state(vx={st.vx:.3f},wz={st.wz:.3f}) "
                    f"rx={cmd_server.rx_count} err={cmd_server.parse_err}"
                )
                last_print = now

            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        chassis.set_cmd(0.0, 0.0, 0.0)
        time.sleep(0.1)
        cmd_server.stop()
        telem.close()
        chassis.stop()
        if uwb is not None:
            uwb.stop()
        print("[car_agent] shutdown ok")


if __name__ == "__main__":
    main()
