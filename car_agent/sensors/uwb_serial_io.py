from __future__ import annotations

import time
from multiprocessing import shared_memory
import numpy as np
import serial
import logging


def init_UWB_shm():
    # 布局: x, y, vx, vy, stamp, err
    a = np.zeros(6, dtype=np.float64)
    shm = shared_memory.SharedMemory(create=True, size=a.nbytes)
    b = np.ndarray(a.shape, dtype=a.dtype, buffer=shm.buf)
    b[:] = a[:]
    b[-1] = 1.0  # err=1 表示未准备好
    return b, shm


def hex_to_int24(hex_byte: bytes) -> int:
    hex_str = hex_byte.hex()
    big_endian_hex = hex_str[4:6] + hex_str[2:4] + hex_str[0:2]
    num = int(big_endian_hex, 16)
    if num & 0x800000:
        num -= 0x1000000
    return num


def Uwb_Handle(frame: bytes):
    pos_X = hex_to_int24(frame[4:7]) / 1000.0
    pos_Y = hex_to_int24(frame[7:10]) / 1000.0
    velocity_X = hex_to_int24(frame[13:16]) / 10000.0
    velocity_Y = hex_to_int24(frame[16:19]) / 10000.0
    return pos_X, pos_Y, velocity_X, velocity_Y


def read_UWB(shape, dtype, buffer_name: str, COM_name: str = "/dev/ttyCH343USB1", baudrate: int = 921600):
    existing_shm = shared_memory.SharedMemory(name=buffer_name)
    shared_numpy = np.ndarray(shape, dtype=dtype, buffer=existing_shm.buf)
    shared_numpy[-1] = 1.0

    logging.basicConfig(
        format="%(asctime)s.%(msecs)03d [%(levelname)s] [%(filename)s:%(lineno)d] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    logger.info("=======UWB准备开始=======")

    try:
        with serial.Serial(
            COM_name,
            baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.001,
        ) as ser:
            if not ser.isOpen():
                ser.open()
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            temp_sum = b""

            while True:
                orgin_data = bytes.fromhex(ser.read(150).hex())
                temp_sum = temp_sum + orgin_data

                start_index = 0
                count = len(temp_sum)

                while count > 200:
                    if temp_sum[start_index] == 0x55 and temp_sum[start_index + 1] == 0x01:
                        frame = temp_sum[start_index : start_index + 128]
                        pos_X, pos_Y, vx, vy = Uwb_Handle(frame)

                        now = time.time()
                        shared_numpy[0] = float(pos_X)
                        shared_numpy[1] = float(pos_Y)
                        shared_numpy[2] = float(vx)
                        shared_numpy[3] = float(vy)
                        shared_numpy[4] = float(now)
                        shared_numpy[5] = 0.0  # err=0

                        start_index += 128
                        count -= 128
                    else:
                        start_index += 1
                        count -= 1

                temp_sum = temp_sum[start_index:]

    except Exception as e:
        logger.error(f"UWB serial loop crashed: {e}")
        try:
            shared_numpy[5] = 1.0
        except Exception:
            pass
        raise
