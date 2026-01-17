import time
from multiprocessing import shared_memory, Process
import numpy as np
import serial
import serial.tools.list_ports
import logging
try:
    from prettytable import PrettyTable
except Exception:
    PrettyTable = None


def init_CAR_shm():
    # (X轴速度命令 Y轴速度命令 Z轴速度命令) (X轴速度 Y轴速度 Z轴速度) (X轴加速度 Y轴加速度 Z轴加速度) （X轴角速度 Y轴角速度 Z轴角速度）错误位
    a = np.zeros(13)  # Start with an existing NumPy array
    shm = shared_memory.SharedMemory(create=True, size=a.nbytes)

    # 共享的数组
    b = np.ndarray(a.shape, dtype=a.dtype, buffer=shm.buf)
    b[:] = a[:]  # 复制数据
    b[-1] = 1  # 初始化错误标识位为1，表示还没准备好
    return b, shm

def read_CAR(shape, dtype, buffer_name, COM_name):
    existing_shm = shared_memory.SharedMemory(name=buffer_name)
    # 从buffer中加载共享的numpy array
    shared_numpy = np.ndarray(shape, dtype=dtype, buffer=existing_shm.buf)
    shared_numpy[-1] = 1  # 初始化错误标识位为1，表示还没准备好

    # ====logger========
    logging.basicConfig(
        format='%(asctime)s.%(msecs)03d [%(levelname)s] [%(filename)s:%(lineno)d] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S')
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    logger.info('=======CAR准备开始=======')
    with serial.Serial(COM_name, 115200,
                       bytesize=serial.EIGHTBITS,
                       parity=serial.PARITY_NONE,
                       stopbits=serial.STOPBITS_ONE,
                       timeout=0.001) as ser:
        if not ser.isOpen():
            ser.open()
        assert ser.isOpen()  # 串口是否已打开

        ser.reset_input_buffer()
        ser.reset_output_buffer()  # 打开串口后就自动开始接收数据，先清空缓存
        temp_sum = b''  # 解码数组

        frame_len = 24  # 单条信息字节长度24
        send_period = 1.0 / 50.0  # 50HZ
        next_send_t = time.perf_counter() + send_period  # 50Hz 定时

        while True:
            start_index = 0
            # orgin_data = bytes.fromhex(ser.read(50).hex())  # 将字节转为16进制
            orgin_data = ser.read(50)
            temp_sum = temp_sum + orgin_data  # 加上之前没读完的字节
            count = len(temp_sum)

            while count > 50:
                if temp_sum[start_index] == 0x7B and temp_sum[start_index + frame_len -1] == 0x7D:
                    uwb_data = temp_sum[start_index:start_index + frame_len]
                    velocity_X, velocity_Y, velocity_Z, acceleration_X, acceleration_Y, acceleration_Z, angular_X, angular_Y, angular_Z = CAR_Handle(uwb_data)
                    shared_numpy[3:12] = [velocity_X, velocity_Y, velocity_Z, acceleration_X, acceleration_Y, acceleration_Z, angular_X, angular_Y, angular_Z]
                    shared_numpy[-1] = 0  # 移除错误位
                    start_index = start_index + 24
                    count = count - 24

                else:
                    start_index = start_index + 1
                    count = count - 1
            temp_sum = temp_sum[start_index:]

            now = time.perf_counter()
            if now >= next_send_t:
                # 从共享区/全局变量获取当前要发的命令（示例：shared_numpy 某些位置存目标）
                # 假设：shared_numpy[0:3] 存的是目标 vx, vy, wz (m/s, m/s, rad/s)，你按自己的定义替换
                command = shared_numpy[0:3].copy()
                send_msg = Command_Trans(command)
                ser.write(send_msg)

                next_send_t += send_period
                # 如果循环偶尔卡顿，避免 next_send_t 落后太多导致连续补发
                if now - next_send_t > 0.5:
                    next_send_t = now + send_period



def hex_to_int(b:bytes)->int:
    """
        输入为长度=2 的 bytes（高字节在前，低字节在后），按 16 位补码解析为有符号整数。
        例：b'\xFF\xDF' -> -33
        """
    raw = (b[0] << 8) | b[1]
    return raw - 0x10000 if (raw & 0x8000) else raw


def int_to_hex16(x: int) -> bytes:
    """
    输入：有符号整数 x
    输出：长度=2 的 bytes（高字节在前，低字节在后），按 16 位补码编码。

    范围：-32768 ~ 32767
    例：
        -33 -> b'\\xFF\\xDF'
        155 -> b'\\x00\\x9B'
    """
    if not (-0x8000 <= x <= 0x7FFF):
        raise ValueError(f"x={x} out of int16 range [-32768, 32767]")
    raw = x & 0xFFFF  # 负数会自动取补码低16位
    return bytes([(raw >> 8) & 0xFF, raw & 0xFF])


# BCC校验
def bcc_xor(data: bytes) -> int:
    """
    BCC 校验：对 data 的每个字节进行异或，返回 0~255 的整数。
    例如：0x1F = 0x7B ^ 0x00 ^ ... ^ 0x00
    """
    bcc = 0
    for x in data:
        bcc ^= x
    return bcc


def Command_Trans(command):
    X_Target, Y_Target, Z_Target = command[:3]
    # xy速度从m/s转换为mm/s，z角速度转换为rad/s，再转换为字节命令
    X_command = int_to_hex16(int(round(X_Target * 1000)))
    Y_command = int_to_hex16(int(round(Y_Target * 1000)))
    Z_command = int_to_hex16(int(round(Z_Target * 1000)))
    frame_wo_bcc = (
            bytes([0x7B, 0x00, 0x00]) +
            X_command + Y_command + Z_command
    )
    if len(frame_wo_bcc) != 9:
        raise RuntimeError("frame_wo_bcc must be 9 bytes")

    bcc = bcc_xor(frame_wo_bcc)
    return frame_wo_bcc + bytes([bcc, 0x7D])

def CAR_Handle(CAR_data):
    frame = CAR_data
    # 1. 按字节序解析原始整数（单位仍然是放大前的编码值）
    vx_raw = hex_to_int(frame[2:4])   # 第 3、4 字节
    vy_raw = hex_to_int(frame[4:6])   # 第 5、6 字节
    vz_raw = hex_to_int(frame[6:8])   # 第 7、8 字节

    ax_raw = hex_to_int(frame[8:10])   # 第 9、10 字节
    ay_raw = hex_to_int(frame[10:12])  # 第 11、12 字节
    az_raw = hex_to_int(frame[12:14])  # 第 13、14 字节

    wx_raw = hex_to_int(frame[14:16])  # 第 15、16 字节
    wy_raw = hex_to_int(frame[16:18])  # 第 17、18 字节
    wz_raw = hex_to_int(frame[18:20])  # 第 19、20 字节

    # 按协议缩放到物理单位
    # 速度：编码单位为 mm/s，除以 1000 变成 m/s
    velocity_X = vx_raw / 1000.0
    velocity_Y = vy_raw / 1000.0
    velocity_Z = vz_raw / 1000.0

    # 加速度：除以 1672 得到 m/s^2
    acceleration_X = ax_raw / 1672.0
    acceleration_Y = ay_raw / 1672.0
    acceleration_Z = az_raw / 1672.0

    # 角速度：除以 3753 得到 rad/s
    angular_X = wx_raw / 3753.0
    angular_Y = wy_raw / 3753.0
    angular_Z = wz_raw / 3753.0

    return (velocity_X, velocity_Y, velocity_Z,
            acceleration_X, acceleration_Y, acceleration_Z,
            angular_X, angular_Y, angular_Z)


if __name__ == "__main__":
    CAR_data, shm = init_CAR_shm()

    p = Process(target=read_CAR, args=(CAR_data.shape, CAR_data.dtype, shm.name, "/dev/ttyCH343USB0"))
    p.start()

    headers = [
        "X轴速度(m/s)", "Y轴速度(m/s)", "Z轴速度(m/s)",
        "X轴加速度(m/s^2)", "Y轴加速度(m/s^2)", "Z轴加速度(m/s^2)",
        "X轴角速度(rad/s)", "Y轴角速度(rad/s)", "Z轴角速度(rad/s)",
        "错误位"
    ]
    # headers = [
    #     "vx(m/s)", "vy(m/s)", "vz(m/s)",
    #     "ax(m/s^2)", "ay(m/s^2)", "az(m/s^2)",
    #     "wx(rad/s)", "wy(rad/s)", "wz(rad/s)",
    #     "err"
    # ]

    try:
        while True:
            # 1) 读取一份快照，避免打印时被并发写入造成显示跳变
            snapshot = CAR_data.copy()

            # 2) 组织 PrettyTable
            t = PrettyTable()
            t.field_names = headers
            t.add_row([f"{x:.6f}" for x in snapshot[3:12]] + [int(snapshot[-1])])

            # 3) 清屏并刷新打印（Windows / Linux / macOS 通用 ANSI）
            print("\x1b[2J\x1b[H", end="")  # 清屏+光标回到左上角
            print(t)

            time.sleep(1)
    finally:
        # 结束与清理，避免共享内存泄露
        if p.is_alive():
            p.terminate()
            p.join(timeout=1)
        shm.close()
        shm.unlink()