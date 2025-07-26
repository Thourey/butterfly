import time
import serial

class BT33:
    def __init__(self, port="/dev/ttyUSB0", baud=115200, timeout=0.01):
        self.uart = serial.Serial(port, baud, timeout=timeout)

    def cmd(self, at_str, wait=0.2):
        self.uart.write((at_str + "\r\n").encode())
        time.sleep(wait)
        lines = self.uart.readlines()
        return [l.decode(errors='ignore').strip() for l in lines]

    def send(self, data: bytes):
        self.uart.write(data)

    def recv(self, size=64) -> bytes:
        return self.uart.read(size)

    def close(self):
        self.uart.close()

# 频率设置为5Hz
FREQ = 5
INTERVAL = 1.0 / FREQ

def main():
    bt = BT33("/dev/ttyUSB0", 115200)
    log_file = open('device_log_test3.txt', 'w', encoding='utf-8')
    t_start = time.perf_counter()

    # 舵机参数设置
    center_angle_1 = 110  # 第一个舵机中心角度
    center_angle_2 = 70   # 第二个舵机中心角度
    amplitude = 45        # 摆动幅度
    step = 30             # 每次角度变化步长
    delay = 0.1          # 额外延迟（秒）

    # 初始化角度和方向
    angle_offset = 0      # 相对于中心的角度偏移
    direction = 1         # 1表示增加，-1表示减少

    seq = 0
    try:
        while True:
            t0 = time.perf_counter()

            # 计算两个舵机的角度（对称运动）
            angle1 = center_angle_1 + angle_offset
            angle2 = center_angle_2 - angle_offset  # 第二个舵机反向运动

            # 1. 发送角度控制指令
            cmd_str = f"AT+DEVICE1${angle1},{angle2}?\r\n"
            bt.send(cmd_str.encode('utf-8'))

            # 2. 接收
            rx = bt.recv(64)
            rx_str = ''
            if rx:
                try:
                    rx_str = rx.decode('utf-8', errors='ignore').strip()
                    print("RX:", rx_str)
                except Exception as e:
                    rx_str = rx.hex()
                    print("RX decode err:", e, "raw:", rx_str)

            # 3. 写日志
            elapsed_ms = int((t0 - t_start) * 1000)
            log_file.write(f"{elapsed_ms}\t{cmd_str.strip()}\t{rx_str}\n\n")
            log_file.flush()

            # 4. 更新角度偏移
            angle_offset += step * direction
            
            # 检查是否到达最大偏移，如果是则改变方向
            if abs(angle_offset) >= amplitude:
                direction *= -1
                angle_offset = amplitude * direction  # 确保不超出范围
                
            seq += 1

            # 5. 精确等待
            dt = time.perf_counter() - t0
            remaining_time = INTERVAL - dt - delay
            if remaining_time > 0:
                time.sleep(remaining_time)

    except KeyboardInterrupt:
        print("\n用户终止")
    finally:
        log_file.close()
        bt.close()

if __name__ == "__main__":
    main()