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

FREQ = 5
INTERVAL = 1.0 / FREQ

def main():
    bt = BT33("/dev/ttyUSB0", 115200)

    # ========= 新增：日志文件 =========
    log_file = open('device_log.txt', 'w', encoding='utf-8')
    t_start = time.perf_counter()  # 计时起点

    seq = 0
    try:
        while True:
            t0 = time.perf_counter()

            # 1. 发送
            cmd_str = "AT+DEVICE1$130,50?\r\n"
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

            # 3. 写日志：时间(ms)  TX  RX
            elapsed_ms = int((t0 - t_start) * 1000)
            log_file.write(f"{elapsed_ms}\t{cmd_str.strip()}\t{rx_str}\n\n")
            log_file.flush()      # 立即落盘，防止异常丢失

            seq += 1

            # 4. 精确等待
            dt = time.perf_counter() - t0
            if dt < INTERVAL:
                time.sleep(INTERVAL - dt)

    except KeyboardInterrupt:
        print("\n用户终止")
    finally:
        log_file.close()
        bt.close()

if __name__ == "__main__":
    main()