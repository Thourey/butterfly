# # import time
# # import serial
# # import re
# # import math

# # # ============================================================
# # # 飞行控制参数 (核心：这些值需要通过实验来调整)
# # # ============================================================
# # # --- 物理扇动参数 ---
# # FLAP_FREQUENCY = 5.0  # 翅膀每秒扇动的次数 (Hz)
# # FLAP_CENTER = 90.0    # 翅膀扇动的中心角度 (0-180度)
# # BASE_AMPLITUDE = 45.0 # 基础扇动振幅 (翅膀会从 90-45=45度 摆到 90+45=135度)

# # # --- PID 参数 ---
# # # 横滚角 (Roll) PID 参数
# # ROLL_KP = 0.8
# # ROLL_KI = 0.1
# # ROLL_KD = 0.3

# # # 航向角 (Yaw) PID 参数
# # YAW_KP = 0.5
# # YAW_KI = 0.05
# # YAW_KD = 0.2

# # # --- 目标姿态 ---
# # TARGET_ROLL = 0.0
# # TARGET_YAW = 0.0
# # # ============================================================

# # # 全局变量，用于存储解析后的姿态角数据
# # pitch = 0.0
# # roll = 0.0
# # yaw = 0.0

# # def clamp(value, min_val, max_val):
# #     """将一个值限制在min和max之间"""
# #     return max(min_val, min(value, max_val))

# # class PID:
# #     """一个简单的PID控制器实现"""
# #     def __init__(self, Kp, Ki, Kd, setpoint=0):
# #         self.Kp = Kp
# #         self.Ki = Ki
# #         self.Kd = Kd
# #         self.setpoint = setpoint
# #         self.last_error = 0
# #         self.integral = 0
# #         self.last_time = time.perf_counter()

# #     def compute(self, current_value):
# #         """计算PID输出"""
# #         current_time = time.perf_counter()
# #         dt = current_time - self.last_time
# #         if dt <= 0: return 0

# #         error = self.setpoint - current_value
# #         self.integral += error * dt
# #         self.integral = clamp(self.integral, -50, 50) # 限制积分饱和

# #         derivative = (error - self.last_error) / dt
# #         output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

# #         self.last_error = error
# #         self.last_time = current_time
# #         return output

# # class BT33:
# #     def __init__(self, port="/dev/ttyUSB0", baud=115200, timeout=1):
# #         try:
# #             self.uart = serial.Serial(port, baud, timeout=timeout)
# #         except serial.SerialException as e:
# #             print(f"错误：无法打开串口 {port}。请检查设备连接或端口号是否正确。")
# #             print(f"详细信息: {e}")
# #             exit()

# #     def send(self, data: bytes):
# #         if self.uart and self.uart.is_open: self.uart.write(data)

# #     def recv(self, size=128) -> bytes:
# #         return self.uart.read(size)

# #     def close(self):
# #         if self.uart and self.uart.is_open: self.uart.close()

# # # 控制循环频率
# # CONTROL_FREQ = 50 # 提高控制频率以获得更平滑的波形
# # INTERVAL = 1.0 / CONTROL_FREQ

# # def main():
# #     global pitch, roll, yaw

# #     roll_pid = PID(ROLL_KP, ROLL_KI, ROLL_KD, setpoint=TARGET_ROLL)
# #     yaw_pid = PID(YAW_KP, YAW_KI, YAW_KD, setpoint=TARGET_YAW)

# #     bt = BT33("/dev/ttyUSB0", 115200)
# #     log_file = open('device_log.txt', 'w', encoding='utf-8')
# #     t_start = time.perf_counter()
# #     cycle_time = 0.0 # 用于驱动正弦波的时间变量

# #     # 初始化修正量，它们的值会一直被使用，直到被新的传感器数据更新
# #     roll_correction = 0.0
# #     yaw_correction = 0.0

# #     try:
# #         while True:
# #             t0 = time.perf_counter()
# #             cycle_time += INTERVAL # 更新周期时间

# #             # 1. 基于上一次的修正量，计算两侧翅膀的扇动振幅
# #             left_amplitude = BASE_AMPLITUDE - roll_correction - yaw_correction
# #             right_amplitude = BASE_AMPLITUDE + roll_correction + yaw_correction
# #             left_amplitude = clamp(left_amplitude, 0, 90)
# #             right_amplitude = clamp(right_amplitude, 0, 90)

# #             # 2. 根据正弦波模型计算当前时刻的瞬时目标角度
# #             wave_pos = math.sin(2 * math.pi * FLAP_FREQUENCY * cycle_time)
# #             servo1_angle = FLAP_CENTER + left_amplitude * wave_pos
# #             servo2_angle = FLAP_CENTER + right_amplitude * wave_pos
# #             servo1_angle = clamp(servo1_angle, 0, 180)
# #             servo2_angle = clamp(servo2_angle, 0, 180)

# #             # 3. 生成并发送指令
# #             cmd_str = f"AT+DEVICE1${int(servo1_angle)},{int(servo2_angle)}?\r\n"
# #             bt.send(cmd_str.encode('utf-8'))

# #             # 4. 尝试接收和解析姿态数据，以更新修正量供下一次循环使用
# #             rx = bt.recv(128)
# #             rx_str = ''
# #             if rx:
# #                 try:
# #                     rx_str = rx.decode('utf-8', errors='ignore').strip()
# #                     # 新增：只要收到非空数据就打印
# #                     if rx_str:
# #                         print(f"RX: '{rx_str}'")
# #                         match = re.search(r"p(-?\d+\.?\d*),r(-?\d+\.?\d*),y(-?\d+\.?\d*)", rx_str)
# #                         if match:
# #                             # 新增：明确打印解析成功
# #                             print("--- 解析成功 ---")
# #                             pitch = float(match.group(1))
# #                             roll = float(match.group(2))
# #                             yaw = float(match.group(3))

# #                             # 关键：只有在收到有效数据时，才更新PID修正量
# #                             roll_correction = roll_pid.compute(roll)
# #                             yaw_correction = yaw_pid.compute(yaw)
# #                         else:
# #                             # 新增：明确打印格式错误
# #                             print("--- 格式错误，无法解析 ---")

# #                 except Exception as e:
# #                     print(f"数据解析错误: {e}")

# #             # 5. 打印状态和日志记录
# #             print(f"TX: {cmd_str.strip()} | RollErr: {roll:.1f}, YawErr: {yaw:.1f} | L/R Amp: {left_amplitude:.1f}/{right_amplitude:.1f}")
# #             elapsed_ms = int((t0 - t_start) * 1000)
# #             # 在日志中记录原始接收数据，无论是否解析成功
# #             log_file.write(f"{elapsed_ms}\t{cmd_str.strip()}\t{rx_str}\n")
# #             log_file.flush()

# #             # 6. 精确延时
# #             dt = time.perf_counter() - t0
# #             if dt < INTERVAL:
# #                 time.sleep(INTERVAL - dt)

# #     except KeyboardInterrupt:
# #         print("\n用户终止程序")
# #     finally:
# #         print("正在关闭资源...")
# #         # 发送指令让舵机归中
# #         bt.send(f"AT+DEVICE1${int(FLAP_CENTER)},{int(FLAP_CENTER)}?\r\n".encode('utf-8'))
# #         time.sleep(0.1)
# #         log_file.close()
# #         bt.close()
# #         print("已关闭。")

# # if __name__ == "__main__":
# #     main()


# import time
# import serial
# import re
# import math

# # ============================================================
# # 飞行控制参数 (核心：这些值需要通过实验来调整)
# # ============================================================
# # --- 物理扇动参数 ---
# FLAP_FREQUENCY = 5.0  # 翅膀每秒扇动的次数 (Hz)
# FLAP_CENTER = 90.0    # 翅膀扇动的中心角度 (0-180度)
# BASE_AMPLITUDE = 45.0 # 基础扇动振幅 (翅膀会从 90-45=45度 摆到 90+45=135度)

# # --- PID 参数 ---
# # 注意：如果姿态误差过大 (例如刚启动时，roll读数是170多度)，PID的修正量可能会非常大，
# # 这会导致一侧翅膀的振幅被“限制”在最大(90)或最小(0)，而另一侧正常摆动。
# # 这是正常现象，表示控制器正在尽力修正巨大误差。
# # 稳定飞行的关键是微调下面的Kp, Ki, Kd参数，找到一个既能快速响应又不过度震荡的平衡点。
# # 横滚角 (Roll) PID 参数
# ROLL_KP = 0.8
# ROLL_KI = 0.1
# ROLL_KD = 0.3

# # 航向角 (Yaw) PID 参数
# YAW_KP = 0.5
# YAW_KI = 0.05
# YAW_KD = 0.2

# # --- 目标姿态 ---
# TARGET_ROLL = 151.0
# TARGET_YAW = 10.0 # 目标航向角 (0-360度)
# # ============================================================

# # 全局变量，用于存储解析后的姿态角数据
# pitch = 0.0
# roll = 0.0
# yaw = 0.0

# def clamp(value, min_val, max_val):
#     """将一个值限制在min和max之间"""
#     return max(min_val, min(value, max_val))

# class PID:
#     """一个简单的PID控制器实现"""
#     def __init__(self, Kp, Ki, Kd, setpoint=0, is_angular=False):
#         self.Kp = Kp
#         self.Ki = Ki
#         self.Kd = Kd
#         self.setpoint = setpoint
#         self.last_error = 0
#         self.integral = 0
#         self.last_time = time.perf_counter()
#         # 新增：用于判断是否需要处理角度环绕问题
#         self.is_angular = is_angular

#     def compute(self, current_value):
#         """计算PID输出"""
#         current_time = time.perf_counter()
#         dt = current_time - self.last_time
#         if dt <= 0: return 0

#         # =========================================================================
#         # 关键修改：处理角度环绕问题 (例如从358度到2度，误差是+4度而不是-356度)
#         # =========================================================================
#         if self.is_angular:
#             error = self.setpoint - current_value
#             if error > 180:
#                 error -= 360
#             elif error < -180:
#                 error += 360
#         else:
#             error = self.setpoint - current_value
        
#         self.integral += error * dt
#         self.integral = clamp(self.integral, -50, 50) # 限制积分饱和

#         derivative = (error - self.last_error) / dt
#         output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

#         self.last_error = error
#         self.last_time = current_time
#         return output

# class BT33:
#     def __init__(self, port="/dev/ttyUSB0", baud=115200, timeout=0):
#         """
#         初始化串口
#         关键修改：将timeout设置为0，使read()操作变为非阻塞，避免拖慢主循环
#         """
#         try:
#             self.uart = serial.Serial(port, baud, timeout=timeout)
#         except serial.SerialException as e:
#             print(f"错误：无法打开串口 {port}。请检查设备连接或端口号是否正确。")
#             print(f"详细信息: {e}")
#             exit()

#     def send(self, data: bytes):
#         if self.uart and self.uart.is_open: self.uart.write(data)

#     def recv(self, size=128) -> bytes:
#         return self.uart.read(size)

#     def close(self):
#         if self.uart and self.uart.is_open: self.uart.close()

# # 控制循环频率
# CONTROL_FREQ = 5 # 提高控制频率以获得更平滑的波形
# INTERVAL = 1.0 / CONTROL_FREQ

# def main():
#     global pitch, roll, yaw

#     roll_pid = PID(ROLL_KP, ROLL_KI, ROLL_KD, setpoint=TARGET_ROLL)
#     # 关键修改：初始化Yaw的PID时，指明它是一个角度控制器
#     yaw_pid = PID(YAW_KP, YAW_KI, YAW_KD, setpoint=TARGET_YAW, is_angular=True)

#     bt = BT33("/dev/ttyUSB0", 115200)
#     log_file = open('device_log.txt', 'w', encoding='utf-8')
#     t_start = time.perf_counter()
#     cycle_time = 0.0 # 用于驱动正弦波的时间变量

#     # 初始化修正量，它们的值会一直被使用，直到被新的传感器数据更新
#     roll_correction = 0.0
#     yaw_correction = 0.0

#     try:
#         while True:
#             t0 = time.perf_counter()
#             cycle_time += INTERVAL # 更新周期时间

#             # 1. 基于上一次的修正量，计算两侧翅膀的扇动振幅
#             left_amplitude = BASE_AMPLITUDE - roll_correction - yaw_correction
#             right_amplitude = BASE_AMPLITUDE + roll_correction + yaw_correction
#             left_amplitude = clamp(left_amplitude, 0, 90)
#             right_amplitude = clamp(right_amplitude, 0, 90)

#             # 2. 根据正弦波模型计算当前时刻的瞬时目标角度
#             wave_pos = math.sin(2 * math.pi * FLAP_FREQUENCY * cycle_time)
#             servo1_angle = FLAP_CENTER + left_amplitude * wave_pos
#             servo2_angle = FLAP_CENTER - right_amplitude * wave_pos
            
#             # 确保最终角度在舵机物理极限内
#             servo1_angle = clamp(servo1_angle, 0, 180)
#             servo2_angle = clamp(servo2_angle, 0, 180)

#             # 3. 生成并发送指令 (此操作现在严格以50Hz执行)
#             cmd_str = f"AT+DEVICE1${int(servo1_angle)},{int(servo2_angle)}?\r\n"
#             bt.send(cmd_str.encode('utf-8'))

#             # 4. 尝试接收和解析姿态数据 (此操作不再阻塞循环)
#             rx = bt.recv(128)
#             rx_str = ''
#             if rx:
#                 try:
#                     rx_str = rx.decode('utf-8', errors='ignore').strip()
#                     if rx_str:
#                         print(f"RX: '{rx_str}'")
#                         match = re.search(r"p(-?\d+\.?\d*),r(-?\d+\.?\d*),y(-?\d+\.?\d*)", rx_str)
#                         if match:
#                             print("--- 解析成功 ---")
#                             pitch = float(match.group(1))
#                             roll = float(match.group(2))
#                             yaw = float(match.group(3))

#                             # 关键：只有在收到有效数据时，才更新PID修正量
#                             roll_correction = roll_pid.compute(roll)
#                             yaw_correction = yaw_pid.compute(yaw)
#                         else:
#                             print("--- 格式错误，无法解析 ---")

#                 except Exception as e:
#                     print(f"数据解析错误: {e}")

#             # 5. 增强的打印和日志记录
#             left_lower_angle = FLAP_CENTER - left_amplitude
#             left_upper_angle = FLAP_CENTER + left_amplitude
#             right_lower_angle = FLAP_CENTER - right_amplitude
#             right_upper_angle = FLAP_CENTER + right_amplitude

#             print("-" * 60)
#             print(f"发送指令: {cmd_str.strip()}")
#             print(f"  左翼: 幅度={left_amplitude:.1f}, 范围=[{left_lower_angle:.1f}, {left_upper_angle:.1f}]")
#             print(f"  右翼: 幅度={right_amplitude:.1f}, 范围=[{right_lower_angle:.1f}, {right_upper_angle:.1f}]")
#             print(f"  姿态: Roll={roll:.1f}, Yaw={yaw:.1f} (目标: R=0, Y=0)")
#             print("-" * 60)
            
#             elapsed_ms = int((t0 - t_start) * 1000)
#             log_file.write(f"{elapsed_ms}\t{cmd_str.strip()}\t{rx_str}\n")
#             log_file.flush()

#             # 6. 精确延时
#             dt = time.perf_counter() - t0
#             if dt < INTERVAL:
#                 time.sleep(INTERVAL - dt)

#     except KeyboardInterrupt:
#         print("\n用户终止程序")
#     finally:
#         print("正在关闭资源...")
#         # 发送指令让舵机归中
#         bt.send(f"AT+DEVICE1${int(FLAP_CENTER)},{int(FLAP_CENTER)}?\r\n".encode('utf-8'))
#         time.sleep(0.1)
#         log_file.close()
#         bt.close()
#         print("已关闭。")

# if __name__ == "__main__":
#     main()




import time
import serial
import re
import math

# ============================================================
# 飞行控制参数 (核心：这些值需要通过实验来调整)
# ============================================================
# --- 物理扇动参数 ---
FLAP_FREQUENCY = 2.0  # 翅膀每秒扇动的次数 (Hz)
FLAP_CENTER = 90.0    # 翅膀扇动的中心角度 (0-180度)
BASE_AMPLITUDE = 45.0 # 基础扇动振幅 (翅膀会从 90-45=45度 摆到 90+45=135度)

# --- PID 参数 ---
# 注意：如果姿态误差过大 (例如刚启动时，roll读数是170多度)，PID的修正量可能会非常大，
# 这会导致一侧翅膀的振幅被“限制”在最大(90)或最小(0)，而另一侧正常摆动。


# 这是正常现象，表示控制器正在尽力修正巨大误差。
# 稳定飞行的关键是微调下面的Kp, Ki, Kd参数，找到一个既能快速响应又不过度震荡的平衡点。
# 横滚角 (Roll) PID 参数
ROLL_KP = 0.08
ROLL_KI = 0.0
ROLL_KD = 0.5

# 航向角 (Yaw) PID 参数
YAW_KP = 0.5
YAW_KI = 0.05
YAW_KD = 0.2

# --- 目标姿态 ---
TARGET_ROLL = 0.0
TARGET_YAW = 0.0 # 目标航向角 (0-360度)
# ============================================================

# 全局变量，用于存储解析后的姿态角数据
pitch = 0.0
roll = 0.0
yaw = 0.0

def clamp(value, min_val, max_val):
    """将一个值限制在min和max之间"""
    return max(min_val, min(value, max_val))

class PID:
    """一个简单的PID控制器实现"""
    def __init__(self, Kp, Ki, Kd, setpoint=0, is_angular=False):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.last_error = 0
        self.integral = 0
        self.last_time = time.perf_counter()
        # 新增：用于判断是否需要处理角度环绕问题
        self.is_angular = is_angular

    def compute(self, current_value):
        """计算PID输出"""
        current_time = time.perf_counter()
        dt = current_time - self.last_time
        if dt <= 0: return 0

        # =========================================================================
        # 关键修改：处理角度环绕问题 (例如从358度到2度，误差是+4度而不是-356度)
        # =========================================================================
        if self.is_angular:
            error = self.setpoint - current_value
            if error > 180:
                error -= 360
            elif error < -180:
                error += 360
        else:
            error = self.setpoint - current_value
        
        self.integral += error * dt
        self.integral = clamp(self.integral, -50, 50) # 限制积分饱和

        derivative = (error - self.last_error) / dt
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        self.last_error = error
        self.last_time = current_time
        return output

class BT33:
    def __init__(self, port="/dev/ttyUSB0", baud=115200, timeout=0):
        """
        初始化串口
        关键修改：将timeout设置为0，使read()操作变为非阻塞，避免拖慢主循环
        """
        try:
            self.uart = serial.Serial(port, baud, timeout=timeout)
        except serial.SerialException as e:
            print(f"错误：无法打开串口 {port}。请检查设备连接或端口号是否正确。")
            print(f"详细信息: {e}")
            exit()

    def send(self, data: bytes):
        if self.uart and self.uart.is_open: self.uart.write(data)

    def recv(self, size=128) -> bytes:
        return self.uart.read(size)

    def close(self):
        if self.uart and self.uart.is_open: self.uart.close()

# 控制循环频率
CONTROL_FREQ = 20 # 提高控制频率以获得更平滑的波形
INTERVAL = 1.0 / CONTROL_FREQ

def main():
    global pitch, roll, yaw

    roll_pid = PID(ROLL_KP, ROLL_KI, ROLL_KD, setpoint=TARGET_ROLL)
    # 关键修改：初始化Yaw的PID时，指明它是一个角度控制器
    yaw_pid = PID(YAW_KP, YAW_KI, YAW_KD, setpoint=TARGET_YAW, is_angular=True)

    bt = BT33("/dev/ttyUSB0", 115200)
    log_file = open('device_log.txt', 'w', encoding='utf-8')
    t_start = time.perf_counter()
    cycle_time = 0.0 # 用于驱动正弦波的时间变量

    # 初始化修正量，它们的值会一直被使用，直到被新的传感器数据更新
    roll_correction = 0.0
    yaw_correction = 0.0

    try:
        while True:
            t0 = time.perf_counter()
            cycle_time += INTERVAL # 更新周期时间

            # 1. 基于上一次的修正量，计算两侧翅膀的扇动振幅
            left_amplitude = BASE_AMPLITUDE - roll_correction #- yaw_correction
            right_amplitude = BASE_AMPLITUDE + roll_correction #+ yaw_correction
            left_amplitude = clamp(left_amplitude, 0, 90)
            right_amplitude = clamp(right_amplitude, 0, 90)

            # 2. 根据正弦波模型计算当前时刻的瞬时目标角度
            wave_pos = math.sin(2 * math.pi * FLAP_FREQUENCY * cycle_time)
            servo1_angle = FLAP_CENTER + left_amplitude * wave_pos
            servo2_angle = FLAP_CENTER - right_amplitude * wave_pos
            
            # 确保最终角度在舵机物理极限内
            servo1_angle = clamp(servo1_angle, 0, 180)
            servo2_angle = clamp(servo2_angle, 0, 180)

            # 3. 生成并发送指令 (此操作现在严格以50Hz执行)
            cmd_str = f"AT+DEVICE1${int(servo1_angle)},{int(servo2_angle)}?\r\n"
            bt.send(cmd_str.encode('utf-8'))

            # 4. 尝试接收和解析姿态数据 (此操作不再阻塞循环)
            rx = bt.recv(128)
            rx_str = ''
            if rx:
                try:
                    rx_str = rx.decode('utf-8', errors='ignore').strip()
                    if rx_str:
                        print(f"RX: '{rx_str}'")
                        match = re.search(r"p(-?\d+\.?\d*),r(-?\d+\.?\d*),y(-?\d+\.?\d*)", rx_str)
                        if match:
                            print("--- 解析成功 ---")
                            # =========================================================================
                            # 关键修改：IMU装反了，交换pitch和roll的值
                            # 传感器数据中的 'p' 实际上是 roll, 'r' 实际上是 pitch
                            # =========================================================================
                            parsed_p_as_roll = float(match.group(1))
                            parsed_r_as_pitch = float(match.group(2))
                            
                            roll = parsed_p_as_roll
                            pitch = parsed_r_as_pitch
                            yaw = float(match.group(3))

                            # PID计算使用交换后正确的 'roll' 值
                            roll_correction = roll_pid.compute(roll)
                            yaw_correction = yaw_pid.compute(yaw)
                        else:
                            print("--- 格式错误，进入离线安全模式 ---")
                            # 关键修改：恢复到基础扇动模式
                            roll_correction = 0.0
                            yaw_correction = 0.0
                except Exception as e:
                    print(f"数据解析错误: {e}, 进入离线安全模式")
                    # 关键修改：恢复到基础扇动模式
                    roll_correction = 0.0
                    yaw_correction = 0.0

            # 5. 增强的打印和日志记录
            left_lower_angle = FLAP_CENTER - left_amplitude
            left_upper_angle = FLAP_CENTER + left_amplitude
            right_lower_angle = FLAP_CENTER - right_amplitude
            right_upper_angle = FLAP_CENTER + right_amplitude

            print("-" * 60)
            print(f"发送指令: {cmd_str.strip()}")
            print(f"  左翼: 幅度={left_amplitude:.1f}, 范围=[{left_lower_angle:.1f}, {left_upper_angle:.1f}]")
            print(f"  右翼: 幅度={right_amplitude:.1f}, 范围=[{right_lower_angle:.1f}, {right_upper_angle:.1f}]")
            print(f"  姿态: Roll={roll:.1f}, Pitch={pitch:.1f}, Yaw={yaw:.1f} (目标: R=0, Y=0)")
            print("-" * 60)
            
            elapsed_ms = int((t0 - t_start) * 1000)
            log_file.write(f"{elapsed_ms}\t{cmd_str.strip()}\t{rx_str}\n")
            log_file.flush()

            # 6. 精确延时
            dt = time.perf_counter() - t0
            if dt < INTERVAL:
                time.sleep(INTERVAL - dt)

    except KeyboardInterrupt:
        print("\n用户终止程序")
    finally:
        print("正在关闭资源...")
        # 发送指令让舵机归中
        bt.send(f"AT+DEVICE1${int(FLAP_CENTER)},{int(FLAP_CENTER)}?\r\n".encode('utf-8'))
        time.sleep(0.1)
        log_file.close()
        bt.close()
        print("已关闭。")

if __name__ == "__main__":
    main()