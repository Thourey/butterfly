import sys
import time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QSlider, 
                             QSpinBox, QGroupBox, QTextEdit, QStatusBar)
from PyQt5.QtCore import Qt, QTimer
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

class WingControlUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.bt = None
        self.init_ui()
        self.init_serial()
        self.setup_timer()
        
    def init_ui(self):
        self.setWindowTitle("扑翼机遥控系统 v1.0")
        self.setGeometry(100, 100, 800, 600)
        
        # 主控件
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        # 左侧控制面板
        control_panel = QGroupBox("舵机控制")
        control_layout = QVBoxLayout()
        
        # 舵机1控制
        self.servo1_group = QGroupBox("左翼舵机 (Servo1)")
        servo1_layout = QVBoxLayout()
        
        self.servo1_label = QLabel("角度: 90°")
        self.servo1_slider = QSlider(Qt.Horizontal)
        self.servo1_slider.setRange(0, 180)
        self.servo1_slider.setValue(90)
        self.servo1_slider.valueChanged.connect(self.update_servo1)
        
        servo1_layout.addWidget(self.servo1_label)
        servo1_layout.addWidget(self.servo1_slider)
        self.servo1_group.setLayout(servo1_layout)
        
        # 舵机2控制
        self.servo2_group = QGroupBox("右翼舵机 (Servo2)")
        servo2_layout = QVBoxLayout()
        
        self.servo2_label = QLabel("角度: 90°")
        self.servo2_slider = QSlider(Qt.Horizontal)
        self.servo2_slider.setRange(0, 180)
        self.servo2_slider.setValue(90)
        self.servo2_slider.valueChanged.connect(self.update_servo2)
        
        servo2_layout.addWidget(self.servo2_label)
        servo2_layout.addWidget(self.servo2_slider)
        self.servo2_group.setLayout(servo2_layout)
        
        # 自动模式控制
        self.auto_group = QGroupBox("自动模式")
        auto_layout = QVBoxLayout()
        
        self.auto_btn = QPushButton("启动自动摆动")
        self.auto_btn.setCheckable(True)
        self.auto_btn.clicked.connect(self.toggle_auto_mode)
        
        # 自动模式参数
        param_layout = QHBoxLayout()
        param_layout.addWidget(QLabel("中心角度1:"))
        self.center1_spin = QSpinBox()
        self.center1_spin.setRange(0, 180)
        self.center1_spin.setValue(110)
        param_layout.addWidget(self.center1_spin)
        
        param_layout.addWidget(QLabel("中心角度2:"))
        self.center2_spin = QSpinBox()
        self.center2_spin.setRange(0, 180)
        self.center2_spin.setValue(70)
        param_layout.addWidget(self.center2_spin)
        
        param_layout.addWidget(QLabel("幅度:"))
        self.amplitude_spin = QSpinBox()
        self.amplitude_spin.setRange(0, 90)
        self.amplitude_spin.setValue(45)
        param_layout.addWidget(self.amplitude_spin)
        
        param_layout.addWidget(QLabel("步长:"))
        self.step_spin = QSpinBox()
        self.step_spin.setRange(1, 30)
        self.step_spin.setValue(10)
        param_layout.addWidget(self.step_spin)
        
        param_layout.addWidget(QLabel("频率(Hz):"))
        self.freq_spin = QSpinBox()
        self.freq_spin.setRange(1, 20)
        self.freq_spin.setValue(5)
        param_layout.addWidget(self.freq_spin)
        
        auto_layout.addLayout(param_layout)
        auto_layout.addWidget(self.auto_btn)
        self.auto_group.setLayout(auto_layout)
        
        # 添加到控制面板
        control_layout.addWidget(self.servo1_group)
        control_layout.addWidget(self.servo2_group)
        control_layout.addWidget(self.auto_group)
        control_layout.addStretch()
        control_panel.setLayout(control_layout)
        
        # 右侧日志面板
        log_panel = QGroupBox("通信日志")
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        
        self.clear_btn = QPushButton("清空日志")
        self.clear_btn.clicked.connect(self.clear_log)
        
        log_layout.addWidget(self.log_text)
        log_layout.addWidget(self.clear_btn)
        log_panel.setLayout(log_layout)
        
        # 添加到主布局
        main_layout.addWidget(control_panel, 1)
        main_layout.addWidget(log_panel, 2)
        
        # 状态栏
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("就绪")
        
        # 自动模式变量
        self.auto_mode = False
        self.angle_offset = 0
        self.direction = 1
        
    def init_serial(self):
        try:
            self.bt = BT33("/dev/ttyUSB0", 115200)
            self.log("串口已连接: /dev/ttyUSB0 @115200bps")
            self.status_bar.showMessage("串口已连接", 3000)
        except Exception as e:
            self.log(f"串口连接失败: {str(e)}")
            self.status_bar.showMessage("串口连接失败", 3000)
    
    def setup_timer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_auto_mode)
        self.update_interval()
    
    def update_interval(self):
        freq = self.freq_spin.value()
        interval = int(1000 / freq)  # 转换为毫秒
        self.timer.setInterval(interval)
    
    def update_servo1(self, angle):
        self.servo1_label.setText(f"角度: {angle}°")
        if not self.auto_mode:
            self.send_command(angle, self.servo2_slider.value())
    
    def update_servo2(self, angle):
        self.servo2_label.setText(f"角度: {angle}°")
        if not self.auto_mode:
            self.send_command(self.servo1_slider.value(), angle)
    
    def send_command(self, angle1, angle2):
        if self.bt:
            try:
                cmd_str = f"AT+DEVICE1${angle1},{angle2}?\r\n"
                self.bt.send(cmd_str.encode('utf-8'))
                self.log(f"发送: {cmd_str.strip()}")
                
                # 接收响应
                rx = self.bt.recv(64)
                if rx:
                    try:
                        rx_str = rx.decode('utf-8', errors='ignore').strip()
                        self.log(f"接收: {rx_str}")
                    except Exception as e:
                        rx_str = rx.hex()
                        self.log(f"接收解码错误: {e}, 原始数据: {rx_str}")
            except Exception as e:
                self.log(f"发送命令错误: {str(e)}")
                self.status_bar.showMessage(f"发送命令错误: {str(e)}", 3000)
    
    def toggle_auto_mode(self):
        self.auto_mode = not self.auto_mode
        if self.auto_mode:
            self.auto_btn.setText("停止自动摆动")
            self.angle_offset = 0
            self.direction = 1
            self.timer.start()
            self.log("自动模式启动")
        else:
            self.auto_btn.setText("启动自动摆动")
            self.timer.stop()
            self.log("自动模式停止")
    
    def update_auto_mode(self):
        if not self.auto_mode:
            return
            
        # 获取参数
        center1 = self.center1_spin.value()
        center2 = self.center2_spin.value()
        amplitude = self.amplitude_spin.value()
        step = self.step_spin.value()
        
        # 计算角度
        angle1 = center1 + self.angle_offset
        angle2 = center2 - self.angle_offset  # 反向运动
        
        # 更新滑块和标签
        self.servo1_slider.setValue(angle1)
        self.servo2_slider.setValue(angle2)
        
        # 发送命令
        self.send_command(angle1, angle2)
        
        # 更新角度偏移
        self.angle_offset += step * self.direction
        
        # 检查是否到达边界
        if abs(self.angle_offset) >= amplitude:
            self.direction *= -1
            self.angle_offset = amplitude * self.direction
    
    def log(self, message):
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        self.log_text.append(f"[{timestamp}] {message}")
    
    def clear_log(self):
        self.log_text.clear()
    
    def closeEvent(self, event):
        if self.bt:
            self.bt.close()
        if self.timer.isActive():
            self.timer.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = WingControlUI()
    window.show()
    sys.exit(app.exec_())