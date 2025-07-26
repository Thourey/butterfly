import cv2
import threading
import numpy as np
import time
import os

class InfraredLightDetector:
    def __init__(self, threshold=200, min_area=5, max_area=1000, blur_kernel=5):
        """
        初始化红外光源检测器
        
        参数:
            threshold: 二值化阈值 (0-255)
            min_area: 最小光源区域面积 (像素)
            max_area: 最大光源区域面积 (像素)
            blur_kernel: 高斯模糊核大小 (奇数)
        """
        self.threshold = threshold
        self.min_area = min_area
        self.max_area = max_area
        self.blur_kernel = blur_kernel
        
    def detect_light_position(self, image):
        """
        检测图像中的点光源位置
        
        参数:
            image: 输入图像 (灰度图)
            
        返回:
            (x, y): 光源中心坐标 (如果没有检测到光源，返回None)
            processed_img: 处理后的二值化图像
        """
        # 1. 图像预处理 - 高斯模糊减少噪声
        if self.blur_kernel > 0:
            blurred = cv2.GaussianBlur(image, (self.blur_kernel, self.blur_kernel), 0)
        else:
            blurred = image.copy()
        
        # 2. 阈值处理
        _, thresholded = cv2.threshold(blurred, self.threshold, 255, cv2.THRESH_BINARY)
        
        # 3. 查找轮廓
        contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 4. 筛选合适的轮廓
        valid_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if self.min_area < area < self.max_area:
                valid_contours.append(cnt)
        
        # 5. 如果没有检测到合适的光源，返回None
        if not valid_contours:
            return None, thresholded
        
        # 6. 选择面积最大的轮廓 (假设只有一个主要光源)
        main_contour = max(valid_contours, key=cv2.contourArea)
        
        # 7. 计算最小外接圆和质心
        (x, y), radius = cv2.minEnclosingCircle(main_contour)
        center = (int(x), int(y))
        radius = int(radius)
        
        return center, thresholded


class CameraFeeder:
    def __init__(self, device_id, window_name, fps=30, buffer_size=30):
        self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.buffer = []
        self.buffer_size = buffer_size
        self.device_id = device_id
        self.window_name = window_name
        self.latest_position = None
        self.detector = InfraredLightDetector(
            threshold=220, 
            min_area=10, 
            max_area=500,
            blur_kernel=5
        )
        self.frame_count = 0
        self.start_time = time.time()

    def read_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return False, None
        
        # 转换为灰度图进行光源检测
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        position, _ = self.detector.detect_light_position(gray)
        self.latest_position = position
        
        # 计算FPS
        self.frame_count += 1
        elapsed_time = time.time() - self.start_time
        fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
        
        # 可视化结果
        vis_img = self._visualize_detection(frame, position, fps)
        
        self.buffer.append(vis_img)
        if len(self.buffer) > self.buffer_size:
            self.buffer.pop(0)
            
        return True, vis_img

    def _visualize_detection(self, frame, position, fps):
        """可视化检测结果"""
        vis_img = frame.copy()
        
        # 显示FPS
        cv2.putText(vis_img, f"FPS: {fps:.1f}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        if position is not None:
            x, y = position
            # 绘制外接圆
            cv2.circle(vis_img, (x, y), 15, (0, 255, 255), 2)
            # 绘制中心十字线
            cv2.line(vis_img, (x-20, y), (x+20, y), (0, 0, 255), 2)
            cv2.line(vis_img, (x, y-20), (x, y+20), (0, 0, 255), 2)
            # 显示坐标
            coord_text = f"({x}, {y})"
            cv2.putText(vis_img, coord_text, (x+25, y+5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # 打印到控制台
            print(f"[{self.window_name}] 光源位置: {coord_text} | FPS: {fps:.1f}")
        
        return vis_img

    def get_latest_position(self):
        return self.latest_position

    def release(self):
        self.cap.release()


def capture_thread(device_id, window_name):
    feeder = CameraFeeder(device_id, window_name)
    print(f"[{window_name}] 已启动 (设备ID: {device_id})")

    try:
        while True:
            ret, vis_img = feeder.read_frame()
            if not ret:
                break
            
            cv2.imshow(window_name, vis_img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                save_path = f"saved_frames_{window_name}"
                os.makedirs(save_path, exist_ok=True)
                for i, img in enumerate(feeder.buffer):
                    cv2.imwrite(f"{save_path}/frame_{i:03d}.jpg", img)
                print(f"[{window_name}] 已保存 {len(feeder.buffer)} 帧")

            if key == ord('q'):
                break
    finally:
        feeder.release()
        cv2.destroyWindow(window_name)
        print(f"[{window_name}] 已停止")


if __name__ == "__main__":
    # 清除控制台
    os.system('cls' if os.name == 'nt' else 'clear')
    
    # 摄像头配置
    cameras = [
        (0, "USB_CAM_1"),
        (2, "USB_CAM_2"),
        (6, "USB_CAM_3")
    ]

    # 启动摄像头线程
    threads = []
    for device_id, name in cameras:
        t = threading.Thread(target=capture_thread, args=(device_id, name))
        t.daemon = True
        t.start()
        threads.append(t)
        time.sleep(1)  # 避免摄像头初始化冲突

    # 主线程显示控制信息
    print("\n=== 红外光源检测系统 ===")
    print("操作说明:")
    print("1. 在各个摄像头窗口按 's' 保存当前缓冲区图像")
    print("2. 在各个摄像头窗口按 'q' 关闭该摄像头")
    print("3. 所有摄像头关闭后程序自动退出\n")

    try:
        while any(t.is_alive() for t in threads):
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n接收到中断信号，正在关闭所有摄像头...")

    print("\n所有摄像头已关闭，程序退出")