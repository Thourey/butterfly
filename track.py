import cv2
import numpy as np
import time
import os
import json
from collections import defaultdict

class InfraredLightDetector:
    def __init__(self, threshold=200, min_area=5, max_area=1000, blur_kernel=5):
        self.threshold = threshold
        self.min_area = min_area
        self.max_area = max_area
        self.blur_kernel = blur_kernel
        
    def detect_light_position(self, image):
        if self.blur_kernel > 0:
            blurred = cv2.GaussianBlur(image, (self.blur_kernel, self.blur_kernel), 0)
        else:
            blurred = image.copy()
        
        _, thresholded = cv2.threshold(blurred, self.threshold, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        valid_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if self.min_area < area < self.max_area:
                valid_contours.append(cnt)
        
        if not valid_contours:
            return None, thresholded
        
        main_contour = max(valid_contours, key=cv2.contourArea)
        (x, y), _ = cv2.minEnclosingCircle(main_contour)
        return (int(x), int(y)), thresholded


class CameraCalibrator:
    def __init__(self, camera_configs):
        self.cameras = camera_configs
        self.calibration_data = defaultdict(dict)
        self.calibration_points = [
            {"name": "原点", "pos": [0, 0, 0], "desc": "将光源放置在坐标系原点(0,0,0)"},
            {"name": "X轴", "pos": [1, 0, 0], "desc": "将光源沿X轴移动1单位长度(1,0,0)"},
            {"name": "Y轴", "pos": [0, 1, 0], "desc": "将光源沿Y轴移动1单位长度(0,1,0)"}
        ]
        self.current_point_index = 0
        self.detection_failed_cams = set()
    
    def capture_calibration_points(self):
        print("\n=== 相机标定开始 ===")
        print("请按顺序放置点光源到指定位置，确保所有摄像头都能看到光源")
        
        # 逐个标定点进行采集
        while self.current_point_index < len(self.calibration_points):
            point = self.calibration_points[self.current_point_index]
            self._capture_single_point(point)
        
        self._calculate_camera_positions()
        self._save_calibration_data()
        print("\n=== 相机标定完成 ===")
    
    def _capture_single_point(self, point):
        """采集单个标定点的数据"""
        while True:
            print(f"\n>>> {point['desc']} <<<")
            input("放置好后按回车键开始检测...")
            
            # 为所有摄像头采集数据
            self.detection_failed_cams.clear()
            temp_results = {}
            
            for cam_id in self.cameras:
                result = self._get_camera_input(cam_id, point["name"])
                if result:
                    temp_results[cam_id] = result
                    print(f"摄像头 {self.cameras[cam_id]} 检测到光源位置: {result}")
                else:
                    print(f"警告: 摄像头 {self.cameras[cam_id]} 未检测到光源!")
                    self.detection_failed_cams.add(cam_id)
            
            if not self.detection_failed_cams:
                # 所有摄像头都检测成功，保存结果
                for cam_id, pos in temp_results.items():
                    self.calibration_data[cam_id][point["name"]] = pos
                self.current_point_index += 1
                print(f"{point['name']} 位置数据采集完成")
                break
            else:
                # 有摄像头检测失败，显示提示并重试
                failed_cam_names = [self.cameras[cid] for cid in self.detection_failed_cams]
                print(f"\n以下摄像头未检测到光源: {', '.join(failed_cam_names)}")
                print("请检查:")
                print("1. 光源是否在所有摄像头视野内")
                print("2. 光源是否足够明亮")
                print("3. 摄像头是否对焦清晰")
                input("调整后按回车键重试...")
    
    def _get_camera_input(self, cam_id, point_name):
        """从单个摄像头获取光源位置"""
        cap = cv2.VideoCapture(cam_id)
        if not cap.isOpened():
            print(f"错误: 无法打开摄像头 {cam_id}")
            return None
        
        # 设置分辨率
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        # 尝试读取多帧以确保摄像头就绪
        for _ in range(5):
            ret, frame = cap.read()
            if ret:
                break
            time.sleep(0.1)
        
        if not ret:
            print(f"警告: 摄像头 {cam_id} 读取帧失败")
            cap.release()
            return None
        
        detector = InfraredLightDetector(threshold=220)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        position, _ = detector.detect_light_position(gray)
        
        cap.release()
        return position
    
    def _calculate_camera_positions(self):
        """使用三点标定法计算相机位置"""
        for cam_id in self.cameras:
            data = self.calibration_data[cam_id]
            
            # 获取三个标定点的图像坐标
            u0, v0 = data["原点"]
            u1, v1 = data["X轴"]
            u2, v2 = data["Y轴"]
            
            # 构建线性方程组 AX = B
            A = np.array([
                [1, 0, -u0],
                [0, 1, -v0],
                [1, 0, -u1],
                [0, 1, -v2]
            ])
            
            B = np.array([
                [0],
                [0],
                [1],
                [1]
            ])
            
            # 最小二乘解
            X = np.linalg.lstsq(A, B, rcond=None)[0]
            
            # 相机参数
            fx = 1 / X[2][0]  # x方向比例因子
            fy = 1 / X[2][0]  # y方向比例因子 (假设fx = fy)
            
            # 相机位置 (x,y,z)
            cam_pos = [
                -u0 / fx,
                -v0 / fy,
                1 / X[2][0]  # 焦距
            ]
            
            self.calibration_data[cam_id]["position"] = cam_pos
            self.calibration_data[cam_id]["focal_length"] = 1 / X[2][0]
            self.calibration_data[cam_id]["pixel_scale"] = (fx, fy)
    
    def _save_calibration_data(self):
        with open("camera_calibration.json", "w") as f:
            json.dump(dict(self.calibration_data), f, indent=4)
        print("相机标定数据已保存到 camera_calibration.json")


class LightTracker:
    def __init__(self):
        try:
            with open("camera_calibration.json", "r") as f:
                self.calibration_data = json.load(f)
            print("相机标定数据加载成功")
        except FileNotFoundError:
            print("错误: 未找到标定文件，请先运行相机标定")
            exit(1)
        
        self.cameras = {}
        for cam_id, config in self.calibration_data.items():
            self.cameras[int(cam_id)] = config
    
    def track_light_3d(self):
        print("\n=== 三维光源追踪开始 ===")
        print("按q键退出追踪模式")
        
        # 初始化摄像头
        caps = {}
        detectors = {}
        for cam_id in self.cameras:
            caps[cam_id] = cv2.VideoCapture(cam_id)
            if not caps[cam_id].isOpened():
                print(f"错误: 无法打开摄像头 {cam_id}")
                continue
            detectors[cam_id] = InfraredLightDetector(threshold=220)
        
        try:
            while True:
                positions = {}
                
                # 从所有摄像头获取光源位置
                for cam_id, cap in caps.items():
                    ret, frame = cap.read()
                    if ret:
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        pos, _ = detectors[cam_id].detect_light_position(gray)
                        if pos:
                            positions[cam_id] = pos
                
                # 如果有至少两个摄像头检测到光源，计算3D位置
                if len(positions) >= 2:
                    try:
                        x, y, z = self._triangulate_position(positions)
                        print(f"三维坐标: X={x:.2f}, Y={y:.2f}, Z={z:.2f}", end='\r')
                    except Exception as e:
                        print(f"定位错误: {str(e)}", end='\r')
                else:
                    print("等待更多摄像头检测到光源...", end='\r')
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            for cap in caps.values():
                cap.release()
    
    def _triangulate_position(self, positions):
        """使用最小二乘法三角定位"""
        A = []
        b = []
        
        for cam_id, (u, v) in positions.items():
            cam_data = self.calibration_data[str(cam_id)]
            fx, fy = cam_data["pixel_scale"]
            cx, cy, cz = cam_data["position"]
            
            # 构建观测方程 (考虑相机位置)
            A.append([fx, 0, -(u - cx)])
            A.append([0, fy, -(v - cy)])
            b.append(fx*cx + (u-cx)*cz)
            b.append(fy*cy + (v-cy)*cz)
        
        # 最小二乘解
        A = np.array(A)
        b = np.array(b)
        x = np.linalg.lstsq(A, b, rcond=None)[0]
        
        return x[0], x[1], x[2]


def main():
    # 配置摄像头ID和名称
    camera_configs = {
        5: "左摄像头",
        2: "中摄像头", 
        6: "右摄像头"
    }
    
    while True:
        print("\n=== 三维点光源定位系统 ===")
        print("1. 相机标定模式")
        print("2. 光源追踪模式")
        print("3. 退出")
        
        choice = input("请选择模式: ")
        
        if choice == "1":
            calibrator = CameraCalibrator(camera_configs)
            calibrator.capture_calibration_points()
        elif choice == "2":
            tracker = LightTracker()
            tracker.track_light_3d()
        elif choice == "3":
            print("程序退出")
            break
        else:
            print("无效选择，请重新输入")


if __name__ == "__main__":
    # 清除控制台
    os.system('cls' if os.name == 'nt' else 'clear')
    main()