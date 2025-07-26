import cv2
import threading
import time
import os

class CameraFeeder:
    def __init__(self, device_id, fps=30, buffer_size=30):
        device_path = f"/dev/video{device_id}"
        self.cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开 {device_path}")
        self.cap.set(cv2.CAP_PROP_FOURCC,
                     cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.buffer = []
        self.buffer_size = buffer_size

    def read_frame(self):
        ret, frame = self.cap.read()
        if ret:
            self.buffer.append(frame)
            if len(self.buffer) > self.buffer_size:
                self.buffer.pop(0)
        return ret, frame

    def release(self):
        self.cap.release()

lock = threading.Lock()

def capture_thread(device_id, window_name):
    try:
        feeder = CameraFeeder(device_id)
        print(f"[{window_name}] 已启动")
        while True:
            with lock:
                ret, frame = feeder.read_frame()
            if not ret:
                print(f"[{window_name}] 读取帧失败")
                time.sleep(0.1)
                continue
            cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                save_dir = f"saved_frames_{window_name}"
                os.makedirs(save_dir, exist_ok=True)
                for i, img in enumerate(feeder.buffer):
                    cv2.imwrite(f"{save_dir}/frame_{i:03d}.jpg", img)
                print(f"[{window_name}] 已保存 {len(feeder.buffer)} 帧")
            if key == ord('q'):
                break
    except Exception as e:
        print(f"[{window_name}] 错误: {e}")
    finally:
        feeder.release()
        cv2.destroyWindow(window_name)
        print(f"[{window_name}] 已停止")

if __name__ == "__main__":
    # 再次确认这三个节点对应的是 3 台不同相机
    dev_nodes = [2, 4, 6]           # 按 v4l2-ctl --list-devices 结果修改
    names     = ["USB_CAM_1",
                 "USB_CAM_2",
                 "USB_CAM_3"]
    threads = [threading.Thread(target=capture_thread, args=(d, n))
               for d, n in zip(dev_nodes, names)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    print("全部退出")