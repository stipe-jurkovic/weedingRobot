import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import subprocess
import cv2
import time  
import numpy as np

from cv_bridge import CvBridge
bridge = CvBridge()

# PI kontroler parametri
err_i = 0
desired_msv = 3.5
k_p = 1
k_i = 0.2
max_i = 2
exposure = 57 
treshold = 0.75 
last_time = 0
sendFps = 5
lastImageTime = 0

class CameraNode(Node):
    def __init__(self, cam):
        super().__init__('camera_node')
        self.cam = cam
        self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cam.set(cv2.CAP_PROP_FPS, 60)
        self.cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # manual mode

        self.publisher = self.create_publisher(CompressedImage, 'image/compressed', 10)
        self.timer = self.create_timer(0.01, self.listener_callback)

        self.get_logger().info("Camera initialized")

    def listener_callback(self) -> None:
        ret, frame = self.cam.read()
        if not ret:
            self.get_logger().warn("Failed to read frame")
            return

        now = time.time()
        global last_time, lastImageTime

        # Auto exposure (max 10Hz)
        if now - last_time > 0.1:
            self.auto_exposure_control(frame)
            last_time = now

        if now - lastImageTime > 1 / sendFps:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"

            # Downscale to 1280x960
            resized_frame = cv2.resize(frame, (1280, 960))
            # Compress using JPEG
            ret, buffer = cv2.imencode('.jpg', resized_frame)
            msg.data = buffer.tobytes()

            self.publisher.publish(msg)
            self.get_logger().info("Published compressed image")
            lastImageTime = now
            # Get current exposure value
            exposure = self.cam.get(cv2.CAP_PROP_EXPOSURE)
            print(f"Current exposure: {exposure}")

    def destroy_node(self):
        self.cam.release()
        self.get_logger().info("Camera released")
        super().destroy_node()

    def set_exposure(self, changedExposure):
        global exposure
        changedExposure = int(np.clip(changedExposure, 1, 5000))  # ograniči na min-max
        exposure = changedExposure
        try:
            subprocess.run([
                "v4l2-ctl", "-d", "/dev/video0",
                "--set-ctrl", "auto_exposure=1",
                "--set-ctrl", f"exposure_time_absolute={changedExposure}"
            ], check=True)
            self.get_logger().info(f"Exposure set via v4l2-ctl: {changedExposure}")
        except Exception as e:
            self.get_logger().warn(f"Failed to set exposure with v4l2-ctl: {e}")

    def auto_exposure_control(self, frame):
        global err_i

        brightness_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[:, :, 2]
        rows, cols = brightness_image.shape

        hist = cv2.calcHist([brightness_image], [0], None, [10], [0, 256])
        mean_sample_value = sum(hist[i] * (i + 1) for i in range(len(hist)))
        mean_sample_value /= (rows * cols)

        err_p = desired_msv - mean_sample_value
        err_i += err_p
        err_i = np.clip(err_i, -max_i, max_i)

        if abs(err_p) > treshold:
            current_exp = float(exposure)
            new_exp = float(current_exp + k_p * err_p + k_i * err_i)
            self.set_exposure(new_exp)
            self.get_logger().info(f"Exposure update: {current_exp:.3f} -> {new_exp:.3f}")


def main(args=None):
    while True:
        try:
            print("Attempting to connect to camera...")
            rclpy.init(args=args)

            cam = cv2.VideoCapture(0, cv2.CAP_V4L2)
            cam.set(cv2.CAP_PROP_FRAME_WIDTH, 4000)
            cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 3000)

            if not cam.isOpened():
                print("Failed to open camera.")
                continue

            print("Camera opened successfully.")
            camera_node = CameraNode(cam)

            rclpy.spin(camera_node)

            camera_node.destroy_node()
            break
        except OSError as e:
            print(f"Camera error: {e}. Retrying in 5 seconds...")
        finally:
            rclpy.shutdown()
        time.sleep(5)


if __name__ == '__main__':
    main()
