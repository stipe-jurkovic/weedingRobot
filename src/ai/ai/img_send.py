import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import subprocess
import cv2
import time  
import numpy as np
import requests
from datetime import datetime
from cv_bridge import CvBridge
bridge = CvBridge()

# Adaptirano sa https://github.com/alexzzhu/auto_exposure_control

# PI kontroler parametri
camera_number = 0
err_i = 0
desired_msv = 3.5
k_p = 1
k_i = 0.8
max_i = 2
exposure = 73
treshold = 0.75 
last_time = 0
sendFps = 5
lastImageTime = 0
last_frame_time = time.time()

def send_image_to_server(frame): ########################### not tested yet
    time_taken = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    _, img_encoded = cv2.imencode('.jpg', frame)
    files = {'image': (time_taken + '.jpg', img_encoded.tobytes(), 'image/jpeg')}
    try:
        response = requests.post('http://192.168.18.107:8000/upload/', files=files, timeout=5)
        print(f"Image sent, status code: {response.status_code}")
    except requests.RequestException as e:
        print(f"Failed to send image: {e}")
    else:
        print("Failed to capture image")
        
class CameraNode(Node):
    def __init__(self, cam):
        super().__init__('camera_node')
        self.cam = cam

        self.publisher = self.create_publisher(CompressedImage, 'image/compressed', 3)
        self.timer = self.create_timer(0.01, self.listener_callback)

        self.get_logger().info("Camera initialized")

    def listener_callback(self) -> None:
        ret, frame = self.cam.read()
        if not ret:
            self.get_logger().warn("Failed to read frame")
            return
        global last_frame_time
        now = time.time()
        fps = 1.0 / (now - last_frame_time)
        last_frame_time = now
        #self.get_logger().info(f"Actual frame rate: {fps:.2f} FPS")

        global last_time, lastImageTime

        if now - last_time > 0.1:
            self.auto_exposure_control(frame)
            last_time = now

        if now - lastImageTime > 1 / sendFps:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"

            resized_frame = cv2.resize(frame, (1600, 1200), interpolation=cv2.INTER_AREA)
            ret, buffer = cv2.imencode('.jpg', resized_frame)
            msg.data = buffer.tobytes()

            self.publisher.publish(msg)
            #self.get_logger().info("Published compressed image")
            lastImageTime = now

    def destroy_node(self):
        self.cam.release()
        self.get_logger().info("Camera released")
        super().destroy_node()
        
    def set_exposure(self, changedExposure):
        global exposure 
        exposure = int(np.clip(changedExposure, 1, 5000))
        self.cam.set(cv2.CAP_PROP_EXPOSURE, exposure)
        #self.get_logger().info(f"Exposure set to: {self.cam.get(cv2.CAP_PROP_EXPOSURE)}")

    def auto_exposure_control(self, frame):
        global err_i

        brightness_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rows, cols = brightness_image.shape

        hist = cv2.calcHist([brightness_image], [0], None, [10], [0, 256])
        mean_sample_value = sum(hist[i] * (i + 1) for i in range(len(hist)))
        mean_sample_value /= (rows * cols)

        err_p = desired_msv - mean_sample_value
        err_i += err_p
        err_i = np.clip(err_i, -max_i, max_i)
        
        boost = 1
        if exposure > 300:
            boost = 100
        elif exposure > 200:
            boost = 40
        elif exposure > 100:
            boost = 10

        if abs(err_p) > treshold and (exposure < 5000 or err_p < 0 or err_i < 0):
            current_exp = float(exposure)
            new_exp = float(current_exp + k_p * err_p + k_i * err_i*boost)
            self.set_exposure(new_exp)
            self.get_logger().info(f"Exposure update: {current_exp:.3f} -> {new_exp:.3f}")


def main(args=None):
    global camera_number
    while True:
        try:
            print("Attempting to connect to camera "+ str(camera_number) +" ...")
            rclpy.init(args=args)
            cam = cv2.VideoCapture(camera_number, cv2.CAP_V4L2)
            cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cam.set(cv2.CAP_PROP_FRAME_WIDTH, 4000)
            cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 3000)
            cam.set(cv2.CAP_PROP_FPS, 30)
            cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # Manual mode
            print(cam.get(cv2.CAP_PROP_EXPOSURE))
            if not cam.isOpened():
                print("Failed to open camera.")
                print("Releasing camera and retrying in 2 seconds...")
                camera_number += 1
                if camera_number > 2:
                    camera_number = 0
                cam.release()
                time.sleep(2)
                continue

            print("Camera opened successfully.")
            camera_node = CameraNode(cam)

            rclpy.spin(camera_node)

            camera_node.destroy_node()
            break  # Exit loop if everything worked

        except Exception as e:
            import traceback
            print("Caught exception during camera init or ROS spin:")
            traceback.print_exc()  # Print full traceback
            print("Retrying in 2 seconds...")

        finally:
            rclpy.shutdown()
            time.sleep(2)


if __name__ == '__main__':
    main()
