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
from std_msgs.msg import String
import gc
import psutil
import os
import threading
import json
import base64

from turbojpeg import TurboJPEG, TJPF_GRAY, TJSAMP_420

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
sendFps = 3
lastImageTime = 0
last_frame_time = time.time()
jpeg = TurboJPEG()

target_width = 640
target_height = 480
        
class CameraNode(Node):
    def __init__(self, cam):
        super().__init__('camera_node')
        self.cam = cam

        self.publisher = self.create_publisher(CompressedImage, 'image/compressed', 1)
        self.annotated_image_publisher = self.create_publisher(CompressedImage, 'image/annotated', 1)
        self.burn_publisher = self.create_publisher(String, 'points_to_burn', 1)
        self.timer = self.create_timer(0.1, self.listener_callback)

        self.subscription = self.create_subscription(
            String,
            'TriggerBurnTopic',
            self.response_callback,
            10
        )
        self.send_next_image_to_api = False

        self.get_logger().info("Camera initialized")

    def listener_callback(self) -> None:
        ret, frame = self.cam.read()
        if not ret:
            self.get_logger().warn("Failed to read frame")
            return
        now = time.time()
        print_memory_usage()
        
        if self.send_next_image_to_api:
            threading.Thread(target=self.send_image_to_server, args=(frame.copy(),), daemon=True).start()
            gc.collect()
            self.send_next_image_to_api = False
    

        global last_time, lastImageTime

        if now - last_time > 0.2:
            brightness_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            hist = cv2.calcHist([brightness_image], [0], None, [10], [0, 256])
            self.auto_exposure_control(hist, brightness_image.shape)
            last_time = now

        if now - lastImageTime > 1 / sendFps:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            resized_frame = cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_AREA)

            msg.data = jpeg.encode(resized_frame, quality=75, jpeg_subsample=TJSAMP_420)

            print(f"Frame size: {frame.shape[1]}x{frame.shape[0]}, Encoded size: {len(msg.data)} bytes")
            self.publisher.publish(msg)
            #self.get_logger().info("Published compressed image")
            lastImageTime = now
        del frame
        gc.collect()  # Force garbage collection to free memory
    def response_callback(self, msg: String):
        self.get_logger().info(f"Received response: {msg.data}")
        if msg.data == "Burn":
            # Trigger burning logic here
            self.get_logger().info("Burning triggered")
            self.send_next_image_to_api = True
        else:
            self.get_logger().warn(f"Unknown command received: {msg.data}")

    def send_image_to_server(self, frame): 
        time_taken = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        _, img_encoded = cv2.imencode('.jpg', frame)
        files = {'image': (time_taken + '.jpg', img_encoded.tobytes(), 'image/jpeg')}
        try:
            response = requests.post('http://192.168.18.107:8000/upload/', files=files, timeout=50)
            print(f"Image sent, status code: {response.status_code}")
            #print(f"Response: {response.text}")
            self.publish_coords(response.text)
        except requests.RequestException as e:
            print(f"Failed to send image: {e}")
            
    def publish_coords(self, response):
        try:
            response_dict = json.loads(response)
        except json.JSONDecodeError:
            print("Error: Response is not valid JSON")
            return []

        # Provjeri postoji li ključ 'coords' i je li tip lista
        coords = response_dict.get('coords')
        if not isinstance(coords, list):
            print("Warning: 'coords' missing or not a list")
            return []

        # Provjeri da su svi elementi liste liste s dva broja (x,y)
        safe_coords = []
        for point in coords:
            if (isinstance(point, list) or isinstance(point, tuple)) and len(point) == 2:
                x, y = point
                if isinstance(x, (int, float)) and isinstance(y, (int, float)):
                    safe_coords.append((x, y))
                else:
                    print(f"Warning: Invalid coordinate values: {point}")
            else:
                print(f"Warning: Invalid coordinate format: {point}")

        msg = String()
        msg.data = str(safe_coords)
        self.burn_publisher.publish(msg)
        self.get_logger().info(f"Published coordinates: {safe_coords}")
                
        annotatedImg = CompressedImage()
        annotatedImg.header.stamp = self.get_clock().now().to_msg()
        annotatedImg.format = "jpeg"
        data_str = response_dict.get("fully_annotated_image", "")

        if data_str:
            try:
                annotatedImg = CompressedImage()
                annotatedImg.header.stamp = self.get_clock().now().to_msg()
                annotatedImg.format = "jpeg"
                annotatedImg.data = base64.b64decode(data_str)

                self.annotated_image_publisher.publish(annotatedImg)
            except Exception as e:
                self.get_logger().error(f"Greška pri dekodiranju base64: {e}")
        else:
            self.get_logger().info("Poruka nije ispravna: nema podataka")
    
    def destroy_node(self):
        self.cam.release()
        self.get_logger().info("Camera released")
        super().destroy_node()
        
    def set_exposure(self, changedExposure):
        global exposure 
        exposure = int(np.clip(changedExposure, 1, 5000))
        self.cam.set(cv2.CAP_PROP_EXPOSURE, exposure)
        #self.get_logger().info(f"Exposure set to: {self.cam.get(cv2.CAP_PROP_EXPOSURE)}")

    def auto_exposure_control(self, hist, shape):
        global err_i
        rows, cols = shape

        mean_sample_value = sum(hist[i] * (i + 1) for i in range(len(hist)))
        mean_sample_value /= (rows * cols)

        err_p = desired_msv - mean_sample_value
        err_i += err_p
        err_i = np.clip(err_i, -max_i, max_i)
        
        boost = 1
        if exposure > 500:
            boost = 100
        elif exposure > 200:
            boost = 20
        elif exposure > 100:
            boost = 10

        if abs(err_p) > treshold and (exposure < 5000 or err_p < 0 or err_i < 0):
            current_exp = float(exposure)
            new_exp = float(current_exp + k_p * err_p + k_i * err_i*boost)
            self.set_exposure(new_exp)
            self.get_logger().info(f"Exposure update: {current_exp:.3f} -> {new_exp:.3f}")

def print_memory_usage():
    process = psutil.Process(os.getpid())
    mem_info = process.memory_info()
    print(f"Memory used: {mem_info.rss / (1024 * 1024):.2f} MB")
    
def set_manual_focus(device, value):
    try:
        # Isključi automatski fokus
        subprocess.run(
            ["v4l2-ctl", "-d", device, "--set-ctrl", "focus_automatic_continuous=0"],
            check=True
        )
        # Postavi fokus na određenu vrijednost (0–1023)
        subprocess.run(
            ["v4l2-ctl", "-d", device, "--set-ctrl", f"focus_absolute={value}"],
            check=True
        )
        print(f"Manual focus set to {value}.")
    except subprocess.CalledProcessError as e:
        print(f"Greška prilikom postavljanja fokusa: {e}")

# Primjer poziva

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
            set_manual_focus("/dev/video" + str(camera_number), 552)
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
