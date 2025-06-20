import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as rosImage
from PIL import Image
import cv2
import time  
import numpy as np
from cv_bridge import CvBridge
bridge = CvBridge()

#adapted from https://github.com/alexzzhu/auto_exposure_control

# PI controller parameters
err_i = 0
desired_msv = 3.5
k_p = 0.05
k_i = 0.01
max_i = 2
exposure = -7  
treshold = 0.75 
last_time = 0 

class CameraNode(Node):
    def __init__(self, cam):
        super().__init__('minimal_subscriber')
        self.cam = cam
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 4000)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 3000)
        self.cam.set(cv2.CAP_PROP_FPS, 30)
        self.cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # manual mode
        self.publisher = self.create_publisher(rosImage, 'images', 10)
        self.timer = self.create_timer(1, self.listener_callback)
        self.get_logger().info("Camera initialized")

    def listener_callback(self) -> None:
        ret, frame = self.cam.read()
        if not ret:
            print("Ne mogu pročitati frame")
        print(f"Frame shape: {frame.shape}, min: {frame.min()}, max: {frame.max()}")
        image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(image_message)
        # Ograniči brzinu update-a ekspozicije (npr. 1x u 0.5s)
        now = time.time()
        global last_time
        if now - last_time > 0.1:
            self.auto_exposure_control(frame)
            last_time = now

    def destroy_node(self):
        self.cam.release()
        self.get_logger().info("Camera released")
        super().destroy_node()

    def set_exposure(self, changedExposure):
        self.cam.set(cv2.CAP_PROP_EXPOSURE, changedExposure)
        global exposure
        exposure = changedExposure

    def auto_exposure_control(self, frame):
        global err_i
        
        brightness_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[:,:,2]
        rows, cols = brightness_image.shape

        hist = cv2.calcHist([brightness_image], [0], None, [10], [0, 256])
        
        mean_sample_value = 0
        for i in range(len(hist)):
            mean_sample_value += hist[i]*(i+1)
        mean_sample_value /= (rows*cols)

        err_p = desired_msv - mean_sample_value
        err_i += err_p
        if abs(err_i) > max_i:
            err_i = np.sign(err_i)*max_i

        print(f"Mean sample value: {float(mean_sample_value):.3f}, Error P: {float(err_p):.3f}, Error I: {float(err_i):.3f}")
        if abs(err_p) > treshold:
            current_exp = float(exposure)
            new_exp = float(current_exp + k_p*err_p + k_i*err_i)
            self.set_exposure(new_exp)
            print(f"Exposure update: {current_exp:.3f} -> {new_exp:.3f}")

def main(args=None):
    while True:
        try:
            print("Attempting to connect to camera...")
            rclpy.init(args=args)
            
            cam = cv2.VideoCapture(0, cv2.CAP_V4L2)
            
            if cam.isOpened():
                print("Camera opened successfully.")
            else:
                print("Failed to open camera.")
                continue
            
            camera_node = CameraNode(cam)

            rclpy.spin(camera_node)
            
            camera_node.destroy_node()
            break  # Exit loop if successful
        except (OSError) as e:
            print(f"Canera error: {e}. Retrying in 5 seconds...")
        finally:
            rclpy.shutdown()
        time.sleep(5)

if __name__ == '__main__':
    main()