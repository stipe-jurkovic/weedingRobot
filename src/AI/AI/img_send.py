import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
import serial
import requests
from PIL import Image
import io
import cv2
from datetime import datetime
import subprocess

lastButtonState = 0

def set_auto_camera_controls():
    # Postavi auto exposure
    subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "-c", "auto_exposure=3"])

    # (opcionalno) Uključi automatski fokus
    subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "-c", "focus_automatic_continuous=1"])

class MinimalSubscriber(Node):
    
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Joy,'joy',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Joy):
        # Triangle button is usually index 3 on many controllers
        global lastButtonState
        if msg.buttons[0] != lastButtonState and msg.buttons[0] == 1:
            set_auto_camera_controls()  

            # Capture image from camera (example using OpenCV)
            cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 4000)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 3000)
            
            for i in range(15):
                cap.read()
            ret, frame = cap.read()
            cap.release()
            timeTaken = datetime.now().strftime("%Y-%m-%d_%H%M%S")
            if ret:
                _, img_encoded = cv2.imencode('.jpg', frame)
                files = {'image': (timeTaken + '.jpg', img_encoded.tobytes(), 'image/jpeg')}
                response = requests.post('http://192.168.18.107:8000/upload', files=files)
                self.get_logger().info(f"Image sent, status code: {response.status_code}")
            else:
                self.get_logger().error("Failed to capture image")
        lastButtonState = msg.buttons[0]


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()