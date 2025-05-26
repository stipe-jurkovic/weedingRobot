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
import time  

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Joy, 'joy', self.listener_callback, 10)
        self.last_button_state = 0
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 4000)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 3000)
        self.set_auto_camera_controls()
        self.get_logger().info("Camera initialized")

    def set_auto_camera_controls(self):
        try:
            subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "-c", "auto_exposure=3"], check=True)
            subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "-c", "focus_automatic_continuous=1"], check=True)
            self.get_logger().info("Auto camera controls set")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to set camera controls: {e}")

    def listener_callback(self, msg: Joy) -> None:
        if msg.buttons[0] != self.last_button_state and msg.buttons[0] == 1:
            self.get_logger().info("Button pressed, capturing image")
            
            for i in range(5):  # Grab a few frames, keep the last one
                ret, frame = self.cap.read()
                
            time_taken = datetime.now().strftime("%Y-%m-%d_%H%M%S")
            if ret:
                _, img_encoded = cv2.imencode('.jpg', frame)
                files = {'image': (time_taken + '.jpg', img_encoded.tobytes(), 'image/jpeg')}
                try:
                    response = requests.post('http://192.168.18.107:8000/upload/', files=files, timeout=5)
                    self.get_logger().info(f"Image sent, status code: {response.status_code}")
                except requests.RequestException as e:
                    self.get_logger().error(f"Failed to send image: {e}")
            else:
                self.get_logger().error("Failed to capture image")
        
        self.last_button_state = msg.buttons[0]

    def destroy_node(self):
        self.cap.release()
        self.get_logger().info("Camera released")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        minimal_subscriber.get_logger().info('Node interrupted by user')
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()