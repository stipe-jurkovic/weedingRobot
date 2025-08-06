import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import serial
import time

class StepperTranscriber(Node):
    
    def __init__(self, ser):
        super().__init__('stepper_transcriber')
        self.ser = ser
        self.subscription = self.create_subscription(
            String,
            'stepper_control',
            self.listener_callback,
            30)
        self.publisher = self.create_publisher(String, 'stepper_control_response', 30)
        # Start a timer that checks for serial messages every xxxms
        self.timer = self.create_timer(0.05, self.check_serial_input)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: String):
        print_to_Serial(self.ser, msg.data)
        self.get_logger().info(f'Received message: "{msg.data}"')
        
    def check_serial_input(self):
        if self.ser.in_waiting > 0:
            try:
                response = self.ser.readline().decode('utf-8', errors='ignore').strip()
                self.get_logger().info(f'Received from serial: "{response}"')
                msg = String()
                msg.data = response
                self.publisher.publish(msg)
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"Serial error: {e}")
        
def print_to_Serial(ser, msg):
    try:
        ser.write((msg + '\n').encode('utf-8'))  # Ensure newline for serial communication
        print(f"Sent to serial: {msg}")
    except Exception as e:
        print(f"Error writing to serial: {e}")

def main(args=None):
    while True:
        try:
            rclpy.init(args=args)
            # Attempt to open the serial port
            ser = serial.Serial("/dev/serial/by-path/platform-xhci-hcd.1-usb-0:2.1:1.0-port0", 115200,timeout=0.05)

            stepper_transcriber = StepperTranscriber(ser)

            rclpy.spin(stepper_transcriber)
            
            stepper_transcriber.destroy_node()
            break  # Exit loop if successful
        except (serial.SerialException, OSError) as e:
            print(f"Serial port error: {e}. Retrying in 5 seconds...")
        finally:
            rclpy.shutdown()
        time.sleep(5)


if __name__ == '__main__':
    main()