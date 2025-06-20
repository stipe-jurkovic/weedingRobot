import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import serial
speed = 20

def map_joystick_to_motors(angularZ, linearX):
    # Combine arcade drive logic
    left = linearX + angularZ
    right = linearX - angularZ

    # Clamp to range [-1, 1]
    max_val = max(abs(left), abs(right), 1)
    left /= max_val
    right /= max_val

    return left, right

class WheelControl(Node):
    
    def __init__(self, ser):
        super().__init__('wheel_control')
        self.subscription = self.create_subscription(Twist,'wheels',self.listener_callback,10)
        self.ser = ser
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Twist):
        # Example speed command {"MotorL" :300,"DirectionL":0,"MotorR":300,"DirectionR":0}
        global speed
        motorR, motorL  = map_joystick_to_motors(msg.angular.z, msg.linear.x)
        dirL = 1 if motorL < 0 else 0
        dirR = 1 if motorR < 0 else 0
        motorLeft = round(abs(motorL)* speed)
        motorRight = round(abs(motorR)* speed)
        brake = 0#1 if msg.buttons[2] == 1 else 0
        message = '{"MotorL" :' + str(motorLeft) + ',"DirectionL":' + str(dirL) + ',"MotorR":' + str(motorRight) + ',"DirectionR":' + str(dirR) + ',"Brake":' + str(brake) + '}\n'
        if motorL != 0 and motorR != 0:
            print(message)
            self.get_logger().info(message)
        self.ser.write(message.encode('utf-8'))

def main(args=None):
    while True:
        try:
            print("Attempting to connect to serial port...")
            rclpy.init(args=args)
            # Attempt to open the serial port
            ser = serial.Serial("/dev/serial/by-path/platform-xhci-hcd.1-usb-0:2.3:1.0-port0", 115200)
            
            if ser.is_open:
                print("Serial port opened successfully.")
            else:
                print("Failed to open serial port.")
                continue
            
            wheel_control = WheelControl(ser)

            rclpy.spin(wheel_control)
            
            wheel_control.destroy_node()
            break  # Exit loop if successful
        except (serial.SerialException, OSError) as e:
            print(f"Serial port error: {e}. Retrying in 5 seconds...")
        finally:
            rclpy.shutdown()
        time.sleep(5)


if __name__ == '__main__':
    main()