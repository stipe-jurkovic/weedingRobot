import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

# Example speed command {"MotorL" :300,"DirectionL":0,"MotorR":300,"DirectionR":0}

max_speed = 100

def map_speed_to_motors(angularZ, linearX):
    # Combine arcade drive logic
    left = linearX + angularZ
    right = linearX - angularZ

    # Clamp both left and right to [-max_speed, max_speed]
    left = round(max(-max_speed, min(left, max_speed)))
    right = round(max(-max_speed, min(right, max_speed)))

    dirL = 1 if left < 0 else 0
    dirR = 1 if right < 0 else 0

    return abs(int(left)), dirL, abs(int(right)), dirR


class WheelControl(Node):
    
    def __init__(self, ser):
        super().__init__('wheel_control')
        self.subscription = self.create_subscription(Twist, 'wheels', self.listener_callback,10)
        self.ser = ser
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Twist):
        motorL, dirL, motorR, dirR  = map_speed_to_motors(msg.angular.z, msg.linear.x)
        brake = 0
        message = '{"MotorL" :' + str(motorL) + ',"DirectionL":' + str(dirL) + ',"MotorR":' + str(motorR) + ',"DirectionR":' + str(dirR) + ',"Brake":' + str(brake) + '}\n'
        if motorL != 0 and motorR != 0:
            print(message)
            #self.get_logger().info(message)
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
            break
        except (serial.SerialException, OSError) as e:
            print(f"Serial port error: {e}. Retrying in 5 seconds...")
        finally:
            rclpy.shutdown()
        time.sleep(5)


if __name__ == '__main__':
    main()