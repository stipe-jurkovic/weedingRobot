import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
import serial
ser = serial.Serial("/dev/serial/by-path/platform-xhci-hcd.1-usb-0:2.3:1.0-port0", 115200)
speed = 20

def map_joystick_to_motors(x, y):
    # Combine arcade drive logic
    left = y + x
    right = y - x

    # Clamp to range [-1, 1]
    max_val = max(abs(left), abs(right), 1)
    left /= max_val
    right /= max_val

    return left, right

class MinimalSubscriber(Node):
    
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Joy,'joy',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Joy):
        # Example speed command {"MotorL" :300,"DirectionL":0,"MotorR":300,"DirectionR":0}
        global speed
        speed += msg.buttons[5] * 1
        speed -= msg.buttons[4] * 1
        if speed > 100:
            speed = 100
        elif speed < 0:
            speed = 0
        motorR, motorL  = map_joystick_to_motors(msg.axes[0], msg.axes[3])
        # motorL = round(abs(msg.axes[1] * speed))
        # motorR = round(abs(msg.axes[3] * speed))
        dirL = 1 if motorL < 0 else 0
        dirR = 1 if motorR < 0 else 0
        brake = 1 if msg.buttons[2] == 1 else 0
        message = '{"MotorL" :' + str(round(abs(motorL)* speed)) + ',"DirectionL":' + str(dirL) + ',"MotorR":' + str(round(abs(motorR)* speed)) + ',"DirectionR":' + str(dirR) + ',"Brake":' + str(brake) + '}\n'
        print(message)
        #self.get_logger().info(message)
        ser.write(message.encode('utf-8'))


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