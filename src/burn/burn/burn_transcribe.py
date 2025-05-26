import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
import serial
import time
ser = serial.Serial("/dev/ttyACM1", 115200)
duration = 1000
pressedUp = False
pressedDown = False
pressedBurn = False

class MinimalSubscriber(Node):
    
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Joy):
        # Example speed command {"BurnDurationMilis" : 1000}
        global duration, pressedUp, pressedDown, pressedBurn
        if msg.buttons[5] == 1 and not pressedUp:
            pressedUp = True
            duration += msg.buttons[5] * 500
            print("Duration: " + str(duration))
        elif msg.buttons[5] == 0 and pressedUp:
            pressedUp = False
        if msg.buttons[4] == 1 and not pressedDown and duration > 100:
            pressedDown = True
            duration -= msg.buttons[4] * 500
            print("Duration: " + str(duration))
        elif msg.buttons[4] == 0 and pressedDown:
            pressedDown = False
        if msg.buttons[2] == 1 and not pressedBurn:
            pressedBurn = True
            message = '{"BurnDurationMilis" :' + str(duration) + '}\n'
            print(message)
            ser.write(message.encode('utf-8'))
        elif msg.buttons[2] == 0 and pressedBurn:
            pressedBurn = False


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