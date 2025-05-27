import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
import serial
import time
ser = serial.Serial("/dev/serial/by-path/platform-xhci-hcd.1-usb-0:2.1:1.0-port0", 115200)

homeWasPressed = False
burnWasPressed = False

def burn(it):
    it.get_logger().info("Burn button pressed, burning for 1 second")
    ser.write(b'G1 F1\n')
    ser.write(b'M3 S200\n')
    time.sleep(1)
    ser.write(b'M5 S0\n')

def home(it):
    it.get_logger().info("Home button pressed, sending home command")
    ser.write(b'$H\n')
    
# Example jog command $J=G91 G21 x-5 f4000
def jog(it, X, Y):
    it.get_logger().info("Jogging X: " + str(X) + ", Y: " + str(Y))
    message = '$J=G91 G21 X' + str(X) + ' Y' + str(Y) + ' F4000 \n'
    time.sleep(0.1)
    ser.write(message.encode('utf-8'))

class MinimalSubscriber(Node):
    
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Joy,'joy',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Joy):
        global homeWasPressed, burnWasPressed
        
        X = msg.axes[4] * -5
        Y = msg.axes[5] * 5
        burnPressed = msg.buttons[1] == 1
        homePressed = msg.buttons[3] == 1
        if homePressed and not homeWasPressed:
            homeWasPressed = True
            home(self)
        elif not homePressed:
            homeWasPressed = False
        
        if burnPressed and not burnWasPressed:
            burnWasPressed = True
            burn(self)
        elif not burnPressed:
            burnWasPressed = False
        
        if X != 0 or Y != 0:    
            jog(self, X, Y)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()