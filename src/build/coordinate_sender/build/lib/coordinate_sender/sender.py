import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CoordinatePublisher(Node):

    def __init__(self):
        super().__init__('coordinate_publisher_waits')

        # Stepper control publisher
        self.publisher = self.create_publisher(String, 'stepper_control', 10)

        # Stepper response subscriber
        self.subscription = self.create_subscription(
            String,
            'stepper_control_response',
            self.response_callback,
            10
        )

        # Coordinates to send (FluidNC format)
        self.coordinates = [
            (26.98, 82.90),
            (27.46, 80.02),
            (53.76, 171.56),
            (55.46, 153.76),
            (58.45, 142.17),
            (58.89, 96.01),
            (59.03, 84.96),
            (59.16, 138.82),
            (59.28, 142.54),
            (59.39, 146.49),
            (60.38, 1.33),
            (60.85, 187.99),
            (61.29, 85.32),
            (61.40, 88.56),
            (61.77, 85.44)
        ]
        self.burn_commands = [
            "M3 S1000",  # Start spindle at 1000 RPM
            "G4 P200",  # Wait for 5 seconds
            "M5",        # Stop spindle
        ]

        self.index = 0
        self.waiting_for_response = False
        self.timer = self.create_timer(0.10, self.send_next_coordinate)

    def send_next_coordinate(self):
        if self.waiting_for_response or self.index >= len(self.coordinates):
            return

        x, y = self.coordinates[self.index]
        msg = String()
        msg.data = f"G0 X{x:.2f} Y{y:.2f}"  # G-code command

        self.publisher.publish(msg)
        self.get_logger().info(f"Sent: {msg.data}")
        
        for command in self.burn_commands:
            burn_msg = String()
            burn_msg.data = command
            self.publisher.publish(burn_msg)
            self.get_logger().info(f"Sent burn command: {command}")

        self.waiting_for_response = True  # Block until response comes in

    def response_callback(self, msg: String):
        self.get_logger().info(f"Received response: {msg.data}")

        if self.waiting_for_response:
            self.index += 1
            self.waiting_for_response = False

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()