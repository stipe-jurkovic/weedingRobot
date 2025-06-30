import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re

buffer_target = 9

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
            (60.38, 1.33),
            (62.27, 11.70),
            (73.94, 27.10),
            (75.14, 27.33),
            (69.73, 41.72),
            (69.73, 42.32),
            (69.84, 44.13),
            (70.08, 44.13),
            (69.72, 45.58),
            (69.60, 46.06),
            (83.71, 1.16),
            (27.46, 80.02),
            (26.98, 82.90),
            (72.91, 62.45),
            (72.91, 62.45),
            (59.03, 84.96),
            (61.29, 85.32),
            (61.77, 85.44),
            (81.14, 68.22),
            (61.40, 88.56),
            (99.29, 46.94),
            (90.72, 66.17),
            (85.91, 72.44),
            (99.52, 52.38),
            (58.89, 96.01),
            (112.79, 4.96),
            (70.58, 88.33),
            (70.70, 88.33),
            (111.99, 26.68),
            (112.10, 29.71),
            (114.43, 19.39),
            (70.08, 98.55),
            (70.08, 98.55),
            (108.74, 60.95),
            (101.24, 78.23),
            (85.48, 98.70),
            (76.84, 114.19),
            (76.96, 114.19),
            (93.83, 107.27),
            (123.75, 72.79),
            (109.20, 101.65),
            (109.80, 102.25),
            (59.16, 138.82),
            (58.45, 142.17),
            (111.71, 106.11),
            (111.59, 106.35),
            (59.28, 142.54),
            (112.30, 107.56),
            (68.80, 139.48),
            (68.80, 139.48),
            (59.39, 146.49),
            (108.30, 118.16),
            (110.58, 118.77),
            (55.46, 153.76),
            (158.95, 44.83),
            (159.69, 43.61),
            (130.62, 103.02),
            (163.23, 38.85),
            (152.64, 71.32),
            (135.57, 100.37),
            (146.96, 93.50),
            (53.76, 171.56),
            (67.31, 169.65),
            (67.42, 169.65),
            (114.09, 144.80),
            (91.40, 161.82),
            (138.73, 123.71),
            (92.81, 169.15),
            (192.59, 21.53),
            (190.14, 42.78),
            (133.09, 144.19),
            (180.79, 77.02),
            (60.85, 187.99),
            (170.44, 101.30),
            (198.34, 1.45),
            (170.80, 101.54),
            (120.77, 158.32),
            (176.43, 94.03),
            (200.86, 10.36),
            (201.23, 29.19),
            (177.12, 101.19),
            (203.95, 5.45),
            (204.00, 17.68),
            (178.95, 101.68),
            (194.85, 71.90),
            (84.64, 190.27),
            (84.63, 192.54),
            (168.47, 127.83),
            (152.12, 153.36),
            (162.58, 142.92),
            (215.39, 37.32),
            (76.50, 204.89),
            (76.02, 205.48),
            (76.02, 205.48),
            (142.25, 172.70),
            (142.85, 172.82),
            (224.21, 5.44),
            (144.30, 172.60),
            (73.27, 215.35),
            (73.27, 215.35),
            (220.15, 58.82),
            (223.52, 50.87),
            (223.52, 50.87),
            (139.91, 183.39),
            (225.12, 51.48),
            (168.31, 159.40),
            (232.42, 11.52),
            (233.80, 8.07),
            (228.19, 51.84),
            (126.01, 200.43),
            (161.45, 173.23),
            (154.16, 180.88),
            (154.50, 186.07),
            (241.63, 16.01),
            (193.77, 147.72),
            (132.69, 210.60),
            (138.36, 207.90),
            (163.91, 188.81),
            (189.89, 165.50),
            (233.93, 106.93),
            (257.97, 12.96),
            (244.57, 83.25),
            (259.16, 2.99),
            (259.27, 5.21),
            (246.05, 82.77),
            (245.35, 94.13),
            (219.29, 147.64),
            (262.07, 40.47),
            (157.26, 214.27),
            (231.53, 132.63),
            (267.37, 14.88),
            (171.44, 207.22),
            (264.69, 52.98),
            (264.93, 54.45),
            (172.89, 208.32),
            (249.56, 106.60),
            (174.33, 209.66),
            (245.84, 125.63),
            (201.90, 190.90),
            (267.18, 81.06),
            (253.36, 125.18),
            (190.59, 210.47),
            (264.64, 104.20),
            (200.85, 203.58),
            (200.85, 203.70),
            (257.67, 125.19),
            (267.47, 105.06),
            (268.20, 106.04),
            (224.68, 180.95),
            (269.31, 121.21),
            (265.05, 143.18),
            (260.23, 173.50),
            (219.21, 229.93),
            (276.73, 163.64),
            (277.93, 167.92),
            (263.79, 189.72),
            (239.08, 221.60),
            (264.27, 191.43),
            (257.54, 201.32),
            (250.18, 214.35),
            (250.41, 216.17),
            (250.41, 216.30),
            (274.56, 200.67),

        ]
        self.burn_commands = [
            "G1 F1",
            "M3 S10",  # Start laser
            "G4 P1",  # Wait for 200ms
            "M5",        # Stop laser
        ]
        
        self.messages = []
        
        for x, y in self.coordinates:
            # Move to coordinate
            self.messages.append(f"G0 X{x:.2f} Y{y:.2f}")
            # Add burn sequence
            self.messages.extend(self.burn_commands)
        
        self.planner_buffer = 15
        self.next_command = "$H"
        self.index = -1
        self.waiting_for_response = False
        self.timer = self.create_timer(0.1, self.send_next_command)
        self.timer = self.create_timer(0.1, self.poll)

    def send_next_command(self):
        if self.index < len(self.messages) - 1 and self.planner_buffer > buffer_target:
            self.index += 1
            self.next_command = self.messages[self.index]  # G-code command
        else:
            print("ReachedEnd")
            return
        
        msg = String()
        msg.data = self.next_command

        self.publisher.publish(msg)
        self.get_logger().info(f"Sent: {msg.data}")

    def response_callback(self, msg: String):
        self.get_logger().info(f"Received response: {msg.data}")
        
        match = re.search(r'Bf:(\d+),(\d+)', msg.data)
        if match:
            planner_buffer = int(match.group(1))
            rx_buffer = int(match.group(2))
            self.planner_buffer = planner_buffer
            self.get_logger().info(f"Planner buffer (queue depth): {planner_buffer}")
            #self.get_logger().info(f"RX buffer: {rx_buffer}")
        else:
            self.get_logger().info("Bf not found in status line.")
    def poll(self):
        self.publisher.publish(String(data="?"))
def main(args=None):
    rclpy.init(args=args)
    node = CoordinatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()