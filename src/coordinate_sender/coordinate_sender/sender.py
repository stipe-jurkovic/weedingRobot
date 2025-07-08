import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re
import ast
import numpy as np
import cv2
import math

buffer_target = 9


# Camera parameters
scale = 10 / 824  # mm per pixel ≈ 0.012135
image_width = 4000
image_height = 3000

# Points: from camera mm → FluidNC mm
camera_mm_points = np.array([
    [-15.3 + 1, 11.3 - 1], # 
    [12.75 + 1, 11.5 - 1],
    [-15.5 + 1, -11.50 - 1],
    [12.75 + 1, -11.9 - 1]
], dtype=np.float32)
fluidnc_points = np.array([
    [280, 0],
    [0, 0],
    [280, 230],
    [0, 230]
], dtype=np.float32)

# Compute transform matrix (homography)
mm_to_fluidnc_matrix = cv2.getPerspectiveTransform(camera_mm_points, fluidnc_points)

def pixel_to_mm(u, v):
    """Converts pixel (u,v) to mm relative to center of image."""
    dx = u - image_width / 2
    dy = -(v - image_height / 2)
    x_mm = dx * scale
    y_mm = dy * scale
    return x_mm, y_mm

def mm_to_fluidnc_limited(x_mm, y_mm):
    pt = np.array([x_mm, y_mm, 1.0], dtype=np.float32)
    result = mm_to_fluidnc_matrix @ pt
    result /= result[2]
    x_fluidnc, y_fluidnc = result[0], result[1]

    # Define bounds from fluidnc_points
    x_min, x_max = 0, 280
    y_min, y_max = 0, 230

    if not (x_min <= x_fluidnc <= x_max) or not (y_min <= y_fluidnc <= y_max):
        return "out of range"
    return x_fluidnc, y_fluidnc

def pixel_to_fluidnc(u, v):
    x_mm, y_mm = pixel_to_mm(u, v)
    print(f"Pixel ({u}, {v}) in mm: ({x_mm:.2f}, {y_mm:.2f})")
    return mm_to_fluidnc_limited(x_mm, y_mm)

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
        self.burn_subscription = self.create_subscription(
            String,
            'points_to_burn',
            self.burn_response_callback,
            1
        )

        # Coordinates to send (FluidNC format)
        self.coordinates = []
        self.burn_commands = [
            "G1 F1",
            "M3 S10",  # Start laser
            "G4 P0.1",  # Wait for 200ms
            "M5",        # Stop laser
        ]
        self.messages = []
        
        self.planner_buffer = 15
        self.next_command = "$H"
        self.index = -1
        self.run = False
        self.timer = self.create_timer(0.1, self.send_next_command)
        self.timer = self.create_timer(0.1, self.poll)
        self.get_logger().info("CoordinatePublisher initialized")

    def send_next_command(self):
        if not self.run:
            return
        if self.index < len(self.messages) and self.planner_buffer > buffer_target:
            self.next_command = self.messages[self.index]  # G-code command
            self.index += 1
        else:
            print("ReachedEnd")
            self.run = False 
            self.index = 0 # Stop sending commands after reaching the end
            return
        
        msg = String()
        msg.data = self.next_command

        self.publisher.publish(msg)
        self.get_logger().info(f"Sent: {msg.data}")
        
    def burn_response_callback(self, msg: String):
        try:
            # Sigurno parsiraj string u listu tuple-ova
            coords = ast.literal_eval(msg.data)
            if not isinstance(coords, (list, tuple)):
                raise ValueError("Coords is not a list or tuple")
            corrected_coords = []
            for point in coords:
                if (not isinstance(point, (list, tuple)) or len(point) != 2):
                    raise ValueError(f"Invalid point format: {point}")
                x, y = point
                self.get_logger().info(f"Received point: x={x}, y={y}")
                dot = pixel_to_fluidnc(x, y)
                if dot != "out of range":
                    corrected_coords.append(dot)
            self.coordinates = sort_points_greedy(corrected_coords) # Update coordinates
            self.get_logger().info(f"Updated coordinates: {self.coordinates}")
            self.messages = []  # Clear previous messages
            for x, y in self.coordinates:
                # Move to coordinate
                self.messages.append(f"G0 X{x:.2f} Y{y:.2f}")
                # Add burn sequence
                self.messages.extend(self.burn_commands)
            self.messages.append(f"G0 X10 Y10")  # Return near home position
            self.messages.append("$H")  # Home
            print(f"Generated messages: {self.messages}")
            self.run = True  # Set run to True to start sending commands

        except (ValueError, SyntaxError) as e:
            self.get_logger().error(f"Failed to parse coordinates: {e}")

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
        if not self.run:
            return
        self.publisher.publish(String(data="?"))
        
def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def sort_points_greedy(points, start=(0,0)):
    points = points.copy()
    
    if len(points) < 2:
        return points
    
    # Find the nearest point to start, that becomes the initial current point
    current_point = min(points, key=lambda p: distance(p, start))
    sorted_path = [current_point]
    points.remove(current_point)
    
    while points:
        nearest = min(points, key=lambda p: distance(p, current_point))
        sorted_path.append(nearest)
        points.remove(nearest)
        current_point = nearest
    
    return sorted_path

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()