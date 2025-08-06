import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re
import ast
import numpy as np
import cv2
import math
from srvs_and_msgs.srv import SetBurnLength
from collections import deque
import time


# Camera parameters
scale = 10 / 824  # mm per pixel ≈ 0.012135
image_width = 4000
image_height = 3000

def get_burn_commands(burn_ms):
    """Generates burn commands with the specified burn time in milliseconds."""
    return [
        "G1 F1",
        "M3 S10",  # Start laser
        "G4 P" + f"{burn_ms / 1000:.3f}",  # Wait for xxx s
        "M5",        # Stop laser
    ]

# Points: from camera mm → FluidNC mm
shift_x = 0.3  # Shift in mm to align with FluidNC coordinates
shift_y = -0.6  # Shift in mm to align with FluidNC coordinates
camera_mm_points = np.array([
    [-15.2 + shift_x, 11.3 + shift_y], # 
    [12.75 + shift_x, 11.5 + shift_y],
    [-15.5 + shift_x, -11.5 + shift_y],
    [12.75 + shift_x, -11.6 + shift_y]
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
        self.publisher = self.create_publisher(String, 'stepper_control', 30)

        # Stepper response subscriber
        self.subscription = self.create_subscription(
            String,
            'stepper_control_response',
            self.response_callback,
            30
        )
        # Points to burn subscriber
        self.burn_subscription = self.create_subscription(
            String,
            'points_to_burn',
            self.burn_response_callback,
            1
        )
        self.coord_queue = deque()
        
        self.set_burn_length = self.create_service(SetBurnLength, 'set_burn_length', self.set_burn_length)
        self.burn_ms = 100  # Burn time in milliseconds
        self.buffer_size = 50

        self.burn_commands = get_burn_commands(self.burn_ms)
        self.messages = []
        self.next_command = ""
        self.index = -1
        self.burn_index = -1
        self.next_burn = False
        self.run = False
        self.number_of_oks = 0
        self.number_of_sent_messages= 0
        
        self.sendTimer = self.create_timer(0.1, self.send_next_command)
        self.get_logger().info("CoordinatePublisher initialized")

    def set_burn_length(self, ms, response):
        """Set the burn length in milliseconds."""
        self.burn_ms = ms.milis
        self.burn_commands = get_burn_commands(self.burn_ms)
        self.get_logger().info(f"Burn length set to {self.burn_ms} ms")
        response.successful = True
        return response  # Indicate success

    def send_next_command(self):
        if not self.run: 
            return # If run is not set to true don't send anything
        if (self.number_of_oks + self.buffer_size) < self.number_of_sent_messages:
            return # Wait for enough OKs before sending more commands
        
        if self.burn_index < len(self.burn_commands) and self.next_burn == True:
            self.publish_message(self.burn_commands[self.burn_index])
            self.burn_index += 1  # Increment index
        elif self.index < len(self.messages):
            next_msg = self.messages[self.index]
            if isinstance(next_msg, tuple) and len(next_msg) == 2:
                self.next_command, burn = next_msg
            else:
                self.next_command = next_msg
                burn = False
            self.publish_message(self.next_command)
            self.next_burn = burn
            if burn:
                self.burn_index = 0
            self.index += 1  # Increment index
        else:
            self.get_logger().info("ReachedEnd")
            self.run = False 
            self.next_burn = False
            self.index = 0 # Stop sending commands after reaching the end
            
            if self.coord_queue: # Dequeue next coordinate job if available
                next_data = self.coord_queue.popleft()
                self.process_coords(next_data)
            
            return
        
                
    def publish_message(self, msg: str):
        self.number_of_sent_messages +=1
        self.publisher.publish(String(data=msg))
        self.get_logger().info(f"Sent: {msg}")

    def burn_response_callback(self, msg: String):
        self.coord_queue.append(msg.data)
        self.get_logger().info(f"Queued new coordinate set, queue size: {len(self.coord_queue)}")

        if not self.run:
            next_data = self.coord_queue.popleft()
            self.process_coords(next_data)
    
    def process_coords(self, data: str):
        try:
            coords = ast.literal_eval(data)
            if not isinstance(coords, (list, tuple)):
                raise ValueError("Coords is not a list or tuple")

            corrected_coords = []
            for point in coords:
                if not isinstance(point, (list, tuple)) or len(point) != 2:
                    raise ValueError(f"Invalid point format: {point}")
                coord = pixel_to_fluidnc(*point)
                if coord != "out of range":
                    corrected_coords.append(coord)

            sortedCoordinates = sort_points_greedy(corrected_coords) # Update coordinates
            
            self.messages.clear() # Clear previous messages
            self.messages.append(("$H", False))  # Home
            for x, y in sortedCoordinates:
                self.messages.append((f"G0 X{x:.2f} Y{y:.2f}", True)) # Move to coordinate
            self.messages.append(("G0 X10 Y10", False))  # Return near home position
            self.messages.append(("$H", False))  # Home

            self.run = True
            self.index = 0
            self.get_logger().info("Started processing coordinate job.")

        except (ValueError, SyntaxError) as e:
            self.get_logger().error(f"Failed to parse coordinates: {e}")

    def response_callback(self, msg: String):
        self.get_logger().info(f"Received response: {msg.data}")
        
        if "ok" in msg.data:
            self.number_of_oks += 1
            print(f"oks: {self.number_of_oks}, sent commands: {self.number_of_sent_messages}")


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        
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