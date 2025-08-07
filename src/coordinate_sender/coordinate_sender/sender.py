import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ast
import math
from srvs_and_msgs.srv import SetBurnLength
from collections import deque
from .translatingPixelToPoint import pixel_to_fluidnc
from .sortPoints import sort_points_greedy
 
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

    def send_next_command(self):
        if not self.run: 
            return # If run is not set to true don't send anything
        if (self.number_of_oks + self.buffer_size) < self.number_of_sent_messages:
            return # Wait for enough OKs before sending more commands
        
        if self.burn_index < len(self.burn_commands) and self.next_burn == True:
            self.publish_message(self.burn_commands[self.burn_index])
            self.burn_index += 1  # Increment index
            return
        
        if self.index < len(self.messages):
            next_msg = self.messages[self.index]
            if isinstance(next_msg, tuple) and len(next_msg) == 2:
                self.next_command, burn = next_msg
            else:
                self.next_command = next_msg
                burn = False
            self.publish_message(self.next_command)
            self.next_burn = burn
            if burn: self.burn_index = 0
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
                if coord is not None: corrected_coords.append(coord)

            self.load_coordinates_into_queue(corrected_coords)

            self.run = True
            self.index = 0
            self.get_logger().info("Started processing coordinate job.")

        except (ValueError, SyntaxError) as e:
            self.get_logger().error(f"Failed to parse coordinates: {e}")
            
    def load_coordinates_into_queue(self, coordinates: list):
        sortedCoordinates = sort_points_greedy(coordinates) # Update coordinates
        self.messages.clear() # Clear previous messages
        self.messages.append(("$H", False))  # Home
        for x, y in sortedCoordinates:
            self.messages.append((f"G0 X{x:.2f} Y{y:.2f}", True)) # Move to coordinate
        self.messages.append(("G0 X10 Y10", False))  # Return near home position
        self.messages.append(("$H", False))  # Home

    def set_burn_length(self, ms, response):
        """Set the burn length in milliseconds."""
        self.burn_ms = ms.milis
        self.burn_commands = get_burn_commands(self.burn_ms)
        self.get_logger().info(f"Burn length set to {self.burn_ms} ms")
        response.successful = True
        return response  # Indicate success

    def response_callback(self, msg: String):
        #self.get_logger().info(f"Received response: {msg.data}")
        
        if msg.data.strip() == "[MSG:Homed:XY]" and self.run is False:
            self.number_of_oks = -1
            self.number_of_sent_messages = 0
            self.get_logger().info("Homing complete — counters reset.")

        if msg.data.strip() == "ok" and self.number_of_sent_messages > 1:
            self.number_of_oks += 1
            self.get_logger().info(f"[Status] OKs received: {self.number_of_oks}, Commands sent: {self.number_of_sent_messages}")



def main(args=None):
    rclpy.init(args=args)
    node = CoordinatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

def get_burn_commands(burn_ms):
    """Generates burn commands with the specified burn time in milliseconds."""
    return [
        "G1 F1",
        "M3 S10",  # Start laser
        "G4 P" + f"{burn_ms / 1000:.3f}",  # Wait for xxx s
        "M5",        # Stop laser
    ]
