
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32
import time

class AutonomousSorter(Node):
    def __init__(self):
        super().__init__('autonomous_sorter')

        # Publishers to control the ESP32 hardware
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dump_pub = self.create_publisher(Float32, '/dump_cmd', 10)
        self.conv_pub = self.create_publisher(Float32, '/conveyor_cmd', 10)

        # Subscriber to read the I2C Color Sensor
        self.color_sub = self.create_subscription(Vector3, '/trash_color', self.color_callback, 10)

        self.sorting_in_progress = False
        self.get_logger().info("Autonomous Sorter Node Initialized. Cruising and looking for target bin...")

    def color_callback(self, msg):
        # Ignore sensor readings if we are already in the middle of a sorting sequence
        if self.sorting_in_progress:
            return

        # Simple thresholding logic: Looking for a "Red" bin
        # Adjust these raw RGB values based on your TCS34725 physical testing
        if msg.x > 150 and msg.y < 80 and msg.z < 80:
            self.get_logger().info(f"Target Red Bin Detected (RGB: {msg.x}, {msg.y}, {msg.z})! Initiating sort sequence.")
            self.sorting_in_progress = True
            self.execute_sort_sequence()

    def execute_sort_sequence(self):
        # 1. Stop the robot completely
        self.publish_twist(0.0, 0.0)
        time.sleep(1.0)

        # 2. Tilt chassis up (Move Dump Servo to 60 degrees)
        self.get_logger().info("Tilting chassis to 60 degrees...")
        self.publish_float(self.dump_pub, 60.0)
        time.sleep(2.0)

        # 3. Run conveyor to eject the sorted waste
        self.get_logger().info("Running conveyor to eject waste...")
        self.publish_float(self.conv_pub, 800.0) # 800 steps/sec FWD
        time.sleep(4.0)

        # Stop conveyor
        self.publish_float(self.conv_pub, 0.0)

        # 4. Lower chassis back to the resting position (0 degrees)
        self.get_logger().info("Lowering chassis...")
        self.publish_float(self.dump_pub, 0.0)
        time.sleep(2.0)

        # 5. Pull away from the bin (Reverse straight back)
        self.get_logger().info("Pulling away from bin...")
        self.publish_twist(-0.3, 0.0)
        time.sleep(1.5)
        self.publish_twist(0.0, 0.0)

        self.get_logger().info("Sorting complete. Resuming navigation state.")
        self.sorting_in_progress = False

    def publish_twist(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def publish_float(self, publisher, value):
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousSorter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped manually.")
    finally:
        node.publish_twist(0.0, 0.0) # Ensure motors stop on exit
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()