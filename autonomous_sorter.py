import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist

class AutonomousSorter(Node):
    def __init__(self):
        super().__init__('autonomous_sorter')

        # Publishers
        self.conveyor_pub = self.create_publisher(Float32, '/conveyor_cmd', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.ir_sub = self.create_subscription(Bool, '/ir_trigger', self.ir_callback, 10)

        # State Machine Configuration
        self.state = 'IDLE'
        self.active_timer = None
        self.empty_debounce_timer = None

        self.blind_dump_time = 3.0       # Minimum seconds to run conveyor at the bin
        self.required_clear_time = 2.0   # Seconds IR sensor must be clear to confirm empty

        self.get_logger().info("Sorter is IDLE. Waiting for trash to be loaded...")

    def _retire_active_timer(self):
        """Destroy the previous phase's timer instead of leaving it canceled-but-alive
        in the node's timer list. Only call this on a DIFFERENT timer than the one
        whose callback is currently executing."""
        if self.active_timer is not None:
            self.active_timer.cancel()
            self.destroy_timer(self.active_timer)
            self.active_timer = None

    def ir_callback(self, msg):
        trash_detected = msg.data

        # PHASE 1: Loading
        if trash_detected and self.state == 'IDLE':
            self.state = 'NAVIGATING_TO_BIN'
            self.get_logger().info("Payload detected! Navigating to the bin...")
            self.simulate_navigation_to_bin()

        # PHASE 5: Verifying Empty (Debouncing)
        elif self.state == 'VERIFYING_EMPTY':
            if not trash_detected:
                if self.empty_debounce_timer is None:
                    self.get_logger().info("Sensor clear. Verifying...")
                    self.empty_debounce_timer = self.create_timer(self.required_clear_time, self.finish_mission)
            else:
                if self.empty_debounce_timer is not None:
                    self.get_logger().info("Trash shifted! Resetting verification timer...")
                    self.empty_debounce_timer.cancel()
                    self.destroy_timer(self.empty_debounce_timer)
                    self.empty_debounce_timer = None

        # Note: We intentionally DO NOTHING with IR data in NAVIGATING or BLIND_DUMP states.

    def simulate_navigation_to_bin(self):
        # TODO: Replace with Nav2 Action Client (NavigateToPose)
        self.get_logger().info("[Nav2 SLAM Simulation] Driving for 5 seconds...")
        self._retire_active_timer()
        self.active_timer = self.create_timer(5.0, self.start_180_turn)

    def start_180_turn(self):
        self.active_timer.cancel()  # stop self from re-firing; destroyed on next transition
        self.state = 'TURNING_180'

        self.get_logger().info("Arrived at bin. Executing 180-degree turn...")

        # TODO: Replace with OpenCV alignment or precise Odometry turn
        turn_msg = Twist()
        turn_msg.angular.z = 1.0  # Spin in place
        self.cmd_vel_pub.publish(turn_msg)

        # Simulate taking 3 seconds to complete the turn
        self._retire_active_timer()
        self.active_timer = self.create_timer(3.0, self.start_blind_dump)

    def start_blind_dump(self):
        self.active_timer.cancel()

        # Stop turning
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        self.state = 'BLIND_DUMP'
        self.get_logger().info(f"Turn complete. Blind dumping for {self.blind_dump_time}s to clear shifted trash...")

        # Start Conveyor
        self.conveyor_pub.publish(Float32(data=700.0))

        # Run blindly for a set time before trusting the sensor again
        self._retire_active_timer()
        self.active_timer = self.create_timer(self.blind_dump_time, self.enter_verification_mode)

    def enter_verification_mode(self):
        self.active_timer.cancel()
        self.state = 'VERIFYING_EMPTY'
        self.get_logger().info("Blind dump finished. Checking IR sensor for remaining payload...")
        self._retire_active_timer()
        # The ir_callback actively handles the VERIFYING_EMPTY state

    def finish_mission(self):
        if self.empty_debounce_timer:
            self.empty_debounce_timer.cancel()  
            self.empty_debounce_timer = None

        self.get_logger().info("Payload successfully cleared! Returning to base...")

        # Stop conveyor
        self.conveyor_pub.publish(Float32(data=0.0))

        self.state = 'NAVIGATING_HOME'

        # TODO: Replace with Nav2 Action Client back to home coordinates
        self.active_timer = self.create_timer(4.0, self.reset_to_idle)

    def reset_to_idle(self):
        self.active_timer.cancel()
        self.state = 'IDLE'
        self.get_logger().info("Arrived at base. Sorter is IDLE. Ready for next payload.")
        self._retire_active_timer()

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousSorter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Failsafe: Stop all motors on shutdown
        node.conveyor_pub.publish(Float32(data=0.0))
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
