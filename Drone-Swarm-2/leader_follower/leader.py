import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode
from mavros_msgs.msg import State
import time
import threading
import sys

# Global lock to protect spin_until_future_complete calls
spin_lock = threading.Lock()

class DroneController(Node):
    def __init__(self, drone_ns):
        super().__init__(f'{drone_ns}_controller')
        self.drone_ns = drone_ns
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # Initialize attributes
        self.current_altitude = 0.0
        self.initial_altitude = None
        self.current_x = 0.0  # Current x position
        self.initial_x = None  # Initial x position (reference)
        self.armed = False
        self.offboard_mode = False
        self.initial_pose = None

        # Create service clients, including a SetMode client for switching modes
        self.takeoff_client = self.create_client(CommandTOL, f'mavros/{drone_ns}/cmd/takeoff')
        self.disarm_client = self.create_client(CommandBool, f'mavros/{drone_ns}/cmd/arming')
        self.mode_client = self.create_client(SetMode, f'mavros/{drone_ns}/set_mode')
        
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for takeoff service...')
        while not self.disarm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for disarm service...')
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')

        self.velocity_publisher = self.create_publisher(TwistStamped, f'mavros/{drone_ns}/setpoint_velocity/cmd_vel', qos_profile)
        self.position_publisher = self.create_publisher(PoseStamped, f'mavros/{drone_ns}/setpoint_position/local', qos_profile)
        self.state_subscriber = self.create_subscription(State, f'mavros/{drone_ns}/state', self.state_callback, qos_profile)
        self.altitude_subscriber = self.create_subscription(PoseStamped, f'mavros/{drone_ns}/local_position/pose', self.altitude_callback, qos_profile)

    def state_callback(self, msg):
        self.armed = msg.armed
        self.offboard_mode = (msg.mode == "OFFBOARD")

    def altitude_callback(self, msg):
        self.current_altitude = msg.pose.position.z
        self.current_x = msg.pose.position.x  # Update current x position dynamically
        if self.initial_pose is None:
            self.initial_pose = msg.pose
            self.initial_altitude = msg.pose.position.z
            self.initial_x = msg.pose.position.x  # Set initial x position as reference

    def takeoff(self, altitude=1.0):
        req = CommandTOL.Request()
        req.altitude = altitude
        future = self.takeoff_client.call_async(req)
        with spin_lock:
            rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f"Takeoff successful to {altitude} meters")
            return True
        else:
            self.get_logger().info("Takeoff failed")
            return False

    def publish_velocity(self, z_velocity):
        msg = TwistStamped()
        msg.twist.linear.z = z_velocity
        self.velocity_publisher.publish(msg)

    def publish_position(self, altitude_offset, x_offset=0.0):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.initial_pose is None:
            self.get_logger().error("Initial pose not received. Exiting script.")
            rclpy.shutdown()
            sys.exit(1)
        else:
            # Calculate the target position
            msg.pose.position.x = self.initial_pose.position.x + x_offset
            msg.pose.position.y = self.initial_pose.position.y
            msg.pose.position.z = self.initial_altitude + altitude_offset

            # Print both the current position and the target position
            current_x = self.initial_pose.position.x
            current_y = self.initial_pose.position.y
            current_z = self.current_altitude  # Use the latest altitude from the callback
            print(f"Current position: x={current_x}, y={current_y}, z={current_z}")
            print(f"Publishing target position: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}")

        self.position_publisher.publish(msg)

    def publish_velocity_x(self, x_velocity):
        msg = TwistStamped()
        msg.twist.linear.x = x_velocity
        self.velocity_publisher.publish(msg)

    def switch_to_stabilize_mode(self):
        self.get_logger().info("Switching to STABILIZE mode for disarm...")
        req = SetMode.Request()
        req.custom_mode = "STABILIZE"
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info("STABILIZE mode set successfully.")
        else:
            self.get_logger().error("Failed to set STABILIZE mode.")
        time.sleep(2)  # Allow time for the mode switch to take effect

    def disarm(self):
        req = CommandBool.Request()
        req.value = False  # False to disarm
        future = self.disarm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info("Drone disarmed successfully")
            return True
        else:
            self.get_logger().info("Drone disarm failed")
            return False

    def execute_mission(self):
        while self.initial_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info("Waiting for initial position data...")
            time.sleep(0.1)
        
        if self.takeoff(1.0):  # Changed from 1.0 to 0.5
            self.get_logger().info("Holding at 0.5m for stabilization...")
            time.sleep(12)
            ascent_offset = 2.0

            self.get_logger().info("Starting ascent to 2m above initial altitude...")
            while self.current_altitude < (self.initial_altitude + ascent_offset):
                rclpy.spin_once(self, timeout_sec=0.1)
                self.publish_position(ascent_offset)  # Only publish position
                time.sleep(1)

            self.get_logger().info("Reached 2m, holding position...")
            time.sleep(4)

            # Move 3 meters in the +x direction
            self.get_logger().info("Starting movement in +x direction...")
            target_x = self.initial_x + 3.0  # Use self.initial_x as the reference
            while self.current_x < target_x:  # Use self.current_x for the condition
                rclpy.spin_once(self, timeout_sec=0.1)
                self.get_logger().info(f"Current x position: {self.current_x}, Target x position: {target_x}")
                self.publish_position(ascent_offset, x_offset=3.0)  # Only publish position
                time.sleep(1)

            self.get_logger().info("Reached target x position, holding position...")
            time.sleep(4)

            # Start descent
            self.get_logger().info("Starting descent...")
            while self.current_altitude > (self.initial_altitude + 0.2):
                rclpy.spin_once(self, timeout_sec=0.1)
                self.publish_position(altitude_offset=0.0)  # Only publish position
                time.sleep(0.1)

            # Ensure the drone has landed
            self.get_logger().info("Drone has landed. Mission complete!")

            # Switch to STABILIZE mode to allow disarming
            self.switch_to_stabilize_mode()
            self.disarm()
        return self.drone_ns

def main(args=None):
    rclpy.init(args=args)
    drone = DroneController("uas_1_1")
    try:
        drone.execute_mission()
    except KeyboardInterrupt:
        pass
    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
