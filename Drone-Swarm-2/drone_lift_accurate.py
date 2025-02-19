import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode  # Added SetMode import
from mavros_msgs.msg import State
import time
import sys


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Declare a parameter for namespace (default is "drone1")
        self.declare_parameter('namespace', 'drone1')
        namespace = self.get_parameter('namespace').value

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # Create service clients with the namespace
        self.takeoff_client = self.create_client(CommandTOL, f'/{namespace}/cmd/takeoff')
        self.disarm_client = self.create_client(CommandBool, f'/{namespace}/cmd/arming')
        self.mode_client = self.create_client(SetMode, f'/{namespace}/set_mode')  # Mode switching client

        # Wait for the services to be available
        while not self.takeoff_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for takeoff service...')
        while not self.disarm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for disarm service...')
        while not self.mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for set_mode service...')

        # Create publishers and subscribers using the namespace
        self.velocity_publisher = self.create_publisher(
            TwistStamped, f'/{namespace}/setpoint_velocity/cmd_vel', qos_profile)
        self.position_publisher = self.create_publisher(
            PoseStamped, f'/{namespace}/setpoint_position/local', qos_profile)
        self.state_subscriber = self.create_subscription(
            State, f'/{namespace}/state', self.state_callback, qos_profile)
        self.altitude_subscriber = self.create_subscription(
            PoseStamped, f'/{namespace}/local_position/pose', self.altitude_callback, qos_profile)

        self.current_altitude = 0.0
        self.armed = False
        self.offboard_mode = False

        # Variables to store the initial position (if not (0,0,0))
        self.initial_pose = None  # Will hold the full Pose message
        self.initial_altitude = None  # Initial z value

    def state_callback(self, msg):
        self.armed = msg.armed
        self.offboard_mode = (msg.mode == "OFFBOARD")

    def altitude_callback(self, msg):
        self.current_altitude = msg.pose.position.z
        self.get_logger().info(f"Altitude callback: {self.current_altitude}")
        # If we haven't stored the initial position yet, do it now.
        if self.initial_pose is None:
            self.initial_pose = msg.pose
            self.initial_altitude = msg.pose.position.z
            self.get_logger().info(
                f"Initial position captured: x={self.initial_pose.position.x}, "
                f"y={self.initial_pose.position.y}, z={self.initial_altitude}")

    def takeoff(self, altitude=1.0):
        req = CommandTOL.Request()
        # For takeoff, the altitude is given as an absolute value.
        # Here, we assume the flight controller understands it as an absolute altitude.
        req.altitude = altitude
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f"Takeoff successful to {altitude} meters")
            return True
        else:
            self.get_logger().info("Takeoff failed")
            return False

    def publish_velocity(self, z_velocity):
        msg = TwistStamped()
        msg.twist.linear.z = z_velocity
        self.velocity_publisher.publish(msg)

    def publish_position(self, altitude_offset):
        """
        Publish a position setpoint based on the initial position.
        The new target altitude is calculated as initial altitude + altitude_offset.
        The x and y positions remain as the initial x and y.
        """
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.initial_pose is None:
            self.get_logger().error("Initial pose not received. Exiting script.")
            rclpy.shutdown()
            sys.exit(1)
        else:
            msg.pose.position.x = self.initial_pose.position.x
            msg.pose.position.y = self.initial_pose.position.y
            msg.pose.position.z = self.initial_altitude + altitude_offset
        self.position_publisher.publish(msg)

    def switch_to_stabilize_mode(self):
        """
        Switch the flight mode to STABILIZE to allow disarming.
        """
        self.get_logger().info("Switching to STABILIZE mode for disarm...")
        req = SetMode.Request()
        req.custom_mode = "STABILIZE"
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info("STABILIZE mode set successfully.")
        else:
            self.get_logger().error("Failed to set STABILIZE mode.")
        time.sleep(2)  # Wait for the mode switch to take effect

    def disarm(self):
        req = CommandBool.Request()
        req.value = False  # False to disarm
        future = self.disarm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info("Drone disarmed successfully")
            return True
        else:
            self.get_logger().info("Drone disarm failed")
            return False

    def execute_mission(self):
        # Wait until we have received the initial position before starting the mission.
        while self.initial_pose is None:
            self.get_logger().info("Waiting for initial position data...")
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        # Optionally, you could adjust the takeoff altitude here based on initial altitude.
        if self.takeoff(1.0):
            time.sleep(5)  # Wait for stabilization

            # Define target altitude offsets (relative to the initial altitude)
            takeoff_offset = 1.0   # Already commanded via CommandTOL (for informational purposes)
            ascent_offset = 2.0    # Target offset during ascent
            descent_offset = 0.0   # Return to initial altitude

            self.get_logger().info(f"Starting ascent to {ascent_offset} m above initial altitude...")
            # Ascend until current altitude reaches (initial_altitude + ascent_offset)
            while self.current_altitude < (self.initial_altitude + ascent_offset):
                rclpy.spin_once(self, timeout_sec=0.1)
                self.get_logger().info(f"Current altitude: {self.current_altitude}")
                self.publish_velocity(0.2)
                self.publish_position(ascent_offset)
                time.sleep(1)

            self.get_logger().info("Reached target altitude, starting descent...")
            # Descend until current altitude is near the initial altitude
            while self.current_altitude > (self.initial_altitude + 0.2):
                rclpy.spin_once(self, timeout_sec=0.1)
                self.get_logger().info(f"Current altitude: {self.current_altitude}")
                self.publish_velocity(-0.2)
                self.publish_position(descent_offset)
                time.sleep(1)

            self.publish_velocity(0.0)
            self.get_logger().info("Mission complete!")

            # Switch to a mode that permits disarming before attempting to disarm
            self.switch_to_stabilize_mode()
            self.disarm()


def main(args=None):
    rclpy.init(args=args)
    controller = DroneController()
    controller.execute_mission()
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
