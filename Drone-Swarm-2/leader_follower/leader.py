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
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=100)

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

        self.current_altitude = 0.0
        self.armed = False
        self.offboard_mode = False
        self.initial_pose = None
        self.initial_altitude = None

    def state_callback(self, msg):
        self.armed = msg.armed
        self.offboard_mode = (msg.mode == "OFFBOARD")

    def altitude_callback(self, msg):
        self.current_altitude = msg.pose.position.z
        self.get_logger().info(f"Altitude callback: {self.current_altitude}")
        if self.initial_pose is None:
            self.initial_pose = msg.pose
            self.initial_altitude = msg.pose.position.z
            self.get_logger().info(
                f"Initial position captured: x={self.initial_pose.position.x}, "
                f"y={self.initial_pose.position.y}, z={self.initial_altitude}"
            )

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

    def publish_position(self, altitude_offset):
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
            rclpy.spin_once(self, timeout_sec=0.02)
            self.get_logger().info("Waiting for initial position data...")
        
        if self.takeoff(1.0):
            time.sleep(3)  # Wait for stabilization
            ascent_offset = 2.0

            self.get_logger().info("Starting ascent to 2m above initial altitude...")
            while self.current_altitude < (self.initial_altitude + ascent_offset):
                rclpy.spin_once(self, timeout_sec=0.02)
                self.get_logger().info(f"Current altitude: {self.current_altitude}")
                self.publish_velocity(0.2)
                self.publish_position(ascent_offset)
                time.sleep(1)

            self.get_logger().info("Reached target altitude, starting descent...")
            while self.current_altitude > (self.initial_altitude + 0.2):
                rclpy.spin_once(self, timeout_sec=0.02)
                self.get_logger().info(f"Current altitude: {self.current_altitude}")
                self.publish_velocity(-0.2)
                self.publish_position(0.0)
                time.sleep(1)

            self.publish_velocity(0.0)
            self.get_logger().info("Mission complete!")

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
