import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.srv import CommandTOL, CommandBool
from mavros_msgs.msg import State
import time
import threading
from multiprocessing.pool import ThreadPool
from tqdm import tqdm
import sys

# Global lock to protect spin_until_future_complete calls
spin_lock = threading.Lock()

class DroneController(Node):
    def __init__(self, drone_ns):
        super().__init__(f'{drone_ns}_controller')
        self.drone_ns = drone_ns
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.takeoff_client = self.create_client(CommandTOL, f'/{drone_ns}/cmd/takeoff')
        self.disarm_client = self.create_client(CommandBool, f'/{drone_ns}/cmd/arming')
        self.velocity_publisher = self.create_publisher(TwistStamped, f'/{drone_ns}/setpoint_velocity/cmd_vel', qos_profile)
        self.position_publisher = self.create_publisher(PoseStamped, f'/{drone_ns}/setpoint_position/local', qos_profile)
        self.state_subscriber = self.create_subscription(State, f'/{drone_ns}/state', self.state_callback, qos_profile)
        self.altitude_subscriber = self.create_subscription(PoseStamped, f'/{drone_ns}/local_position/pose', self.altitude_callback, qos_profile)

        self.current_altitude = 0.0
        self.armed = False
        self.offboard_mode = False

        # Variables to store the initial position (if not (0,0,0))
        self.initial_pose = None  # Will hold the full Pose message
        self.initial_altitude = None  # Initial z value

        while not self.takeoff_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for takeoff service...')
        while not self.disarm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for disarm service...')

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

    def disarm(self):
        req = CommandBool.Request()
        req.value = False
        future = self.disarm_client.call_async(req)
        with spin_lock:
            rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info("Drone disarmed successfully")
            return True
        else:
            self.get_logger().info("Drone disarm failed")
            return False

    def execute_mission(self):
        # Wait until we have received the initial position before starting the mission.
        while self.initial_pose is None:
            self.get_logger().info("Waiting for initial position data...")
            time.sleep(0.1)
            
        if self.takeoff(1.0):
            time.sleep(5)  # Wait for stabilization

            ascent_offset = 2.0

            self.get_logger().info("Starting ascent to 2m...")
            while self.current_altitude < (self.initial_altitude + ascent_offset):
                rclpy.spin_once(self, timeout_sec=0.1)
                self.get_logger().info(f"Current altitude: {self.current_altitude}")
                self.publish_velocity(0.2)
                self.publish_position(2.0)
                time.sleep(1)

            self.get_logger().info("Reached 2m, starting descent...")
            while self.current_altitude > (self.initial_altitude + 0.2):
                rclpy.spin_once(self, timeout_sec=0.1)
                self.get_logger().info(f"Current altitude: {self.current_altitude}")
                self.publish_velocity(-0.2)
                self.publish_position(0.0)
                time.sleep(1)

            self.publish_velocity(0.0)
            self.get_logger().info("Mission complete!")
            self.disarm()
        return self.drone_ns

def main(args=None):
    rclpy.init(args=args)

    # Instantiate two drone controllers with different namespaces.
    drone1 = DroneController("mavros/uas_1_1")
    drone2 = DroneController("mavros/uas_1_2")
    drones = [drone1, drone2]

    # Create a MultiThreadedExecutor for ROS callbacks.
    executor = rclpy.executors.MultiThreadedExecutor()
    for drone in drones:
        executor.add_node(drone)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Use a ThreadPool with tqdm to run missions concurrently.
    def run_mission(drone):
        return drone.execute_mission()

    with ThreadPool(len(drones)) as pool:
        for ns in tqdm(pool.imap_unordered(run_mission, drones), total=len(drones)):
            print(f"Mission for {ns} completed.")

    executor.shutdown()
    for drone in drones:
        drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
