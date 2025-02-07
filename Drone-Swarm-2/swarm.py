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

    def publish_position(self, target_altitude):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = target_altitude
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
        if self.takeoff(1.0):
            time.sleep(5)  # Wait for stabilization

            self.get_logger().info("Starting ascent to 2m...")
            while self.current_altitude < 2.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                self.get_logger().info(f"Current altitude: {self.current_altitude}")
                self.publish_velocity(0.2)
                self.publish_position(2.0)
                time.sleep(1)

            self.get_logger().info("Reached 2m, starting descent...")
            while self.current_altitude > 0.2:
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
    drone1 = DroneController("drone1")
    drone2 = DroneController("drone2")
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
