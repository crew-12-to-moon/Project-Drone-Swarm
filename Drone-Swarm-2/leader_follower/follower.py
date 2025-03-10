import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode  # Added SetMode import
from mavros_msgs.msg import State
import time
import threading
import sys

# Global lock to protect spin_until_future_complete calls
spin_lock = threading.Lock()

class FollowerController(Node):
    def __init__(self, drone_ns, leader_ns):
        super().__init__(f'{drone_ns}_controller')
        self.drone_ns = drone_ns
        self.leader_ns = leader_ns
        
        # Increase queue depth to 100 for higher update throughput.
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=100)
        
        # Follower clients and publishers
        self.takeoff_client = self.create_client(CommandTOL, f'mavros/{drone_ns}/cmd/takeoff')
        self.disarm_client = self.create_client(CommandBool, f'mavros/{drone_ns}/cmd/arming')
        self.mode_client = self.create_client(SetMode, f'mavros/{drone_ns}/set_mode')  # Mode switching client
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')
        
        self.velocity_publisher = self.create_publisher(TwistStamped, f'mavros/{drone_ns}/setpoint_velocity/cmd_vel', qos_profile)
        self.position_publisher = self.create_publisher(PoseStamped, f'mavros/{drone_ns}/setpoint_position/local', qos_profile)
        self.state_subscriber = self.create_subscription(State, f'mavros/{drone_ns}/state', self.state_callback, qos_profile)
        self.altitude_subscriber = self.create_subscription(PoseStamped, f'mavros/{drone_ns}/local_position/pose', self.altitude_callback, qos_profile)
        
        # Subscribe to the leader's local position topic.
        self.leader_pose_subscriber = self.create_subscription(
            PoseStamped,
            f'mavros/{leader_ns}/local_position/pose',
            self.leader_pose_callback,
            qos_profile
        )

        self.current_altitude = 0.0
        self.leader_altitude = None       # Leader's current altitude
        self.leader_initial_altitude = None  # Leader's initial altitude
        self.armed = False
        self.offboard_mode = False

        # Follower's initial pose and altitude.
        self.initial_pose = None
        self.initial_altitude = None

        # Low-pass filter variables
        self.filtered_target_altitude = None
        self.alpha = 0.5  # Smoothing factor; adjust between 0 and 1

        # Wait for services to be available.
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for takeoff service...')
        while not self.disarm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for disarm service...')

    def state_callback(self, msg):
        self.armed = msg.armed
        self.offboard_mode = (msg.mode == "OFFBOARD")

    def altitude_callback(self, msg):
        self.current_altitude = msg.pose.position.z
        self.get_logger().info(f"Follower altitude callback: {self.current_altitude}")
        if self.initial_pose is None:
            self.initial_pose = msg.pose
            self.initial_altitude = msg.pose.position.z
            self.get_logger().info(
                f"Follower initial position captured: x={self.initial_pose.position.x}, "
                f"y={self.initial_pose.position.y}, z={self.initial_altitude}"
            )

    def leader_pose_callback(self, msg):
        # Save the leader's current altitude.
        self.leader_altitude = msg.pose.position.z
        self.get_logger().info(f"Received leader current altitude: {self.leader_altitude}")
        # Capture leader's initial altitude once.
        if self.leader_initial_altitude is None:
            self.leader_initial_altitude = msg.pose.position.z
            self.get_logger().info(f"Leader initial altitude captured: {self.leader_initial_altitude}")

    def takeoff(self, altitude=1.0):
        req = CommandTOL.Request()
        req.altitude = altitude
        future = self.takeoff_client.call_async(req)
        with spin_lock:
            rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f"Follower takeoff successful to {altitude} meters")
            return True
        else:
            self.get_logger().info("Follower takeoff failed")
            return False

    def publish_velocity(self, z_velocity):
        msg = TwistStamped()
        msg.twist.linear.z = z_velocity
        self.velocity_publisher.publish(msg)

    def publish_position(self):
        """
        Publish a position setpoint for the follower. This setpoint is computed using the leader's
        current altitude and an offset determined by the difference between the leader's and follower's
        initial altitudes.
            offset = leader_initial_altitude - follower_initial_altitude
        Then, the target altitude for the follower is:
            target_z = leader_current_altitude - offset
        A low-pass filter is applied to smooth the computed target altitude.
        """
        if self.initial_pose is None:
            self.get_logger().error("Follower initial pose not received. Exiting.")
            rclpy.shutdown()
            sys.exit(1)
        if self.leader_altitude is None or self.leader_initial_altitude is None:
            self.get_logger().error("Leader data not received yet. Waiting...")
            return

        altitude_offset = self.leader_initial_altitude - self.initial_altitude
        computed_target = self.leader_altitude - altitude_offset

        # Apply low-pass filter to smooth the target altitude.
        if self.filtered_target_altitude is None:
            self.filtered_target_altitude = computed_target
        else:
            self.filtered_target_altitude = self.alpha * computed_target + (1 - self.alpha) * self.filtered_target_altitude

        target_z = self.filtered_target_altitude

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.initial_pose.position.x
        msg.pose.position.y = self.initial_pose.position.y
        msg.pose.position.z = target_z
        self.position_publisher.publish(msg)
        self.get_logger().info(f"Publishing follower filtered target altitude: {target_z}")

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
        req.value = False
        future = self.disarm_client.call_async(req)
        with spin_lock:
            rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info("Follower drone disarmed successfully")
            return True
        else:
            self.get_logger().info("Follower drone disarm failed")
            return False

    def execute_mission(self):
        # Wait until both the follower's initial pose and leader's initial altitude are available.
        while self.initial_pose is None or self.leader_initial_altitude is None:
            rclpy.spin_once(self, timeout_sec=0.02)
            self.get_logger().info("Waiting for follower initial pose and leader initial altitude...")
            time.sleep(0.02)
        if self.takeoff(1.0):
            time.sleep(5)  # Wait for stabilization

            ascent_offset = 2.0

            self.get_logger().info("Follower starting mission based on leader altitude...")
            # Update loop now runs at approximately 20 Hz (sleeping 0.05 sec per iteration)
            while self.current_altitude < (self.initial_altitude + ascent_offset):
                rclpy.spin_once(self, timeout_sec=0.02)
                self.get_logger().info(f"Follower current altitude: {self.current_altitude}")
                self.publish_velocity(0.2)
                self.publish_position()
                time.sleep(0.05)

            self.get_logger().info("Follower reached target altitude, starting descent...")
            while self.current_altitude > (self.initial_altitude + 0.2):
                rclpy.spin_once(self, timeout_sec=0.02)
                self.get_logger().info(f"Follower current altitude: {self.current_altitude}")
                self.publish_velocity(-0.2)
                self.publish_position()
                time.sleep(0.02)

            self.publish_velocity(0.0)
            self.get_logger().info("Follower mission complete!")
            # Switch to a stabilizing flight mode before disarming.
            self.switch_to_stabilize_mode()
            self.disarm()
        return self.drone_ns

def main(args=None):
    rclpy.init(args=args)
    # For the follower, the drone namespace is "uas_1_2" and the leader's namespace is "uas_1_1"
    drone = FollowerController("uas_1_2", "uas_1_1")
    try:
        drone.execute_mission()
    except KeyboardInterrupt:
        pass
    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
