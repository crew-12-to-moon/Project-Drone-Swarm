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
        self.qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        
        # Increase queue depth to 100 for higher update throughput.
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        
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
        
        self.current_altitude = 0.0
        self.leader_altitude = None       # Leader's current altitude
        self.leader_initial_altitude = None  # Leader's initial altitude
        self.leader_initial_x = None  # Leader's initial x position
        self.armed = False
        self.offboard_mode = False

        # Follower's initial pose and altitude.
        self.initial_pose = None
        self.initial_altitude = None
        self.initial_x = None  # Follower's initial x position

        # Wait for services to be available.
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for takeoff service...')
        while not self.disarm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for disarm service...')
        
        self.leader_pose_subscriber = None  # Initialize subscriber as None

    def create_leader_subscription(self):
        # Create subscription to leader's position
        self.leader_pose_subscriber = self.create_subscription(
            PoseStamped,
            f'mavros/{self.leader_ns}/local_position/pose',
            self.leader_pose_callback,
            self.qos_profile
        )

    def destroy_leader_subscription(self):
        if self.leader_pose_subscriber:
            self.destroy_subscription(self.leader_pose_subscriber)
            self.leader_pose_subscriber = None

    def state_callback(self, msg):
        self.armed = msg.armed
        self.offboard_mode = (msg.mode == "OFFBOARD")

    def altitude_callback(self, msg):
        self.current_altitude = msg.pose.position.z
        #self.get_logger().info(f"Follower altitude callback: {self.current_altitude}")
        if self.initial_pose is None:
            self.initial_pose = msg.pose
            self.initial_altitude = msg.pose.position.z
            self.initial_x = msg.pose.position.x
            '''self.get_logger().info(
                f"Follower initial position captured: x={self.initial_pose.position.x}, "
                f"y={self.initial_pose.position.y}, z={self.initial_altitude}"
            )'''

    def leader_pose_callback(self, msg):
        # Save the leader's current altitude and x position.
        self.leader_altitude = msg.pose.position.z
        self.leader_x = msg.pose.position.x
        #self.get_logger().info(f"Received leader current altitude: {self.leader_altitude}, x: {self.leader_x}")
        # Capture leader's initial altitude and x position once.
        if self.leader_initial_altitude is None:
            self.leader_initial_altitude = msg.pose.position.z
            self.leader_initial_x = msg.pose.position.x
            #self.get_logger().info(f"Leader initial altitude captured: {self.leader_initial_altitude}, x: {self.leader_initial_x}")

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
        if self.initial_pose is None:
            self.get_logger().error("Follower initial pose not received. Exiting.")
            rclpy.shutdown()
            sys.exit(1)
        if self.leader_altitude is None or self.leader_initial_altitude is None:
            self.get_logger().error("Leader data not received yet. Waiting...")
            return
        
        computed_target_z = self.leader_altitude - self.ground_offset
        computed_target_x = self.leader_x - (self.leader_initial_x - self.initial_x)
        target_z = computed_target_z
        target_x = computed_target_x

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = target_x
        msg.pose.position.y = self.initial_pose.position.y
        msg.pose.position.z = target_z
        self.position_publisher.publish(msg)
        #self.get_logger().info(f"Publishing follower target position: x={msg.pose.position.x}, z={target_z}")

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
        # First get both initial poses
        self.create_leader_subscription()
        self.get_logger().info("Waiting for initial poses...")
        
        while self.initial_pose is None or self.leader_initial_altitude is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
            
        # Store initial ground offset
        self.ground_offset = self.leader_initial_altitude - self.initial_altitude
        self.x_offset = self.leader_initial_x - self.initial_x
        #self.get_logger().info(f"Initial altitude offset: {self.ground_offset}, Initial x offset: {self.x_offset}")
        
        # Unsubscribe from leader
        self.destroy_leader_subscription()
        self.leader_altitude = None  # Reset leader altitude
        self.leader_x = None  # Reset leader x position
        
        if self.takeoff(1.0):
            self.get_logger().info("Holding at 1m for stabilization...")
            time.sleep(10)  # Wait for 5 seconds after takeoff

            # Now resubscribe to leader's position for following
            self.create_leader_subscription()
            self.get_logger().info("Resubscribed to leader's position")

            self.get_logger().info("Starting to follow leader's position...")
            
            # Continuously follow the leader's position
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                #self.get_logger().info(f"Follower current altitude: {self.current_altitude}")
                
                # Ensure leader's altitude data is received
                if self.leader_altitude is None or self.leader_x is None:
                    self.get_logger().error("Leader data not received yet. Waiting...")
                    continue
                
                self.publish_position()
                time.sleep(0.1)  # Adjust the sleep time as needed

                # Check if the follower has landed
                if self.current_altitude < self.initial_altitude + 0.1:  # Adjust threshold as needed
                    print("altitude before disarming: ", self.current_altitude)
                    self.get_logger().info("Follower has landed. Disarming...")
                    break

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
