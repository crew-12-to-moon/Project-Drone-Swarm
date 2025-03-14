import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.srv import CommandTOL, CommandBool
from mavros_msgs.msg import State
import asyncio
from tqdm import tqdm
import sys
import threading

class DroneController(Node):
    def __init__(self, drone_ns):
        super().__init__(f'{drone_ns.replace("/", "_")}_controller')
        self.drone_ns = drone_ns
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.takeoff_client = self.create_client(CommandTOL, f'mavros/{drone_ns}/cmd/takeoff')
        self.disarm_client = self.create_client(CommandBool, f'mavros/{drone_ns}/cmd/arming')
        self.velocity_publisher = self.create_publisher(TwistStamped, f'mavros/{drone_ns}/setpoint_velocity/cmd_vel', qos_profile)
        self.position_publisher = self.create_publisher(PoseStamped, f'mavros/{drone_ns}/setpoint_position/local', qos_profile)
        self.state_subscriber = self.create_subscription(State, f'mavros/{drone_ns}/state', self.state_callback, qos_profile)
        self.altitude_subscriber = self.create_subscription(PoseStamped, f'mavros/{drone_ns}/local_position/pose', self.altitude_callback, qos_profile)

        self.current_altitude = 0.0
        self.armed = False
        self.offboard_mode = False

        self.initial_pose = None
        self.initial_altitude = None

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
        if self.initial_pose is None:
            self.initial_pose = msg.pose
            self.initial_altitude = msg.pose.position.z
            self.get_logger().info(
                f"Initial position captured: x={self.initial_pose.position.x}, "
                f"y={self.initial_pose.position.y}, z={self.initial_altitude}")

    async def takeoff(self, altitude=1.0):
        req = CommandTOL.Request()
        req.altitude = altitude
        future = self.takeoff_client.call_async(req)
        await future
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

    def publish_position(self, x_offset, y_offset, z_offset):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.initial_pose is None:
            self.get_logger().error("Initial pose not received. Exiting script.")
            rclpy.shutdown()
            sys.exit(1)
        else:
            msg.pose.position.x = self.initial_pose.position.x + x_offset
            msg.pose.position.y = self.initial_pose.position.y + y_offset
            msg.pose.position.z = self.initial_altitude + z_offset
        self.position_publisher.publish(msg)

    async def disarm(self):
        req = CommandBool.Request()
        req.value = False
        future = self.disarm_client.call_async(req)
        await future
        if future.result() is not None and future.result().success:
            self.get_logger().info("Drone disarmed successfully")
            return True
        else:
            self.get_logger().info("Drone disarm failed")
            return False

    async def execute_mission(self):
        while self.initial_pose is None:
            self.get_logger().info("Waiting for initial position data...")
            await asyncio.sleep(0.1)
            
        if await self.takeoff(1.0):
            await asyncio.sleep(5)

            ascent_offset = 2.0

            self.get_logger().info("Starting ascent to 2m...")
            while self.current_altitude < (self.initial_altitude + ascent_offset):
                rclpy.spin_once(self, timeout_sec=0.1)
                self.get_logger().info(f"Current altitude: {self.current_altitude}")
                self.publish_velocity(0.2)
                self.publish_position(0.0, 0.0, 2.0)
                await asyncio.sleep(1)

            self.get_logger().info("Reached 2m, starting descent...")
            while self.current_altitude > (self.initial_altitude + 0.2):
                rclpy.spin_once(self, timeout_sec=0.1)
                self.get_logger().info(f"Current altitude: {self.current_altitude}")
                self.publish_velocity(-0.2)
                self.publish_position(0.0, 0.0, 0.0)
                await asyncio.sleep(1)

            self.publish_velocity(0.0)
            self.get_logger().info("Mission complete!")
            await self.disarm()
        return self.drone_ns

async def main(args=None):
    rclpy.init(args=args)

    drone1 = DroneController("uas_1_1")
    drone2 = DroneController("uas_1_2")
    drones = [drone1, drone2]

    executor = rclpy.executors.MultiThreadedExecutor()
    for drone in drones:
        executor.add_node(drone)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    async def run_mission(drone):
        return await drone.execute_mission()

    results = await asyncio.gather(*[run_mission(drone) for drone in drones])
    for ns in tqdm(results, total=len(drones)):
        print(f"Mission for {ns} completed.")

    executor.shutdown()
    for drone in drones:
        drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
