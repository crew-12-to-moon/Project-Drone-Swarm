import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.srv import CommandTOL, CommandBool  # Added CommandBool import for disarming
from mavros_msgs.msg import State
import time


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.disarm_client = self.create_client(CommandBool, '/mavros/cmd/arming')  # Disarm client

        self.velocity_publisher = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', qos_profile)
        self.state_subscriber = self.create_subscription(State, '/mavros/state', self.state_callback, qos_profile)
        self.altitude_subscriber = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.altitude_callback, qos_profile)

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

    def disarm(self):
        req = CommandBool.Request()
        req.value = False  # Setting False to disarm
        future = self.disarm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
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
                time.sleep(1)

            self.get_logger().info("Reached 2m, starting descent...")

            while self.current_altitude > 0.2:
                rclpy.spin_once(self, timeout_sec=0.1)
                self.get_logger().info(f"Current altitude: {self.current_altitude}")
                self.publish_velocity(-0.2)
                time.sleep(1)

            self.publish_velocity(0.0)
            self.get_logger().info("Mission complete!")
            # After mission completion, disarm the drone
            self.disarm()


def main(args=None):
    rclpy.init(args=args)
    controller = DroneController()
    controller.execute_mission()
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
