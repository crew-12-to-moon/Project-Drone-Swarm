import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
import time
import sys

class GpsDeniedDroneController(Node):
    def __init__(self):
        super().__init__('gps_denied_drone_controller')

        self.declare_parameter('namespace', 'drone1')
        namespace = self.get_parameter('namespace').value

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.arm_client = self.create_client(CommandBool, f'/{namespace}/cmd/arming')
        self.mode_client = self.create_client(SetMode, f'/{namespace}/set_mode')
        self.local_position_publisher = self.create_publisher(PoseStamped, f'/{namespace}/setpoint_position/local', qos_profile)

        while not self.arm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for arm service...')
        while not self.mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for set_mode service...')

        self.armed = False
        self.offboard_mode = False
        self.current_pose = None

        self.timer = self.create_timer(0.1, self.publish_setpoint)

    def state_callback(self, msg):
        self.armed = msg.armed
        self.offboard_mode = (msg.mode == "OFFBOARD")

    def position_callback(self, msg):
        self.current_pose = msg

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().mode_sent

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def publish_setpoint(self):
        if self.current_pose is None:
            return
        
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.current_pose.pose.position.x
        pose.pose.position.y = self.current_pose.pose.position.y
        pose.pose.position.z = 4.0  # Target altitude
        self.local_position_publisher.publish(pose)

    def execute_mission(self):
        self.get_logger().info("Setting OFFBOARD mode...")
        for _ in range(10):  # Send setpoints before mode switch
            self.publish_setpoint()
            time.sleep(0.1)
        if self.set_mode("OFFBOARD"):
            self.get_logger().info("OFFBOARD mode set successfully")
        else:
            self.get_logger().info("Failed to set OFFBOARD mode")
            return
        
        self.get_logger().info("Arming drone...")
        if self.arm():
            self.get_logger().info("Drone armed successfully")
        else:
            self.get_logger().info("Failed to arm drone")
            return

        self.get_logger().info("Holding altitude for 30 seconds...")
        time.sleep(30)
        
        self.get_logger().info("Landing...")
        if self.set_mode("LAND"):
            self.get_logger().info("Landing initiated")
        else:
            self.get_logger().info("Landing command failed")


def main(args=None):
    rclpy.init(args=args)
    controller = GpsDeniedDroneController()
    controller.execute_mission()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
