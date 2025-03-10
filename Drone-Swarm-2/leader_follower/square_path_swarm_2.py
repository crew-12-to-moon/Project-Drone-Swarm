import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode
from mavros_msgs.msg import State
import time
import sys
import numpy as np
import threading


class LeaderDrone(Node):
    def __init__(self):
        super().__init__('leader_drone')

        self.declare_parameter('namespace', 'leader')
        namespace = self.get_parameter('namespace').value

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.takeoff_client = self.create_client(CommandTOL, f'/{namespace}/cmd/takeoff')
        self.disarm_client = self.create_client(CommandBool, f'/{namespace}/cmd/arming')
        self.mode_client = self.create_client(SetMode, f'/{namespace}/set_mode')

        while not self.takeoff_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for takeoff service...')
        while not self.disarm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for disarm service...')
        while not self.mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for set_mode service...')

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
        self.initial_pose = None
        self.initial_altitude = None

    def state_callback(self, msg):
        self.armed = msg.armed
        self.offboard_mode = (msg.mode == "OFFBOARD")

    def altitude_callback(self, msg):
        self.current_altitude = msg.pose.position.z
        if self.initial_pose is None:
            self.initial_pose = msg.pose
            self.initial_altitude = msg.pose.position.z

    def takeoff(self, altitude=2.0):
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

    def publish_position(self, x_offset, y_offset, altitude_offset):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.initial_pose is None:
            self.get_logger().error("Initial pose not received. Exiting script.")
            rclpy.shutdown()
            sys.exit(1)
        else:
            msg.pose.position.x = self.initial_pose.position.x + x_offset
            msg.pose.position.y = self.initial_pose.position.y + y_offset
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
        time.sleep(2)

    def disarm(self):
        req = CommandBool.Request()
        req.value = False
        future = self.disarm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info("Drone disarmed successfully")
            return True
        else:
            self.get_logger().info("Drone disarm failed")
            return False

    def execute_mission(self):
        while self.initial_pose is None:
            self.get_logger().info("Waiting for initial position data...")
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        if self.takeoff(2.0):
            time.sleep(5)

            square_path = [
                (0.5, 0.0),
                (0.5, 0.5),
                (0.0, 0.5),
                (0.0, 0.0)
            ]

            for x_offset, y_offset in square_path:
                self.get_logger().info(f"Moving to x={x_offset}, y={y_offset}")
                self.publish_position(x_offset, y_offset, 2.0)
                time.sleep(5)

            self.get_logger().info("Landing at the end position...")
            self.publish_position(0.0, 0.0, 0.0)
            time.sleep(5)

            self.switch_to_stabilize_mode()
            self.disarm()


class FollowerDrone(Node):
    def __init__(self):
        super().__init__('follower_drone')

        self.declare_parameter('namespace', 'follower')
        namespace = self.get_parameter('namespace').value

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.velocity_publisher = self.create_publisher(
            TwistStamped, f'/{namespace}/setpoint_velocity/cmd_vel', qos_profile)
        self.position_publisher = self.create_publisher(
            PoseStamped, f'/{namespace}/setpoint_position/local', qos_profile)
        self.state_subscriber = self.create_subscription(
            State, f'/{namespace}/state', self.state_callback, qos_profile)
        self.leader_pose_subscriber = self.create_subscription(
            PoseStamped, '/leader/local_position/pose', self.leader_pose_callback, qos_profile)

        self.current_altitude = 0.0
        self.armed = False
        self.offboard_mode = False
        self.leader_pose = None
        self.offset = 2.0

    def state_callback(self, msg):
        self.armed = msg.armed
        self.offboard_mode = (msg.mode == "OFFBOARD")

    def leader_pose_callback(self, msg):
        self.leader_pose = msg.pose
        self.follow_leader()

    def follow_leader(self):
        if self.leader_pose is None:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.leader_pose.position.x + self.offset
        msg.pose.position.y = self.leader_pose.position.y + self.offset
        msg.pose.position.z = self.leader_pose.position.z
        self.position_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    leader = LeaderDrone()
    follower = FollowerDrone()

    leader_thread = threading.Thread(target=rclpy.spin, args=(leader,))
    follower_thread = threading.Thread(target=rclpy.spin, args=(follower,))

    leader_thread.start()
    follower_thread.start()

    leader.execute_mission()

    leader.destroy_node()
    follower.destroy_node()
    rclpy.shutdown()

    leader_thread.join()
    follower_thread.join()


if __name__ == '__main__':
    main()
