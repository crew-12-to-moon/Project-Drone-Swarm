#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class OffboardControlNode(Node):
    def __init__(self):
        super().__init__('offb_node_py')

        # Initialize state
        self.current_state = State()

        # Subscribers
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10
        )

        # Publishers
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        # Service Clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Wait for services
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set mode service...')

        # Timer for setpoint publishing
        self.timer = self.create_timer(0.05, self.publish_setpoint)  # 20Hz

        # Pose target
        self.pose = PoseStamped()
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 2.0

        self.offb_set_mode = SetMode.Request()
        self.offb_set_mode.custom_mode = 'GUIDED'

        self.arm_cmd = CommandBool.Request()
        self.arm_cmd.value = True

        self.last_req = self.get_clock().now()

    def state_cb(self, msg):
        self.current_state = msg

    def publish_setpoint(self):
        # Wait for connection to FCU
        if not self.current_state.connected:
            self.get_logger().info('Waiting for FCU connection...')
            return

        # Set mode to GUIDED
        if self.current_state.mode != 'GUIDED' and (self.get_clock().now() - self.last_req).nanoseconds > 5e9:
            future = self.set_mode_client.call_async(self.offb_set_mode)
            rclpy.spin_until_future_complete(self, future)
            if future.result() and future.result().mode_sent:
                self.get_logger().info('GUIDED mode enabled')
            self.last_req = self.get_clock().now()

        # Arm the vehicle
        if not self.current_state.armed and (self.get_clock().now() - self.last_req).nanoseconds > 5e9:
            future = self.arming_client.call_async(self.arm_cmd)
            rclpy.spin_until_future_complete(self, future)
            if future.result() and future.result().success:
                self.get_logger().info('Vehicle armed')
            self.last_req = self.get_clock().now()

        # Publish setpoint
        self.local_pos_pub.publish(self.pose)


def main(args=None):
    rclpy.init(args=args)

    offboard_control_node = OffboardControlNode()

    try:
        rclpy.spin(offboard_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        offboard_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
