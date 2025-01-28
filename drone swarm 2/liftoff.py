import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class DroneLiftNode(Node):
    def __init__(self):
        super().__init__('drone_lift_node')
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.current_state = None

        # Timer for publishing setpoints
        self.setpoint_timer = self.create_timer(0.2, self.publish_setpoint)

        # Target Pose
        self.target_pose = PoseStamped()
        self.target_pose.header = Header()
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 0.0  # Initial height

        self.height_increment = 0.1  # Step size for height increment
        self.current_direction = "up"  # Direction of movement: "up" or "down"
        self.max_height = 1.0  # Maximum height the drone should reach
        self.is_armed = False

    def state_callback(self, msg):
        self.current_state = msg
        self.is_armed = msg.armed

    def publish_setpoint(self):
        if self.current_state is None or not self.is_armed:
            self.get_logger().warn("Drone is not armed or state not received yet.")
            return

        # Update the target pose based on the current direction
        self.target_pose.header.stamp = self.get_clock().now().to_msg()

        if self.current_direction == "up":
            self.target_pose.pose.position.z += self.height_increment
            if self.target_pose.pose.position.z >= self.max_height:  # Check if max height is reached
                self.get_logger().info("Reached maximum height. Switching direction to down.")
                self.current_direction = "down"

        elif self.current_direction == "down":
            self.target_pose.pose.position.z -= self.height_increment
            if self.target_pose.pose.position.z <= 0.0:  # Check if ground is reached
                self.get_logger().info("Reached the ground. Stopping movement.")
                self.setpoint_timer.cancel()  # Stop the timer to stop publishing setpoints

        # Publish the updated target pose
        self.local_pos_pub.publish(self.target_pose)
        self.get_logger().info(f"Publishing target position: {self.target_pose.pose.position}")

def main(args=None):
    rclpy.init(args=args)
    drone_lift_node = DroneLiftNode()

    try:
        rclpy.spin(drone_lift_node)
    except KeyboardInterrupt:
        drone_lift_node.get_logger().info('Shutting down drone lift node...')
    finally:
        drone_lift_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
