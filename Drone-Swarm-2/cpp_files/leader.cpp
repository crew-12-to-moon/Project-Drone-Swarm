#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/state.hpp"

using namespace std::chrono_literals;

std::mutex spin_mutex; // Global lock

class DroneController : public rclcpp::Node
{
public:
  DroneController(const std::string & drone_ns)
  : Node(drone_ns + "_controller"), drone_ns_(drone_ns),
    current_altitude_(0.0), armed_(false), offboard_mode_(false),
    initial_altitude_(0.0), initial_pose_received_(false)
  {
    // Set up a QoS profile with BEST_EFFORT reliability and depth of 10
    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    qos_profile.best_effort();

    std::string base_ns = "mavros/" + drone_ns_;
    // Create service clients
    takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>(base_ns + "/cmd/takeoff");
    disarm_client_  = this->create_client<mavros_msgs::srv::CommandBool>(base_ns + "/cmd/arming");
    mode_client_    = this->create_client<mavros_msgs::srv::SetMode>(base_ns + "/set_mode");

    while (!takeoff_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for takeoff service...");
    }
    while (!disarm_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for disarm service...");
    }
    while (!mode_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
    }

    // Create publishers
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(base_ns + "/setpoint_velocity/cmd_vel", qos_profile);
    position_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(base_ns + "/setpoint_position/local", qos_profile);

    // Create subscriptions
    state_subscriber_ = this->create_subscription<mavros_msgs::msg::State>(
      base_ns + "/state", qos_profile,
      std::bind(&DroneController::state_callback, this, std::placeholders::_1));
    altitude_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      base_ns + "/local_position/pose", qos_profile,
      std::bind(&DroneController::altitude_callback, this, std::placeholders::_1));
  }

  bool takeoff(double altitude = 1.0)
  {
    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    req->altitude = altitude;
    auto future = takeoff_client_->async_send_request(req);
    {
      std::lock_guard<std::mutex> lock(spin_mutex);
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    }
    auto result = future.get();
    if (result != nullptr && result->success) {
      RCLCPP_INFO(this->get_logger(), "Takeoff successful to %.2f meters", altitude);
      return true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Takeoff failed");
      return false;
    }
  }

  void publish_velocity(double z_velocity)
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.twist.linear.z = z_velocity;
    velocity_publisher_->publish(msg);
  }

  void publish_position(double altitude_offset)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->get_clock()->now();
    if (!initial_pose_received_) {
      RCLCPP_ERROR(this->get_logger(), "Initial pose not received. Exiting script.");
      rclcpp::shutdown();
      std::exit(1);
    } else {
      msg.pose.position.x = initial_pose_.position.x;
      msg.pose.position.y = initial_pose_.position.y;
      msg.pose.position.z = initial_altitude_ + altitude_offset;
    }
    position_publisher_->publish(msg);
  }

  void switch_to_stabilize_mode()
  {
    RCLCPP_INFO(this->get_logger(), "Switching to STABILIZE mode for disarm...");
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = "STABILIZE";
    auto future = mode_client_->async_send_request(req);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    auto result = future.get();
    if (result != nullptr && result->mode_sent) {
      RCLCPP_INFO(this->get_logger(), "STABILIZE mode set successfully.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set STABILIZE mode.");
    }
    std::this_thread::sleep_for(2s);
  }

  bool disarm()
  {
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = false;  // False to disarm
    auto future = disarm_client_->async_send_request(req);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    auto result = future.get();
    if (result != nullptr && result->success) {
      RCLCPP_INFO(this->get_logger(), "Drone disarmed successfully");
      return true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Drone disarm failed");
      return false;
    }
  }

  std::string execute_mission()
  {
    // Wait until initial pose is received
    while (!initial_pose_received_) {
      rclcpp::spin_some(this->get_node_base_interface());
      RCLCPP_INFO(this->get_logger(), "Waiting for initial position data...");
      std::this_thread::sleep_for(20ms);
    }
    if (takeoff(1.0)) {
      // Hold position at 1 meter for 10 seconds
      RCLCPP_INFO(this->get_logger(), "Holding at 1m for 10 seconds...");
      std::this_thread::sleep_for(10s);

      double ascent_offset = 2.0;
      RCLCPP_INFO(this->get_logger(), "Starting ascent to 2m above initial altitude...");
      while (current_altitude_ < (initial_altitude_ + ascent_offset)) {
        rclcpp::spin_some(this->get_node_base_interface());
        RCLCPP_INFO(this->get_logger(), "Current altitude: %.2f", current_altitude_);
        publish_velocity(0.2);
        publish_position(ascent_offset);
        std::this_thread::sleep_for(1s);
      }

      // Hold position at 2 meters for 10 seconds
      RCLCPP_INFO(this->get_logger(), "Holding at 2m for 10 seconds...");
      std::this_thread::sleep_for(10s);

      RCLCPP_INFO(this->get_logger(), "Reached target altitude, starting descent...");
      while (current_altitude_ > (initial_altitude_ + 0.2)) {
        rclcpp::spin_some(this->get_node_base_interface());
        RCLCPP_INFO(this->get_logger(), "Current altitude: %.2f", current_altitude_);
        publish_velocity(-0.2);
        publish_position(0.0);
        std::this_thread::sleep_for(1s);
      }
      publish_velocity(0.0);
      RCLCPP_INFO(this->get_logger(), "Mission complete!");
      switch_to_stabilize_mode();
      disarm();
    }
    return drone_ns_;
  }

private:
  void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    armed_ = msg->armed;
    offboard_mode_ = (msg->mode == "OFFBOARD");
  }

  void altitude_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_altitude_ = msg->pose.position.z;
    RCLCPP_INFO(this->get_logger(), "Altitude callback: %.2f", current_altitude_);
    if (!initial_pose_received_) {
      initial_pose_ = msg->pose;
      initial_altitude_ = msg->pose.position.z;
      initial_pose_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Initial position captured: x=%.2f, y=%.2f, z=%.2f",
                  initial_pose_.position.x, initial_pose_.position.y, initial_altitude_);
    }
  }

  // Members
  std::string drone_ns_;
  double current_altitude_;
  bool armed_;
  bool offboard_mode_;
  geometry_msgs::msg::Pose initial_pose_;
  bool initial_pose_received_;
  double initial_altitude_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_publisher_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr altitude_subscriber_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr disarm_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneController>("uas_1_1");
  node->execute_mission();
  rclcpp::shutdown();
  return 0;
}
