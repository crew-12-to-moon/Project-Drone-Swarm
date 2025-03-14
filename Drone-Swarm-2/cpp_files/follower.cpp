#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <cstdlib>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/state.hpp"

using namespace std::chrono_literals;

std::mutex spin_mutex;

class FollowerController : public rclcpp::Node
{
public:
  FollowerController(const std::string & drone_ns, const std::string & leader_ns)
  : Node(drone_ns + "_controller"), drone_ns_(drone_ns), leader_ns_(leader_ns),
    current_altitude_(0.0), armed_(false), offboard_mode_(false),
    initial_altitude_(0.0), initial_pose_received_(false),
    leader_altitude_(0.0), leader_initial_altitude_(0.0),
    leader_initial_received_(false), leader_data_received_(false)
  {
    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    qos_profile.best_effort();

    std::string base_ns = "mavros/" + drone_ns_;
    std::string leader_base = "mavros/" + leader_ns_;

    // Create follower service clients
    takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>(base_ns + "/cmd/takeoff");
    disarm_client_  = this->create_client<mavros_msgs::srv::CommandBool>(base_ns + "/cmd/arming");
    mode_client_    = this->create_client<mavros_msgs::srv::SetMode>(base_ns + "/set_mode");
    while (!mode_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
    }

    // Create publishers
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(base_ns + "/setpoint_velocity/cmd_vel", qos_profile);
    position_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(base_ns + "/setpoint_position/local", qos_profile);

    // Create subscriptions for follower data
    state_subscriber_ = this->create_subscription<mavros_msgs::msg::State>(
      base_ns + "/state", qos_profile,
      std::bind(&FollowerController::state_callback, this, std::placeholders::_1));
    altitude_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      base_ns + "/local_position/pose", qos_profile,
      std::bind(&FollowerController::altitude_callback, this, std::placeholders::_1));

    // Subscribe to leader's pose
    leader_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      leader_base + "/local_position/pose", qos_profile,
      std::bind(&FollowerController::leader_pose_callback, this, std::placeholders::_1));

    while (!takeoff_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for takeoff service...");
    }
    while (!disarm_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for disarm service...");
    }
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
      RCLCPP_INFO(this->get_logger(), "Follower takeoff successful to %.2f meters", altitude);
      return true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Follower takeoff failed");
      return false;
    }
  }

  void publish_velocity(double z_velocity)
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.twist.linear.z = z_velocity;
    velocity_publisher_->publish(msg);
  }

  void publish_position()
  {
    if (!initial_pose_received_) {
      RCLCPP_ERROR(this->get_logger(), "Follower initial pose not received. Exiting.");
      rclcpp::shutdown();
      std::exit(1);
    }
    if (!leader_initial_received_) {
      RCLCPP_ERROR(this->get_logger(), "Leader data not received yet. Waiting...");
      return;
    }
    double altitude_offset = leader_initial_altitude_ - initial_altitude_;
    double target_z = leader_altitude_ - altitude_offset;
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.pose.position.x = initial_pose_.position.x;
    msg.pose.position.y = initial_pose_.position.y;
    msg.pose.position.z = target_z;
    position_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing follower target altitude: %.2f", target_z);
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
    req->value = false;
    auto future = disarm_client_->async_send_request(req);
    {
      std::lock_guard<std::mutex> lock(spin_mutex);
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    }
    auto result = future.get();
    if (result != nullptr && result->success) {
      RCLCPP_INFO(this->get_logger(), "Follower drone disarmed successfully");
      return true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Follower drone disarm failed");
      return false;
    }
  }

  std::string execute_mission()
  {
    while (!initial_pose_received_ || !leader_initial_received_) {
      rclcpp::spin_some(this->get_node_base_interface());
      RCLCPP_INFO(this->get_logger(), "Waiting for follower initial pose and leader initial altitude...");
      std::this_thread::sleep_for(20ms);
    }
    if (takeoff(1.0)) {
      std::this_thread::sleep_for(5s);
      double ascent_offset = 2.0;
      RCLCPP_INFO(this->get_logger(), "Follower starting mission based on leader altitude...");
      while (current_altitude_ < (initial_altitude_ + ascent_offset)) {
        rclcpp::spin_some(this->get_node_base_interface());
        RCLCPP_INFO(this->get_logger(), "Follower current altitude: %.2f", current_altitude_);
        publish_velocity(0.2);
        publish_position();
        std::this_thread::sleep_for(50ms);
      }
      RCLCPP_INFO(this->get_logger(), "Follower reached target altitude, starting descent...");
      while (current_altitude_ > (initial_altitude_ + 0.2)) {
        rclcpp::spin_some(this->get_node_base_interface());
        RCLCPP_INFO(this->get_logger(), "Follower current altitude: %.2f", current_altitude_);
        publish_velocity(-0.2);
        publish_position();
        std::this_thread::sleep_for(20ms);
      }
      publish_velocity(0.0);
      RCLCPP_INFO(this->get_logger(), "Follower mission complete!");
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
    double raw_altitude = msg->pose.position.z;
    altitude_history_.push_back(raw_altitude);
    if (altitude_history_.size() > filter_size_) {
      altitude_history_.pop_front();
    }

    double smoothed_altitude = std::accumulate(altitude_history_.begin(), altitude_history_.end(), 0.0) / altitude_history_.size();

    if (std::abs(smoothed_altitude - current_altitude_) > deadband_) {
      current_altitude_ = smoothed_altitude;
      RCLCPP_INFO(this->get_logger(), "Follower altitude callback: %.2f", current_altitude_);
    }

    if (!initial_pose_received_) {
      initial_pose_ = msg->pose;
      initial_altitude_ = msg->pose.position.z;
      initial_pose_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Follower initial position captured: x=%.2f, y=%.2f, z=%.2f",
                  initial_pose_.position.x, initial_pose_.position.y, initial_altitude_);
    }
  }

  void leader_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    leader_altitude_ = msg->pose.position.z;
    RCLCPP_INFO(this->get_logger(), "Received leader current altitude: %.2f", leader_altitude_);
    if (!leader_initial_received_) {
      leader_initial_altitude_ = msg->pose.position.z;
      leader_initial_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Leader initial altitude captured: %.2f", leader_initial_altitude_);
    }
    leader_data_received_ = true;
  }

  // Members
  std::string drone_ns_;
  std::string leader_ns_;
  double current_altitude_;
  bool armed_;
  bool offboard_mode_;
  geometry_msgs::msg::Pose initial_pose_;
  bool initial_pose_received_;
  double initial_altitude_;

  double leader_altitude_;
  double leader_initial_altitude_;
  bool leader_initial_received_;
  bool leader_data_received_;

  std::deque<double> altitude_history_;
  const size_t filter_size_ = 10;
  const double deadband_ = 0.05;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_publisher_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr altitude_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr leader_pose_subscriber_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr disarm_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowerController>("uas_1_2", "uas_1_1");
  node->execute_mission();
  rclcpp::shutdown();
  return 0;
}
