#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <future>
#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/state.hpp"

using namespace std::chrono_literals;

// Global mutex and condition variable for synchronization
std::mutex spin_mutex;
std::condition_variable cv;

class DroneController : public rclcpp::Node
{
public:
  DroneController(const std::string & drone_ns)
  : Node(drone_ns + "_controller"), drone_ns_(drone_ns),
    current_altitude_(0.0), armed_(false), offboard_mode_(false),
    initial_pose_received_(false)
  {
    // QoS profile: BEST_EFFORT with depth=10.
    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    qos_profile.best_effort();

    std::string base_ns = "mavros/" + drone_ns_;
    // Create service clients.
    takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>(base_ns + "/cmd/takeoff");
    disarm_client_  = this->create_client<mavros_msgs::srv::CommandBool>(base_ns + "/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(base_ns + "/set_mode");

    // Create publishers.
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      base_ns + "/setpoint_velocity/cmd_vel", qos_profile);
    position_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      base_ns + "/setpoint_position/local", qos_profile);

    // Create subscriptions.
    state_subscriber_ = this->create_subscription<mavros_msgs::msg::State>(
      base_ns + "/state", qos_profile,
      std::bind(&DroneController::state_callback, this, std::placeholders::_1));
    altitude_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      base_ns + "/local_position/pose", qos_profile,
      std::bind(&DroneController::altitude_callback, this, std::placeholders::_1));

    // Wait for service availability.
    while (!takeoff_client_->wait_for_service(2s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for takeoff service...");
    }
    while (!disarm_client_->wait_for_service(2s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for disarm service...");
    }
    while (!set_mode_client_->wait_for_service(2s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for set mode service...");
    }
  }

  void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    armed_ = msg->armed;
    offboard_mode_ = (msg->mode == "OFFBOARD");
  }

  void altitude_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(spin_mutex);
    current_altitude_ = msg->pose.position.z;
    RCLCPP_INFO(this->get_logger(), "Altitude callback: %.2f", current_altitude_);
    if (!initial_pose_received_) {
      initial_pose_ = msg->pose;
      initial_altitude_ = msg->pose.position.z;
      initial_pose_received_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Initial position captured: x=%.2f, y=%.2f, z=%.2f",
                  initial_pose_.position.x, initial_pose_.position.y, initial_altitude_);
    }
    cv.notify_all();
  }

  bool takeoff(double altitude = 1.0)
  {
    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    req->altitude = altitude;
    auto future = takeoff_client_->async_send_request(req);
    // Wait for the future to be ready
    future.wait();
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

  bool set_mode(const std::string & mode)
  {
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = mode;
    auto future = set_mode_client_->async_send_request(req);
    // Wait for the future to be ready
    future.wait();
    auto result = future.get();
    if (result != nullptr && result->mode_sent) {
      RCLCPP_INFO(this->get_logger(), "Mode set to %s successfully", mode.c_str());
      return true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Failed to set mode to %s", mode.c_str());
      return false;
    }
  }

  bool disarm()
  {
    if (!set_mode("STABILIZED")) {
      return false;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = false;  // False to disarm.
    auto future = disarm_client_->async_send_request(req);
    // Wait for the future to be ready
    future.wait();
    auto result = future.get();
    if (result != nullptr && result->success) {
      RCLCPP_INFO(this->get_logger(), "Drone disarmed successfully");
      return true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Drone disarm failed");
      return false;
    }
  }

  // The execute_mission method waits for the initial pose, performs a takeoff,
  // then ascends and descends while publishing velocity and position commands.
  std::string execute_mission()
  {
    {
      std::unique_lock<std::mutex> lock(spin_mutex);
      while (!initial_pose_received_) {
        RCLCPP_INFO(this->get_logger(), "Waiting for initial position data...");
        cv.wait(lock);
      }
    }
    if (takeoff(1.0)) {
      std::this_thread::sleep_for(5s); // Wait for stabilization.
      double ascent_offset = 2.0;
      RCLCPP_INFO(this->get_logger(), "Starting ascent to 2m...");
      {
        std::unique_lock<std::mutex> lock(spin_mutex);
        while (current_altitude_ < (initial_altitude_ + ascent_offset)) {
          RCLCPP_INFO(this->get_logger(), "Current altitude: %.2f", current_altitude_);
          publish_velocity(0.2);
          publish_position(2.0);
          cv.wait_for(lock, 1s);
        }
      }
      RCLCPP_INFO(this->get_logger(), "Reached 2m, starting descent...");
      {
        std::unique_lock<std::mutex> lock(spin_mutex);
        while (current_altitude_ > (initial_altitude_ + 0.2)) {
          RCLCPP_INFO(this->get_logger(), "Current altitude: %.2f", current_altitude_);
          publish_velocity(-0.2);
          publish_position(0.0);
          cv.wait_for(lock, 1s);
        }
      }
      publish_velocity(0.0);
      RCLCPP_INFO(this->get_logger(), "Mission complete!");
      disarm();
    }
    return drone_ns_;
  }

private:
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
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Instantiate two drone controllers with different namespaces.
  auto drone1 = std::make_shared<DroneController>("uas_1_1");
  auto drone2 = std::make_shared<DroneController>("uas_1_2");
  std::vector<std::shared_ptr<DroneController>> drones { drone1, drone2 };

  // Create a MultiThreadedExecutor and add both nodes.
  rclcpp::executors::MultiThreadedExecutor executor;
  for (auto & drone : drones) {
    executor.add_node(drone);
  }
  // Spin the executor in a separate thread.
  std::thread executor_thread([&executor]() {
    executor.spin();
  });

  // Launch the missions concurrently using std::async.
  std::vector<std::future<std::string>> futures;
  for (auto & drone : drones) {
    futures.push_back(std::async(std::launch::async, &DroneController::execute_mission, drone));
  }
  
  // Wait for all missions to complete and print a message.
  for (auto & fut : futures) {
    std::string ns = fut.get();
    std::cout << "Mission for " << ns << " completed." << std::endl;
  }

  // Shutdown the executor and join the thread.
  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  rclcpp::shutdown();
  return 0;
}
