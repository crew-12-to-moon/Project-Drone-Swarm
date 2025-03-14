#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <chrono>
#include <thread>
#include <vector>
#include <future>

using namespace std::chrono_literals;

class DroneController : public rclcpp::Node {
public:
    DroneController(const std::string &drone_ns)
        : Node(drone_ns + "_controller"), drone_ns_(drone_ns) {
        qos_profile_ = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/" + drone_ns + "/cmd/takeoff");
        disarm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/" + drone_ns + "/cmd/arming");
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("mavros/" + drone_ns + "/setpoint_velocity/cmd_vel", qos_profile_);
        position_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/" + drone_ns + "/setpoint_position/local", qos_profile_);
        state_subscriber_ = this->create_subscription<mavros_msgs::msg::State>("mavros/" + drone_ns + "/state", qos_profile_, std::bind(&DroneController::state_callback, this, std::placeholders::_1));
        altitude_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("mavros/" + drone_ns + "/local_position/pose", qos_profile_, std::bind(&DroneController::altitude_callback, this, std::placeholders::_1));

        current_altitude_ = 0.0;
        armed_ = false;
        offboard_mode_ = false;
        initial_pose_ = nullptr;
        initial_altitude_ = 0.0;

        while (!takeoff_client_->wait_for_service(2s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for takeoff service...");
        }
        while (!disarm_client_->wait_for_service(2s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for disarm service...");
        }
    }

    void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
        armed_ = msg->armed;
        offboard_mode_ = (msg->mode == "OFFBOARD");
    }

    void altitude_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_altitude_ = msg->pose.position.z;
        RCLCPP_INFO(this->get_logger(), "Altitude callback: %f", current_altitude_);
        if (initial_pose_ == nullptr) {
            initial_pose_ = std::make_shared<geometry_msgs::msg::Pose>(msg->pose);
            initial_altitude_ = msg->pose.position.z;
            RCLCPP_INFO(this->get_logger(), "Initial position captured: x=%f, y=%f, z=%f", initial_pose_->position.x, initial_pose_->position.y, initial_altitude_);
        }
    }

    rclcpp::FutureReturnCode takeoff(float altitude) {
        auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        request->altitude = altitude;
        auto future = takeoff_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            if (future.get()->success) {
                RCLCPP_INFO(this->get_logger(), "Takeoff successful to %f meters", altitude);
                return rclcpp::FutureReturnCode::SUCCESS;
            } else {
                RCLCPP_INFO(this->get_logger(), "Takeoff failed");
                return rclcpp::FutureReturnCode::FAILURE;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Takeoff service call failed");
            return rclcpp::FutureReturnCode::FAILURE;
        }
    }

    void publish_velocity(float z_velocity) {
        auto msg = geometry_msgs::msg::TwistStamped();
        msg.twist.linear.z = z_velocity;
        velocity_publisher_->publish(msg);
    }

    void publish_position(float x_offset, float y_offset, float z_offset) {
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->now();
        if (initial_pose_ == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Initial pose not received. Exiting script.");
            rclcpp::shutdown();
            std::exit(1);
        } else {
            msg.pose.position.x = initial_pose_->position.x + x_offset;
            msg.pose.position.y = initial_pose_->position.y + y_offset;
            msg.pose.position.z = initial_altitude_ + z_offset;
        }
        position_publisher_->publish(msg);
    }

    rclcpp::FutureReturnCode disarm() {
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = false;
        auto future = disarm_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            if (future.get()->success) {
                RCLCPP_INFO(this->get_logger(), "Drone disarmed successfully");
                return rclcpp::FutureReturnCode::SUCCESS;
            } else {
                RCLCPP_INFO(this->get_logger(), "Drone disarm failed");
                return rclcpp::FutureReturnCode::FAILURE;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Disarm service call failed");
            return rclcpp::FutureReturnCode::FAILURE;
        }
    }

    rclcpp::FutureReturnCode execute_mission() {
        while (initial_pose_ == nullptr) {
            RCLCPP_INFO(this->get_logger(), "Waiting for initial position data...");
            std::this_thread::sleep_for(100ms);
        }

        if (takeoff(1.0) == rclcpp::FutureReturnCode::SUCCESS) {
            std::this_thread::sleep_for(5s);

            float ascent_offset = 2.0;

            RCLCPP_INFO(this->get_logger(), "Starting ascent to 2m...");
            while (current_altitude_ < (initial_altitude_ + ascent_offset)) {
                rclcpp::spin_some(this->shared_from_this());
                RCLCPP_INFO(this->get_logger(), "Current altitude: %f", current_altitude_);
                publish_velocity(0.2);
                publish_position(0.0, 0.0, 2.0);
                std::this_thread::sleep_for(1s);
            }

            RCLCPP_INFO(this->get_logger(), "Reached 2m, starting descent...");
            while (current_altitude_ > (initial_altitude_ + 0.2)) {
                rclcpp::spin_some(this->shared_from_this());
                RCLCPP_INFO(this->get_logger(), "Current altitude: %f", current_altitude_);
                publish_velocity(-0.2);
                publish_position(0.0, 0.0, 0.0);
                std::this_thread::sleep_for(1s);
            }

            publish_velocity(0.0);
            RCLCPP_INFO(this->get_logger(), "Mission complete!");
            disarm();
        }
        return rclcpp::FutureReturnCode::SUCCESS;
    }

private:
    std::string drone_ns_;
    rclcpp::QoS qos_profile_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr disarm_client_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_publisher_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr altitude_subscriber_;

    float current_altitude_;
    bool armed_;
    bool offboard_mode_;
    std::shared_ptr<geometry_msgs::msg::Pose> initial_pose_;
    float initial_altitude_;
};

void run_mission(std::shared_ptr<DroneController> drone) {
    drone->execute_mission();
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto drone1 = std::make_shared<DroneController>("uas_1_1");
    auto drone2 = std::make_shared<DroneController>("uas_1_2");
    std::vector<std::shared_ptr<DroneController>> drones = {drone1, drone2};

    rclcpp::executors::MultiThreadedExecutor executor;
    for (auto &drone : drones) {
        executor.add_node(drone);
    }
    std::thread executor_thread([&executor]() { executor.spin(); });

    std::vector<std::future<void>> futures;
    for (auto &drone : drones) {
        futures.push_back(std::async(std::launch::async, run_mission, drone));
    }

    for (auto &future : futures) {
        future.get();
    }

    executor.cancel();
    executor_thread.join();

    for (auto &drone : drones) {
        drone->destroy_node();
    }
    rclcpp::shutdown();

    return 0;
}
