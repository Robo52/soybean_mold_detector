#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "d1_arm_control_cpp/arm_controller.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief Arm Control Node for D1 Robotic Arm
 * 
 * This node controls the D1 Robotic Arm, positioning it for
 * different photo angles around the plant.
 */
class ArmControlNode : public rclcpp::Node
{
public:
  ArmControlNode()
  : Node("arm_control_node")
  {
    // Declare parameters
    declare_parameter("device_path", "/dev/ttyUSB0");
    declare_parameter("baud_rate", 115200);
    declare_parameter("velocity", 0.5);
    
    // Get parameters
    auto device_path = get_parameter("device_path").as_string();
    auto baud_rate = get_parameter("baud_rate").as_int();
    auto velocity = get_parameter("velocity").as_double();
    
    // Initialize arm controller
    arm_controller_ = std::make_unique<ArmController>(device_path, baud_rate);
    
    // Connect to arm
    if (arm_controller_->connect()) {
      RCLCPP_INFO(get_logger(), "Successfully connected to D1 arm");
      arm_controller_->setMaxVelocity(velocity);
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to connect to D1 arm");
    }
    
    // Publisher for arm status
    arm_status_publisher_ = create_publisher<std_msgs::msg::String>("arm_status", 10);
    
    // Subscriber for arm commands
    arm_command_subscription_ = create_subscription<std_msgs::msg::String>(
      "arm_commands", 10, std::bind(&ArmControlNode::arm_command_callback, this, _1));
      
    RCLCPP_INFO(get_logger(), "Arm control node initialized");
  }

private:
  /**
   * @brief Process incoming arm commands
   * 
   * @param msg Command message
   */
  void arm_command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received arm command: %s", msg->data.c_str());
    
    std::string command = msg->data;
    
    if (!arm_controller_->isConnected()) {
      RCLCPP_ERROR(get_logger(), "Arm not connected, cannot process command");
      auto status_msg = std_msgs::msg::String();
      status_msg.data = "ARM_NOT_CONNECTED";
      arm_status_publisher_->publish(status_msg);
      return;
    }
    
    // Create a separate thread for arm movement
    // to avoid blocking the ROS 2 executor
    std::thread arm_thread([this, command]() {
      bool success = false;
      
      // Process different position commands
      if (command == "POSITION_HOME") {
        success = move_arm_to_position("HOME");
      } else if (command == "POSITION_FRONT") {
        success = move_arm_to_position("FRONT");
      } else if (command == "POSITION_TOP") {
        success = move_arm_to_position("TOP");
      } else if (command == "POSITION_RIGHT_SIDE") {
        success = move_arm_to_position("RIGHT_SIDE");
      } else if (command == "POSITION_BACK") {
        success = move_arm_to_position("BACK");
      } else if (command == "POSITION_LEFT_SIDE") {
        success = move_arm_to_position("LEFT_SIDE");
      } else {
        RCLCPP_WARN(get_logger(), "Unknown arm command: %s", command.c_str());
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "UNKNOWN_COMMAND";
        arm_status_publisher_->publish(status_msg);
        return;
      }
      
      // Publish status
      auto status_msg = std_msgs::msg::String();
      status_msg.data = success ? "POSITION_REACHED" : "POSITION_FAILED";
      arm_status_publisher_->publish(status_msg);
    });
    
    // Detach thread so it can run independently
    arm_thread.detach();
  }
  
  /**
   * @brief Move arm to specified position and wait for completion
   * 
   * @param position_name Name of the position
   * @return true if movement successful
   * @return false if movement failed
   */
  bool move_arm_to_position(const std::string& position_name)
  {
    RCLCPP_INFO(get_logger(), "Moving arm to position: %s", position_name.c_str());
    
    if (!arm_controller_->moveToPosition(position_name)) {
      RCLCPP_ERROR(get_logger(), "Failed to send position command");
      return false;
    }
    
    // Wait for arm to reach position
    RCLCPP_INFO(get_logger(), "Waiting for arm to reach position...");
    
    // Add a timeout (10 seconds)
    if (!arm_controller_->waitForCompletion(10000)) {
      RCLCPP_ERROR(get_logger(), "Arm movement timed out");
      return false;
    }
    
    RCLCPP_INFO(get_logger(), "Arm in position");
    
    // Add a short delay to stabilize before taking photo
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    return true;
  }

  // Arm controller
  std::unique_ptr<ArmController> arm_controller_;
  
  // Publishers and subscribers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_status_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_command_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmControlNode>());
  rclcpp::shutdown();
  return 0;
}
