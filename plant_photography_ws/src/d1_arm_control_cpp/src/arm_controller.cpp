#include "d1_arm_control_cpp/arm_controller.hpp"
#include <chrono>
#include <thread>
#include <iostream>

// Stub implementation for D1 arm SDK
// SDK calls
namespace d1_arm_sdk {
    class D1Arm {
    public:
        bool connect(const std::string& device, int baud_rate) {
            std::cout << "D1Arm: Connecting to " << device << " at " << baud_rate << " baud" << std::endl;
            return true;
        }
        
        void setMaxVelocity(double velocity) {
            std::cout << "D1Arm: Setting max velocity to " << velocity << std::endl;
        }
        
        bool moveToCartesianPose(double x, double y, double z, double roll, double pitch, double yaw) {
            std::cout << "D1Arm: Moving to position (" << x << ", " << y << ", " << z 
                      << ") with orientation (" << roll << ", " << pitch << ", " << yaw << ")" << std::endl;
            return true;
        }
        
        bool isMoving() {
            // Simulated implementation - would actually check arm status
            static int counter = 0;
            counter++;
            return counter < 10;  // Simulate movement for 10 calls
        }
    };
}

ArmController::ArmController(const std::string& device_path, int baud_rate)
    : device_path_(device_path), 
      baud_rate_(baud_rate),
      connected_(false),
      moving_(false) {
    
    // Create arm instance
    arm_ = std::make_unique<d1_arm_sdk::D1Arm>();
    
    // Initialize preset positions
    initPresetPositions();
}

ArmController::~ArmController() {
    // Cleanup resources if needed
}

bool ArmController::connect() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (arm_) {
        connected_ = arm_->connect(device_path_, baud_rate_);
        return connected_;
    }
    return false;
}

bool ArmController::isConnected() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return connected_;
}

bool ArmController::isMoving() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (arm_ && connected_) {
        return arm_->isMoving();
    }
    return false;
}

void ArmController::setMaxVelocity(double velocity) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (arm_ && connected_) {
        arm_->setMaxVelocity(velocity);
    }
}

bool ArmController::moveToPosition(const std::string& position_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!arm_ || !connected_) {
        return false;
    }
    
    // Find the preset position
    auto it = preset_positions_.find(position_name);
    if (it == preset_positions_.end()) {
        std::cerr << "Unknown position: " << position_name << std::endl;
        return false;
    }
    
    // Move to the position
    const Position& pos = it->second;
    moving_ = arm_->moveToCartesianPose(
        pos.x, pos.y, pos.z, 
        pos.roll, pos.pitch, pos.yaw);
    
    return moving_;
}

bool ArmController::moveToCartesianPose(double x, double y, double z, 
                                        double roll, double pitch, double yaw) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!arm_ || !connected_) {
        return false;
    }
    
    moving_ = arm_->moveToCartesianPose(x, y, z, roll, pitch, yaw);
    return moving_;
}

void ArmController::setPresetPosition(const std::string& name, 
                                     double x, double y, double z,
                                     double roll, double pitch, double yaw) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    Position pos = {x, y, z, roll, pitch, yaw};
    preset_positions_[name] = pos;
}

bool ArmController::waitForCompletion(unsigned int timeout_ms) {
    auto start_time = std::chrono::steady_clock::now();
    
    while (isMoving()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Check for timeout if specified
        if (timeout_ms > 0) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - start_time).count();
                
            if (elapsed >= timeout_ms) {
                return false;  // Timed out
            }
        }
    }
    
    return true;  // Movement completed
}

void ArmController::initPresetPositions() {
    // Initialize preset positions for photography
    // These positions need to be calibrated for the specific robot and arm setup
    
    // HOME position - arm tucked in safe position
    setPresetPosition("HOME", 0.0, 0.0, 0.3, 0.0, 0.0, 0.0);
    
    // FRONT position - camera directly in front of plant
    setPresetPosition("FRONT", 0.3, 0.0, 0.4, 0.0, 0.0, 0.0);
    
    // TOP position - camera looking down at plant from above
    setPresetPosition("TOP", 0.0, 0.0, 0.6, 0.0, 1.57, 0.0);
    
    // RIGHT_SIDE position - camera viewing right side of plant
    setPresetPosition("RIGHT_SIDE", 0.0, -0.3, 0.4, 0.0, 0.0, 1.57);
    
    // BACK position - camera behind plant
    setPresetPosition("BACK", -0.3, 0.0, 0.4, 0.0, 0.0, 3.14);
    
    // LEFT_SIDE position - camera viewing left side of plant
    setPresetPosition("LEFT_SIDE", 0.0, 0.3, 0.4, 0.0, 0.0, -1.57);
}
