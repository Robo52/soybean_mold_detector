#ifndef ARM_CONTROLLER_HPP
#define ARM_CONTROLLER_HPP

#include <string>
#include <vector>
#include <memory>
#include <mutex>

// Forward declarations for D1 arm SDK
// Include SDK headers
namespace d1_arm_sdk {
    class D1Arm;
}

/**
 * @brief Class for interfacing with the D1 Robotic Arm
 * 
 * This class provides a simplified interface to control the D1 Robotic Arm,
 * with functions specifically designed for the plant photography application.
 */
class ArmController {
public:
    /**
     * @brief Construct a new Arm Controller object
     * 
     * @param device_path Path to the serial device for the arm
     * @param baud_rate Baud rate for serial communication
     */
    ArmController(const std::string& device_path, int baud_rate);
    
    /**
     * @brief Destroy the Arm Controller object
     */
    ~ArmController();
    
    /**
     * @brief Connect to the arm
     * 
     * @return true if connection is successful
     * @return false if connection fails
     */
    bool connect();
    
    /**
     * @brief Check if the arm is connected
     * 
     * @return true if the arm is connected
     * @return false if the arm is not connected
     */
    bool isConnected() const;
    
    /**
     * @brief Check if the arm is currently moving
     * 
     * @return true if the arm is moving
     * @return false if the arm is stationary
     */
    bool isMoving() const;
    
    /**
     * @brief Set the maximum velocity of the arm
     * 
     * @param velocity Velocity as a fraction of maximum (0.0 to 1.0)
     */
    void setMaxVelocity(double velocity);
    
    /**
     * @brief Move the arm to a predefined position for photography
     * 
     * @param position_name Name of predefined position (FRONT, TOP, etc.)
     * @return true if the command was accepted
     * @return false if the command was rejected
     */
    bool moveToPosition(const std::string& position_name);
    
    /**
     * @brief Move the arm to a specific Cartesian position
     * 
     * @param x X coordinate in meters
     * @param y Y coordinate in meters
     * @param z Z coordinate in meters
     * @param roll Roll angle in radians
     * @param pitch Pitch angle in radians
     * @param yaw Yaw angle in radians
     * @return true if the command was accepted
     * @return false if the command was rejected
     */
    bool moveToCartesianPose(double x, double y, double z, 
                             double roll, double pitch, double yaw);
    
    /**
     * @brief Set a predefined position for photography
     * 
     * @param name Position name
     * @param x X coordinate
     * @param y Y coordinate
     * @param z Z coordinate
     * @param roll Roll angle
     * @param pitch Pitch angle
     * @param yaw Yaw angle
     */
    void setPresetPosition(const std::string& name, 
                          double x, double y, double z,
                          double roll, double pitch, double yaw);
    
    /**
     * @brief Wait for current movement to complete
     * 
     * @param timeout_ms Timeout in milliseconds (0 for no timeout)
     * @return true if movement completed successfully
     * @return false if movement failed or timed out
     */
    bool waitForCompletion(unsigned int timeout_ms = 0);

private:
    // D1 arm instance
    std::unique_ptr<d1_arm_sdk::D1Arm> arm_;
    
    // Device communication parameters
    std::string device_path_;
    int baud_rate_;
    
    // State variables
    bool connected_;
    bool moving_;
    
    // Thread safety
    mutable std::mutex mutex_;
    
    // Preset arm positions for different photos
    struct Position {
        double x, y, z;           // Cartesian coordinates
        double roll, pitch, yaw;  // Orientation
    };
    
    std::map<std::string, Position> preset_positions_;
    
    // Initialize preset positions
    void initPresetPositions();
};

#endif // ARM_CONTROLLER_HPP
