# soybean_mold_detector


An autonomous robotic system for identifying and photographing plants using advanced robotics and computer vision. This project integrates a Unitree GO2 EDU robotic dog with a D1 Robotic Arm and a StereoLabs ZED AI Stereo Camera to create a platform capable of finding plants, navigating to them, and capturing detailed multi-angle photographs.

## System Overview

The plant photography robot operates through the following autonomous sequence:

1. **Plant Detection**: The robot scans the room using the ZED AI Stereo Camera, leveraging both RGB imagery and depth sensing to identify plants in the environment.

2. **Navigation**: Once a plant is detected, the robot calculates an optimal position and navigates to it, placing itself at an appropriate distance for photography.

3. **Multi-Angle Photography**: While maintaining a stationary base position, the D1 Robotic Arm moves the ZED camera around the plant to capture photographs from five distinct angles:
   - Front view (facing the plant directly)
   - Top view (looking down at the plant)
   - Right side view
   - Back view
   - Left side view

4. **Data Collection**: For each position, the system captures and saves multiple data types:
   - High-resolution RGB images
   - Depth maps showing the 3D structure of the plant
   - AI detection overlays highlighting plant features

The entire process runs autonomously after initialization, requiring no further human intervention.

## Hardware Components

### Unitree GO2 EDU Robotic Dog
- Quadruped robotic platform providing mobility and stability
- Onboard computing for system control and AI processing
- Power supply for all components

### D1 Robotic Arm
- 6-DOF robotic arm for precise camera positioning
- Mounted on the GO2's back platform
- Programmed with optimized movement paths for plant photography

### StereoLabs ZED AI Stereo Camera
- AI-powered stereo vision camera
- USB-C connection to camera, USB 3.1 connection to robot
- Features utilized:
  - RGB imaging (1280×720 resolution)
  - Depth mapping (up to 20m range)
  - AI-based object detection
  - 3D spatial awareness

## Software Architecture

The system is built on ROS2 (Robot Operating System 2) and organized into three separate packages to maintain proper separation of Python and C++ components:

### 1. plant_photography_py (Python Package)
- **main_control_node.py**: Central coordination system managing the overall process flow
- **plant_detection_node.py**: Computer vision for plant identification using ZED AI
- **navigation_node.py**: Controls GO2 robot movement and positioning
- **zed_camera_control_node.py**: Manages ZED camera operation and photo capture

### 2. d1_arm_control_cpp (C++ Package)
- **arm_control_node.cpp**: High-level arm movement control
- **arm_controller.cpp**: Low-level interface with D1 arm hardware
- Optimized C++ implementation for precise arm control

### 3. plant_photography_bringup (Launch Package)
- **system.launch.py**: Single launch file to start all system components
- Configurable parameters for hardware setup

## Installation and Setup

### Prerequisites
- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill
- ZED SDK 3.8 or later
- CUDA 11.7+ (for ZED AI functionality)
- C++14 compatible compiler
- Python 3.10+

### Installation Steps

1. **Set up ROS2 workspace:**
```bash
mkdir -p ~/plant_photography_ws/src
cd ~/plant_photography_ws/src
```

2. **Install ZED SDK:**
```bash
wget https://download.stereolabs.com/zedsdk/3.8/cu117/ubuntu22 -O zed_sdk_ubuntu22_cu117_v3.8.run
chmod +x zed_sdk_ubuntu22_cu117_v3.8.run
./zed_sdk_ubuntu22_cu117_v3.8.run
```

3. **Clone required repositories:**
```bash
# Clone project packages
git clone <URL-to-plant-photography-py>
git clone <URL-to-d1-arm-control-cpp>
git clone <URL-to-plant-photography-bringup>

# Clone dependencies
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
git clone <URL-to-unitree-go2-interface>
git clone <URL-to-d1-arm-interface>

# Install system dependencies
cd ~/plant_photography_ws
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
```

4. **Build the workspace:**
```bash
colcon build
source install/setup.bash
```

5. **Hardware connections:**
   - Mount D1 Arm securely on GO2 platform
   - Attach ZED camera to arm end effector
   - Connect USB-C to camera, USB 3.1 to GO2
   - Route cable along arm with sufficient slack for full movement

6. **Calibrate the system:**
   - Run the arm calibration procedure to set optimal photography positions
   - Test ZED camera connection and AI detection
   - Verify system communication

## Usage

### Running the System

Launch the complete system with a single command:

```bash
ros2 launch plant_photography_bringup system.launch.py
```

With custom parameters:
```bash
ros2 launch plant_photography_bringup system.launch.py device_path:=/dev/ttyUSB1 camera_model:=zed2
```

### Monitoring Operation

The system provides status updates through several ROS2 topics:

```bash
# View overall system logs
ros2 topic echo /rosout

# Monitor specific system components
ros2 topic echo /navigation_status  # Robot movement status
ros2 topic echo /arm_status         # Arm position status
ros2 topic echo /camera_status      # Photography status

# View camera feed in real-time
ros2 run image_view image_view --ros-args -r image:=/zed/zed_node/rgb/image_rect_color
```

### Output Data

The system saves all photography data to the `~/plant_photos` directory, with the following organization:

- **RGB Images**: `plant_<position>_rgb_<timestamp>.jpg`
- **Depth Data**: 
  - Raw depth: `plant_<position>_depth_raw_<timestamp>.png`
  - Colorized visualization: `plant_<position>_depth_color_<timestamp>.jpg`
- **AI Detections**: `plant_<position>_detections_<timestamp>.jpg`

Each photography session creates a complete set of images from all five perspectives, providing comprehensive plant documentation.

## Configuration

The system can be customized through configuration files:

### ZED Camera Settings
In `plant_photography_py/config/detection_config.yaml`:
- Resolution and frame rate
- AI detection sensitivity
- Depth sensing mode

### Arm Positions
In `d1_arm_control_cpp/config/arm_config.yaml`:
- Coordinates for each photography position
- Movement speed and acceleration
- Stabilization parameters

### Navigation Parameters
In `plant_photography_py/config/navigation_config.yaml`:
- Approach distance to plant
- Scanning speed
- Position tolerance

## Troubleshooting

### ZED Camera Issues
- **Camera not detected**: Ensure USB 3.0/3.1 port is used and cable is properly connected
- **Poor depth quality**: Adjust lighting conditions or modify depth_mode parameter
- **AI detection failures**: Check for CUDA compatibility and update ZED SDK

### Arm Movement Problems
- **Positioning errors**: Recalibrate arm preset positions for your specific setup
- **Movement failures**: Check serial connection and ensure proper power supply
- **Communication timeouts**: Verify baudrate settings match your hardware

### Navigation Challenges
- **Poor plant detection**: Adjust HSV thresholds or enable ZED AI detection
- **Positioning inaccuracy**: Modify approach_distance and position_tolerance parameters
- **Robot instability**: Ensure flat, stable surface for operation

## System Extensions

The modular architecture allows for several potential enhancements:

- **3D Plant Modeling**: Create complete 3D models by integrating point cloud data
- **Multiple Plant Photography**: Extend system to identify and photograph all plants in a room
- **Time-lapse Documentation**: Program scheduled photography for plant growth studies
- **Plant Species Classification**: Train custom AI models for plant identification

## License

This project is licensed under the Apache License 2.0.

## Acknowledgments

- Unitree Robotics for the GO2 EDU platform
- StereoLabs for the ZED camera and SDK
- Open-source ROS2 community for development tools and libraries
