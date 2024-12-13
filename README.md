# ArUco Marker Robot Control with ROS2

This project demonstrates a ROS2-based robot control system simulated in Gazebo. The robot's movement is controlled based on ArUco markers detected in the camera feed. It integrates a usb camera driver, nodes for marker detection, and logic for sending movement commands to the robot.

## Features
- **Camera Integration**: Uses the `usb_cam` package to capture the video feed from a laptop or external camera.
- **ArUco Marker Detection**: Detects markers in the video feed and extracts their IDs and positions.
- **Robot Control**: Moves the robot based on the detected ArUco marker's position.(Check camera_subscriber.py)
- **Gazebo Simulation**: Simulates the robot's environment for testing and visualization.

## Requirements
- ROS2 (Humble or later version recommended)
- Gazebo
- `usb_cam` ROS2 package
- OpenCV (for ArUco marker detection)

## Installation
1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/aruco_marker_robot_control.git
   cd aruco_marker_robot_control
   ```

2. Build the ROS2 workspace:
   ```bash
   colcon build
   source install/setup.bash
   ```

3. Install dependencies:
   ```bash
   sudo apt install ros-<ros2-distro>-usb-cam
   sudo apt install ros-<ros2-distro>-gazebo-ros
   sudo apt install python3-opencv
   ```

## Usage

### 1. Launch the Camera Driver
Start the camera driver to capture the video feed:
```bash
ros2 launch usb_cam usb_cam.launch.py
```

### 2. Run the Custom Node
Run the node to process the video feed and detect ArUco markers:
```bash
ros2 run aruco_marker_robot_control camera_subscriber
```

### 3. Launch the Robot in Gazebo
Start the Gazebo simulation environment:
```bash
ros2 launch aruco_marker_robot_control robot_sim.launch.py
```

