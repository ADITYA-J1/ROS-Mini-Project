# Lidar Camera SLAM

This repository contains a ROS2-based SLAM (Simultaneous Localization and Mapping) system that uses both LiDAR and camera data for mapping and navigation.

## Prerequisites

- ROS2 Humble
- Gazebo
- Required ROS2 packages:
  ```bash
  sudo apt-get install ros-humble-teleop-twist-keyboard
  ```

## Building the Package

1. Clone this repository into your ROS2 workspace:
   ```bash
   cd ~/Project
   ```

2. Build the package:
   ```bash
   colcon build
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Running the SLAM System

1. Launch the SLAM system:
   ```bash
   ros2 launch lidar_camera_slam slam_launch.py
   ```
   This will start:
   - Gazebo simulation with a simple world
   - SLAM node
   - Object detection node
   - RGB camera node
   - RViz2 for visualization

2. In a new terminal, start the teleop keyboard control:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

## Robot Control

Use the following keys to control the robot:
- `i`: Move forward
- `k`: Stop
- `j`: Rotate left
- `l`: Rotate right
- `,`: Move backward
- `u`, `o`: Diagonal forward movement
- `m`, `.`: Diagonal backward movement
- `q`: Increase speed
- `z`: Decrease speed

## Monitoring Sensor Data

You can monitor different sensor outputs using the following commands (run each in a separate terminal):

1. LiDAR scan data:
   ```bash
   ros2 topic echo /scan
   ```

2. Point cloud data:
   ```bash
   ros2 topic echo /point_cloud
   ```

3. RGB camera data:
   ```bash
   ros2 topic echo /camera/image_raw
   ```

4. Detected objects:
   ```bash
   ros2 topic echo /detected_objects
   ```

## Available Topics

Key topics published by the system:
- `/camera/image_raw`: RGB camera feed
- `/point_cloud`: LiDAR point cloud data
- `/scan`: LiDAR scan data
- `/detected_objects`: Object detection results
- `/map`: Generated map data
- `/tf`: Transform data
- `/colored_point_cloud`: Colored point cloud from sensor fusion

## Troubleshooting

If the robot or sensors are not responding:
1. Make sure all nodes are running:
   ```bash
   ros2 node list
   ```

2. Check if topics are being published:
   ```bash
   ros2 topic list
   ```

3. If topics are missing, try re-sourcing the workspace:
   ```bash
   source install/setup.bash
   ```

## System Architecture

The system consists of several nodes:
- SLAM Node: Handles mapping and localization
- Object Detection Node: Processes camera data for object detection
- RGB Camera Node: Manages camera input
- Sensor Fusion: Combines LiDAR and camera data

The simulation runs in a simple world environment with walls and various objects for testing SLAM and object detection capabilities. 