# TurtleBot3 House World — ROS 2 Jazzy + Gazebo Harmonic

This README provides instructions to set up and run a TurtleBot3 simulation in the **house world** environment using **ROS 2 Jazzy** and **Gazebo Harmonic**. The setup includes launching the Gazebo world, displaying the TurtleBot3's camera feed, controlling the robot via keyboard, and running a YOLO-based camera node for object detection.

---

## Prerequisites

- **ROS 2 Jazzy Jalisco** installed (`/opt/ros/jazzy`).
- **TurtleBot3 packages** installed in a workspace (`~/turtlebot3_ws`).
- **Gazebo Harmonic** installed and configured.
- A Python virtual environment (`~/yolo_clip_env`) with required dependencies for the YOLO node.
- TurtleBot3 model set to `waffle_pi` (default for this setup).

Ensure the following environment variables and workspace are set up:

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
```

---

## Instructions

Run the following commands in separate terminal windows to set up the TurtleBot3 simulation.

### Terminal 1: Launch Gazebo House World

Launches the TurtleBot3 in the house world environment in Gazebo Harmonic.

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

### Terminal 2: Run Camera Subscriber Node

Subscribes to the TurtleBot3 camera feed and displays the images.

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run ri_pkg camera_subscriber
```

### Terminal 3: Control Robot with Keyboard

Enables keyboard control for the TurtleBot3 using the teleop node.

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run turtlebot3_teleop teleop_keyboard
```

### Terminal 4: Run Enhanced YOLO Camera Node

Runs an enhanced YOLO-based object detection node that:

- Detects objects in the camera feed with CLIP classification
- Calculates real-world coordinates using robot pose and LiDAR data
- Saves detected objects to JSON with duplicate prevention
- Displays object coordinates and total count on the image

Ensure the Python script is executable and the virtual environment is activated.

```bash
source ~/yolo_clip_env/bin/activate
chmod +x ~/turtlebot3_ws/src/ri_pkg/ri_pkg/yolo_clip_camera_node.py
python3 ~/turtlebot3_ws/src/ri_pkg/ri_pkg/yolo_clip_camera_node.py
```

**Alternative**: You can now also run it as a ROS 2 node:

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source ~/yolo_clip_env/bin/activate
ros2 run ri_pkg yolo_clip_camera_node
```

---

## Notes

- **Environment Setup**: Ensure the ROS 2 workspace (`~/turtlebot3_ws`) and the virtual environment (`~/yolo_clip_env`) are properly configured with all dependencies installed.
- **Enhanced YOLO Node**: The `yolo_clip_camera_node.py` script now includes:
  - Robot pose subscription (`/odom`) for position tracking
  - LiDAR data processing (`/scan`) for distance measurement
  - Real-world coordinate calculation using camera angles and LiDAR ranges
  - JSON object storage (`detected_objects.json`) with duplicate prevention (0.5m threshold)
  - Object coordinates display on the camera feed
- **YOLO Node Dependencies**: The YOLO node requires a virtual environment with YOLO dependencies (e.g., `ultralytics`, `opencv-python`, `torch`, `transformers`). Install these dependencies in `~/yolo_clip_env` before running the node.
- **Object Detection Output**: Detected objects are saved to `detected_objects.json` in the `ri_pkg` package directory (`~/turtlebot3_ws/src/ri_pkg/detected_objects.json`) with the following information:
  - Object ID, label, confidence score
  - World coordinates (x, y) and distance from robot
  - Robot pose at time of detection
  - CLIP classification results (if enabled)
  - Detection timestamp
- **TurtleBot3 Model**: The `TURTLEBOT3_MODEL` environment variable is set to `waffle_pi`. If using a different model (e.g., `burger`), update the variable accordingly.
- **File Permissions**: The `chmod +x` command for the YOLO node script only needs to be run once to make it executable.
- **Troubleshooting**:
  - If Gazebo fails to launch, verify that Gazebo Harmonic is installed and compatible with ROS 2 Jazzy.
  - Ensure the `ri_pkg` package is built in your workspace (`colcon build` in `~/turtlebot3_ws`).
  - Check that the camera topic (e.g., `/camera/image_raw`), odometry (`/odom`), and LiDAR (`/scan`) topics are being published correctly.
  - If objects are not being saved, check that the robot pose and LiDAR data are being received properly.

---

## Additional Information

- **ROS 2 Documentation**: Refer to the [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/index.html) for details on ROS 2 setup.
- **TurtleBot3 Documentation**: See the [TurtleBot3 ROS 2 documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/) for additional configuration options.
- **Gazebo Harmonic**: Ensure Gazebo is properly configured as per the [Gazebo documentation](https://gazebosim.org/docs/harmonic).
