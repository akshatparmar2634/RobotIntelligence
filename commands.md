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

### Terminal 1: Launch Gazebo House World and RVIZ

Launches the TurtleBot3 in the house world environment in Gazebo Harmonic.

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py use_sim_true:=True
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map_house.yaml
```

### Terminal 2: Run Camera Subscriber Node

Subscribes to the TurtleBot3 camera feed and displays the images.

```bash
ros2 run ri_pkg camera_subscriber
```

### Terminal 3: Control Robot with Keyboard

Enables keyboard control for the TurtleBot3 using the teleop node.

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### Terminal 4: Run Enhanced YOLO + VLM Camera Node

Runs an enhanced YOLO + SmolVLM2 node that combines object detection with vision-language model descriptions and coordinate mapping:

- Detects objects using YOLO with detailed descriptions from SmolVLM2 vision-language model
- Calculates real-world coordinates using robot pose and LiDAR data
- Saves detected objects with VLM descriptions to JSON with duplicate prevention
- Displays object coordinates and total count on the image
- Runs at 2 FPS for optimal performance

Ensure the Python script is executable and the virtual environment is activated.

```bash
source ~/yolo_clip_env/bin/activate
chmod +x ~/turtlebot3_ws/src/ri_pkg/ri_pkg/yolo_improved_vlm_node.py
python3 ~/turtlebot3_ws/src/ri_pkg/ri_pkg/yolo_improved_vlm_node.py
```

**Alternative**: You can also run it as a ROS 2 node:

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source ~/yolo_clip_env/bin/activate
ros2 run ri_pkg yolo_improved_vlm_node
```

## Notes

- **Environment Setup**: Ensure the ROS 2 workspace (`~/turtlebot3_ws`) and the virtual environment (`~/yolo_clip_env`) are properly configured with all dependencies installed.
- **Enhanced VLM Node**: The `yolo_improved_vlm_node.py` script (primary recommendation) includes:
  - SmolVLM2 vision-language model for detailed object descriptions
  - Robot pose subscription (`/odom`) for position tracking
  - LiDAR data processing (`/scan`) for distance measurement
  - Real-world coordinate calculation using camera angles and LiDAR ranges
  - JSON object storage (`detected_objects_vlm.json`) with duplicate prevention (0.5m threshold)
  - Frame rate limiting (2 FPS) for optimal performance and stability
  - GPU memory management for stable operation
- **YOLO+CLIP Node**: The `yolo_clip_camera_node.py` script (alternative) includes:
  - CLIP-based object classification (faster but less detailed than VLM)
  - Same coordinate mapping and storage functionality
  - JSON object storage (`detected_objects.json`)
- **VLM Node Dependencies**: The enhanced VLM node requires a virtual environment with dependencies including `ultralytics`, `opencv-python`, `torch`, `transformers`, and `Pillow`. The SmolVLM2 model will be automatically downloaded on first run. Install these dependencies in `~/yolo_clip_env` before running the node.
- **Object Detection Output**: Detected objects are saved to JSON files in the `ri_pkg` package directory:
  - **Primary**: `detected_objects_vlm.json` (VLM node) - includes rich VLM descriptions
  - **Alternative**: `detected_objects.json` (YOLO+CLIP node) - includes CLIP classifications
  
  Both files contain the following information:
  - Object ID, label, confidence score
  - World coordinates (x, y) and distance from robot
  - Robot pose at time of detection
  - CLIP classification results (YOLO+CLIP node) or VLM descriptions (VLM node)
  - Detection timestamp
- **TurtleBot3 Model**: The `TURTLEBOT3_MODEL` environment variable is set to `waffle_pi`. If using a different model (e.g., `burger`), update the variable accordingly.
- **File Permissions**: The `chmod +x` command for the YOLO node script only needs to be run once to make it executable.
- **Troubleshooting**:
  - If Gazebo fails to launch, verify that Gazebo Harmonic is installed and compatible with ROS 2 Jazzy.
  - Ensure the `ri_pkg` package is built in your workspace (`colcon build` in `~/turtlebot3_ws`).
  - Check that the camera topic (`/camera/image_raw`), odometry (`/odom`), and LiDAR (`/scan`) topics are being published correctly.
  - If objects are not being saved, check that the robot pose and LiDAR data are being received properly.
  - For VLM node: If you encounter GPU memory issues, the node will automatically manage memory and skip problematic frames.
  - For VLM node: The SmolVLM2 model downloads automatically on first run (~500MB), ensure good internet connection.

---

## Additional Information

- **ROS 2 Documentation**: Refer to the [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/index.html) for details on ROS 2 setup.
- **TurtleBot3 Documentation**: See the [TurtleBot3 ROS 2 documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/) for additional configuration options.
- **Gazebo Harmonic**: Ensure Gazebo is properly configured as per the [Gazebo documentation](https://gazebosim.org/docs/harmonic).
