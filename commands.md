# TurtleBot3 House World — ROS 2 Jazzy + Gazebo Harmonic

This README provides instructions to set up and run a TurtleBot3 simulation in the **house world** environment using **ROS 2 Jazzy** and **Gazebo Harmonic**. The setup includes launching the Gazebo world, displaying the TurtleBot3's camera feed, and running YOLO-based semantic mapping with both **manual teleop control** and **autonomous navigation** options.

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

## 🤖 Autonomous Navigation (Recommended)

For hands-free exploration and systematic semantic mapping, use the autonomous navigation system:

### Terminal 1: Launch Gazebo House World and Nav2

```bash
# Set environment
export TURTLEBOT3_MODEL=waffle_pi
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash

# Launch Gazebo house world
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py use_sim_time:=true

# In another terminal, launch Nav2 for navigation
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=$HOME/map_house.yaml
```

⚠️ **Important**: After launching Nav2, open RVIZ and set the **initial pose estimate** by clicking the "2D Pose Estimate" button and clicking/dragging on the map where the robot is located.

### Terminal 2: Check System Status (Recommended)

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash

# Run diagnostics to check if everything is ready
ros2 run ri_pkg navigation_diagnostics
```

### Terminal 3: Launch Autonomous Navigation System

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash

# Launch the navigation system (camera subscriber only, semantic mapping runs separately)
ros2 launch ri_pkg autonomous_navigation.launch.py
```

### Terminal 4: Launch Semantic Mapping (Required)

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source ~/yolo_clip_env/bin/activate

# Run semantic mapping node in virtual environment
ros2 run ri_pkg yolo_improved_vlm_node
```

The robot will automatically navigate through **5 custom waypoints** covering your specified goal points:

**Navigation Sequence:**
1. **Starting Position** (0, 0) - Robot initialization point
2. **Goal Point 1** (-7, 6.25) - First target location  
3. **Goal Point 2** (-4.5, 4) - Second target location
4. **Goal Point 3** (-2.5, 7) - Third target location
5. **Return to Start** (0, 0) - Complete the exploration cycle

The robot performs semantic mapping at each location with strategic observation pauses.

---

## 📱 Manual Control (Alternative)

For manual exploration with keyboard control:

### Terminal 1: Launch Gazebo House World and RVIZ

Launches the TurtleBot3 in the house world environment in Gazebo Harmonic.

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py use_sim_time:=True
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map_house.yaml
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

ros2 run turtlebot3_teleop teleop_keyboard
```

### Terminal 4: Run Enhanced YOLO + VLM Camera Node

Runs an enhanced YOLO + SmolVLM2 node for semantic mapping during manual exploration:

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source ~/yolo_clip_env/bin/activate

# Make executable (first time only)
chmod +x ~/turtlebot3_ws/src/ri_pkg/ri_pkg/yolo_improved_vlm_node.py

# Run as Python script
python3 ~/turtlebot3_ws/src/ri_pkg/ri_pkg/yolo_improved_vlm_node.py

# OR run as ROS 2 node
ros2 run ri_pkg yolo_improved_vlm_node
```

**Alternative - YOLO+CLIP Mode (faster, less detailed):**

```bash
source ~/yolo_clip_env/bin/activate
python3 ~/turtlebot3_ws/src/ri_pkg/ri_pkg/yolo_clip_camera_node.py
python3 ~/turtlebot3_ws/src/ri_pkg/ri_pkg/camera_subscriber.py

```

---

## 🆚 Comparison: Autonomous vs Manual

| Feature | **Autonomous Navigation** | **Manual Control** |
|---------|--------------------------|-------------------|
| **Control** | ✅ Fully automated | ❌ Requires constant input |
| **Coverage** | ✅ Systematic 5-point custom exploration | ❌ Depends on human navigation |
| **Consistency** | ✅ Reproducible paths | ❌ Variable exploration |
| **Efficiency** | ✅ Optimized waypoint sequence | ❌ Random/inefficient paths |
| **Hands-free** | ✅ No human intervention needed | ❌ Continuous keyboard control |
| **Semantic Quality** | ✅ Strategic observation poses | ❌ Variable observation quality |
| **Time Required** | ✅ ~10-15 minutes for full house | ❌ 30+ minutes, depends on operator |
| **Use Case** | 🎯 Production/research deployment | 🔧 Testing/debugging/custom exploration |

**Recommendation**: Use **Autonomous Navigation** for systematic semantic mapping. Use **Manual Control** only for testing, debugging, or custom exploration patterns.

---

## Generated Files

Both autonomous and manual modes generate the following output files in the `ri_pkg` package directory:

### Semantic Mapping Output:
- **`detected_objects_vlm.json`** - Objects with VLM descriptions and world coordinates
- **`detected_objects.json`** - Objects with CLIP classification (if using CLIP mode)

### Navigation Output (Autonomous mode only):
- **`navigation_waypoints.json`** - Navigation progress and visited waypoints  
- **`exploration_report.json`** - Complete exploration summary

---

## Notes

- **Environment Setup**: Ensure the ROS 2 workspace (`~/turtlebot3_ws`) and the virtual environment (`~/yolo_clip_env`) are properly configured with all dependencies installed.

### Autonomous Navigation Features:
- **Nav2 Integration**: Uses ROS 2 Nav2 stack for intelligent path planning and obstacle avoidance
- **5 Custom Waypoints**: Targeted exploration of user-specified goal points (-7,6.25), (-4.5,4), (-2.5,7)
- **Smart Recovery**: Handles navigation failures and continues exploration
- **Progress Tracking**: Saves progress and can resume interrupted explorations
- **Observation Pauses**: Strategic pauses at each waypoint for comprehensive semantic mapping
- **Coordinate Integration**: Seamlessly works with existing YOLO+VLM coordinate mapping

### Enhanced VLM Node Features:
- **SmolVLM2 Vision-Language Model**: Detailed object descriptions beyond simple classification
- **Real-world Coordinate Mapping**: Uses robot pose (`/odom`) and LiDAR data (`/scan`) 
- **Duplicate Prevention**: 0.5m threshold prevents duplicate object entries
- **JSON Storage**: Saves objects to `detected_objects_vlm.json` with rich metadata
- **Frame Rate Control**: 2 FPS for optimal performance and GPU memory management
- **Auto-download**: SmolVLM2 model downloads automatically on first run (~500MB)

### Dependencies:
- **Virtual Environment**: `~/yolo_clip_env` with `ultralytics`, `opencv-python`, `torch`, `transformers`, `Pillow`
- **ROS 2 Packages**: `nav2_msgs`, `turtlebot3_gazebo`, `turtlebot3_navigation2`
- **Hardware**: GPU recommended for VLM processing (CPU fallback available)
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
### Troubleshooting:

**🔧 Quick Diagnostics:**
```bash
# Run system diagnostics first
ros2 run ri_pkg navigation_diagnostics
```

**Navigation Issues:**
- **Nav2 not ready**: Ensure navigation2 launch is running and map is loaded
- **Goal rejected**: Set initial pose estimate in RVIZ using "2D Pose Estimate" tool
- **Robot not moving**: Check that cmd_vel topic is not blocked by other publishers  
- **Navigation failures**: Robot automatically skips failed waypoints and continues
- **Stuck in loops**: Stop and restart the navigation node

**VLM/Semantic Mapping Issues:**
- **Module not found (ultralytics)**: Run VLM node separately with virtual environment
- **GPU memory errors**: Node automatically manages memory and skips problematic frames
- **Model download**: SmolVLM2 model downloads automatically on first run (~500MB)
- **Slow performance**: Normal - VLM processing runs at 2 FPS for stability

**Setup Issues:**
- **Topics missing**: Ensure Gazebo and Nav2 are launched first
- **Robot localization**: Check `/odom` topic and robot pose in RVIZ
- **Map not loading**: Verify map file exists at `$HOME/map_house.yaml`

**General Tips:**
- Ensure the house map (`map_house.yaml`) is available and accurate
- Use `ros2 topic list` to verify all required topics are publishing
- Monitor robot pose and costmap for proper localization in RVIZ
- Check ROS 2 workspace is properly sourced in all terminals

---

## Quick Commands Summary

**🤖 Autonomous Navigation (Recommended):**
```bash
# Terminal 1: Launch Gazebo + Nav2
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py use_sim_time:=true
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=$HOME/map_house.yaml
# IMPORTANT: Set initial pose estimate in RVIZ!

# Terminal 2: Check system status
ros2 run ri_pkg navigation_diagnostics

# Terminal 3: Launch navigation system  
ros2 launch ri_pkg autonomous_navigation.launch.py

# Terminal 4: Launch semantic mapping (separate for virtual environment)
source ~/yolo_clip_env/bin/activate && ros2 run ri_pkg yolo_improved_vlm_node

# Terminal 5 (optional): Manual control
ros2 run ri_pkg navigation_controller
```

**📱 Manual Control (Testing/Debug):**
```bash
# Terminal 1-3: Same as above (Gazebo + Nav2)
# Terminal 4: Manual teleop
ros2 run turtlebot3_teleop teleop_keyboard
# Terminal 5: Manual VLM node
ros2 run ri_pkg yolo_improved_vlm_node
```

---

## Additional Information

- **ROS 2 Documentation**: [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/index.html)
- **TurtleBot3 Documentation**: [TurtleBot3 ROS 2 Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/)
- **Nav2 Documentation**: [Navigation2 Framework](https://navigation.ros.org/)
- **Gazebo Documentation**: [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)




## Camera Bringup

```bash
ros2 launch turtlebot3_bringup camera.launch.py   width:=640 height:=480 framerate:=30 pixel_format:=YUYV   image_transport:=theora   theora_quality:=15   theora_target_bitrate:=8000
```

## quick commands

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py use_sim_time:=True
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map_house.yaml

source ~/yolo_clip_env/bin/activate
python3 ~/turtlebot3_ws/src/ri_pkg/ri_pkg/coordinate_mapping.py

source ~/yolo_clip_env/bin/activate
python3 ~/turtlebot3_ws/src/ri_pkg/ri_pkg/autonomous_navigation_node.py
```
