# Complete Setup Guide - Robot Intelligence Package (ri_pkg)

This guide provides step-by-step instructions for setting up the entire TurtleBot3 autonomous navigation and semantic mapping system from scratch.

## Table of Contents

1. [System Requirements](#system-requirements)
2. [Install ROS 2 Jazzy](#install-ros-2-jazzy)
3. [Setup TurtleBot3 Workspace](#setup-turtlebot3-workspace)
4. [Clone Repository](#clone-repository)
5. [Install Dependencies](#install-dependencies)
6. [Build Workspace](#build-workspace)
7. [Download Model Weights](#download-model-weights)
8. [Create Map (One-time Setup)](#create-map-one-time-setup)
9. [Configure Waypoints](#configure-waypoints)
10. [Verification](#verification)
11. [Next Steps](#next-steps)

---

## System Requirements

### Recommended Hardware

- **OS:** Ubuntu 24.04.3 LTS (Noble Numbat) - for ROS2 Jazzy Jalisco
- **Kernel:** Linux 6.14.0-32-generic or newer
- **Processor:** Intel Core i7-12700H or equivalent (12th Gen Intel® Core™ i7-12700H × 20)
- **Memory:** 16 GB RAM minimum
- **GPU:** NVIDIA GeForce RTX 3050 Ti or better with 4GB+ VRAM
  - GPU is **strongly recommended** for running Gazebo and VLM models
  - Install NVIDIA drivers for Linux
- **Storage:** 20 GB free disk space
- **Windowing System:** X11 (64-bit)

### Software Prerequisites

- Ubuntu 24.04.3 LTS (Noble Numbat)
- Python 3.10 or newer
- Git
- NVIDIA GPU drivers (if using GPU)

---

## Install ROS 2 Jazzy

### Step 1: Set up ROS 2 Repository

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup Sources
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 2: Install ROS 2 Jazzy Desktop

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install ros-jazzy-desktop -y
sudo apt install ros-dev-tools -y
```

### Step 3: Setup ROS 2 Environment

```bash
# Add to .bashrc for automatic sourcing
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
```

### Step 4: Test ROS 2 Installation

```bash
# Terminal 1
ros2 run demo_nodes_cpp talker

# Terminal 2 (new terminal)
ros2 run demo_nodes_py listener
```

If you see messages being sent and received, ROS 2 is properly installed!

**Important:** Follow the complete ROS 2 tutorials:

- [Installation — ROS 2 Documentation: Jazzy documentation](https://docs.ros.org/en/jazzy/Installation.html)
- [ROS 2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)
- [TurtleSim Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)

---

## Setup TurtleBot3 Workspace

**CRITICAL:** Follow these guides **IN ORDER** and **COMPLETE EVERY STEP**. Make sure to select "Jazzy" at the top of each page!

### Step 1: Quick Start Guide

Follow: [TurtleBot3 Quick Start](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

This guide covers:

- Installing TurtleBot3 packages
- Setting up PC environment
- Basic configuration

### Step 2: Simulation Setup

Follow: [TurtleBot3 Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

Install Gazebo and simulation packages:

```bash
# Install Gazebo
sudo apt install ros-jazzy-gazebo-ros-pkgs -y

# Create workspace
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src

# Clone TurtleBot3 repositories
git clone -b jazzy-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b jazzy-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# Build workspace
cd ~/turtlebot3_ws
colcon build --symlink-install

# Source workspace
source ~/turtlebot3_ws/install/setup.bash
echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc

# Set TurtleBot3 model
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Test Simulation

```bash
# Launch empty world
ros2 launch turtlebot3_gazebo empty_world.launch.py

# In a new terminal, test teleop
ros2 run turtlebot3_teleop teleop_keyboard
```

### Step 4: SLAM Setup

Follow: [TurtleBot3 SLAM Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/)

```bash
# Install SLAM packages
sudo apt install ros-jazzy-cartographer -y
sudo apt install ros-jazzy-cartographer-ros -y
sudo apt install ros-jazzy-navigation2 -y
sudo apt install ros-jazzy-nav2-bringup -y
```

### Step 5: Navigation Setup

Follow: [TurtleBot3 Navigation Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/)

After completing these guides, verify your workspace structure:

```bash
ls ~/turtlebot3_ws/src
```

Should show:

```text
DynamixelSDK  turtlebot3  turtlebot3_msgs  turtlebot3_simulations
```

---

## Clone Repository

Clone the ri_pkg repository into your workspace:

```bash
cd ~/turtlebot3_ws/src
git clone https://github.com/akshatparmar2634/RobotIntelligence.git ri_pkg
```

Verify the repository is cloned:

```bash
ls ~/turtlebot3_ws/src
```

Should now show:

```text
DynamixelSDK  ri_pkg  turtlebot3  turtlebot3_msgs  turtlebot3_simulations
```

---

## Install Dependencies

### Step 1: Install System Dependencies

```bash
# Update package lists
sudo apt update

# Install FFmpeg (required for audio processing)
sudo apt install ffmpeg -y

# Install Python development tools
sudo apt install python3-pip python3-venv -y

# Install ROS 2 package dependencies
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Step 2: Create Python Virtual Environment

```bash
# Create virtual environment
python3 -m venv ~/yolo_clip_env

# Activate environment
source ~/yolo_clip_env/bin/activate

# Upgrade pip
pip install --upgrade pip wheel setuptools
```

### Step 3: Install Python Dependencies

```bash
# Navigate to ri_pkg directory
cd ~/turtlebot3_ws/src/ri_pkg

# Install all requirements
pip install -r requirements.txt

# Install additional audio library
pip install pydub

# Verify installation
pip list | grep torch
pip list | grep ultralytics
pip list | grep transformers
```

**Note:** The `requirements.txt` includes:

- `ultralytics` - YOLOv8/v10 object detection
- `transformers` - InternVL 3.5 VLM model
- `torch`, `torchvision` - Deep learning framework
- `opencv-python` - Computer vision
- `sounddevice`, `pyttsx3` - Audio I/O for speech features
- And many more dependencies for the perception pipeline

### Step 4: Verify GPU Setup (if using NVIDIA GPU)

```bash
# Check NVIDIA driver
nvidia-smi

# Check PyTorch CUDA availability
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}'); print(f'CUDA version: {torch.version.cuda}')"
```

Expected output should show `CUDA available: True`

---

## Build Workspace

Build the entire workspace:

```bash
# Deactivate Python virtual environment first
deactivate

# Navigate to workspace root
cd ~/turtlebot3_ws

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build all packages
colcon build --symlink-install

# Source the workspace
source ~/turtlebot3_ws/install/setup.bash
```

Verify the build:

```bash
# Check if ri_pkg is built
ros2 pkg list | grep ri_pkg

# Check available executables
ros2 pkg executables ri_pkg
```

Expected output should include:

- autonomous_navigation_node
- camera_subscriber
- perception_waffle
- perception_internvl
- And other nodes listed in setup.py

---

## Download Model Weights

Download YOLO model weights to the workspace root:

```bash
cd ~/turtlebot3_ws

# Download YOLOv8s (recommended for balance of speed and accuracy)
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8s.pt

# Optional: Download other YOLO variants
# wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt  # Faster, less accurate
# wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8m.pt  # Balanced
# wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8l.pt  # Slower, more accurate

# Or for YOLOv10
wget https://github.com/jameslahm/yolov10/releases/download/v1.0/yolov10s.pt
```

Verify download:

```bash
ls ~/turtlebot3_ws/*.pt
```

---

## Create Map (One-time Setup)

Before running autonomous navigation, you need to create a map of your environment.

### Step 1: Launch Gazebo with House World

```bash
# Terminal 1
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

### Step 2: Run SLAM

```bash
# Terminal 2
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### Step 3: Drive Robot to Map Environment

```bash
# Terminal 3
source /opt/ros/jazzy/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Drive the robot around the entire house to create a complete map.

### Step 4: Save the Map

```bash
# Terminal 4 (once mapping is complete)
cd ~
ros2 run nav2_map_server map_saver_cli -f map_house
```

This creates two files:

- `~/map_house.yaml` - Map metadata
- `~/map_house.pgm` - Map image

---

## Configure Waypoints

The navigation system uses waypoints defined in code. To customize waypoints:

1. Open the navigation node:

```bash
nano ~/turtlebot3_ws/src/ri_pkg/ri_pkg/autonomous_navigation_node.py
```

2. Find the `initialize_house_waypoints` function (around line 167)

3. Modify waypoint coordinates as needed:

```python
house_waypoints = [
    {"x": 0.0, "y": 0.0, "theta": 0.0, "description": "Starting position"},
    {"x": -4.5, "y": 4.0, "theta": 1.57, "description": "Goal point 1"},
    {"x": -7.0, "y": 6.25, "theta": 0.0, "description": "Goal point 2"},
    {"x": -2.5, "y": 7.0, "theta": 3.14, "description": "Goal point 3"},
    {"x": 0.0, "y": 0.0, "theta": 0.0, "description": "Return to start"}
]
```

4. Save and rebuild:

```bash
cd ~/turtlebot3_ws
colcon build --packages-select ri_pkg
source install/setup.bash
```

---

## Verification

Run these checks to ensure everything is set up correctly:

### 1. Check ROS 2 Installation

```bash
ros2 --version
# Should show: ros2 jazzy
```

### 2. Check Workspace Build

```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 pkg list | grep -E "(turtlebot3|ri_pkg)"
```

### 3. Check Python Environment

```bash
source ~/yolo_clip_env/bin/activate
python3 -c "import torch, ultralytics, transformers, cv2; print('All imports successful!')"
deactivate
```

### 4. Check YOLO Weights

```bash
ls -lh ~/turtlebot3_ws/*.pt
```

### 5. Check Map Files

```bash
ls -lh ~/map_house.*
```

### 6. Test Gazebo Launch

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
# Let it launch, then Ctrl+C to stop
```

---

## Next Steps

Now that setup is complete, proceed to the [README.md](README.md) for:

- **Running Instructions:** How to launch all 5 terminals
- **Usage Guide:** Understanding the human-in-the-loop clarification
- **Architecture:** How the system components interact
- **Troubleshooting:** Solutions to common issues

### Quick Start Command Summary

Once everything is set up, you'll run these commands in separate terminals:

```bash
# Terminal 1: Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py use_sim_time:=True

# Terminal 2: Navigation2
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map_house.yaml

# Terminal 3: Perception (choose one)
source ~/yolo_clip_env/bin/activate
python3 ~/turtlebot3_ws/src/ri_pkg/ri_pkg/perception_internvl.py  # For simulation
# OR
python3 ~/turtlebot3_ws/src/ri_pkg/ri_pkg/perception_waffle.py    # For hardware

# Terminal 4: Autonomous Navigation
source ~/yolo_clip_env/bin/activate
python3 ~/turtlebot3_ws/src/ri_pkg/ri_pkg/autonomous_navigation_node.py

# Terminal 5 (Optional): Camera Viewer
source ~/yolo_clip_env/bin/activate
python3 ~/turtlebot3_ws/src/ri_pkg/ri_pkg/camera_subscriber.py
```

---

## Troubleshooting Setup Issues

### Issue: ROS 2 commands not found

**Solution:**

```bash
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### Issue: colcon build fails

**Solution:**

```bash
# Install missing dependencies
cd ~/turtlebot3_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Issue: CUDA not available in PyTorch

**Solution:**

```bash
# Check NVIDIA driver
nvidia-smi

# Reinstall PyTorch with CUDA support
source ~/yolo_clip_env/bin/activate
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
```

### Issue: Gazebo crashes or is very slow

**Solution:**

- Ensure you have a dedicated GPU
- Update GPU drivers
- Close unnecessary applications
- Reduce Gazebo rendering quality in world file

### Issue: TurtleBot3 packages not found

**Solution:**

```bash
cd ~/turtlebot3_ws/src
# Re-clone if necessary
git clone -b jazzy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Additional Resources

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Ultralytics YOLOv8 Documentation](https://docs.ultralytics.com/)
- [InternVL Documentation](https://huggingface.co/OpenGVLab/InternVL3_5-1B-Instruct)

---

## 🎥 Demo Videos

Check out our demo videos to see the system in action:

**📁 [View All Demo Videos on Google Drive](https://drive.google.com/drive/u/1/folders/1enSYF7NpExeNTTHyAyAxu4c-zXxf875O)**

- **Gazebo Simulation Demo** - Shows autonomous navigation in simulated environment
- **Hardware Demo (TurtleBot3 Waffle Pi)** - Shows the system running on real robot

---

**Setup Complete! 🎉**

You're now ready to run the autonomous navigation and semantic mapping system. Refer to [README.md](README.md) for detailed running instructions and [QUICK_START.md](QUICK_START.md) for a quick reference guide.
