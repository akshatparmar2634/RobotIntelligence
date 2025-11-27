# Autonomous Navigation + YOLO VLM Semantic Mapping (ri_pkg)

This package delivers a full TurtleBot3 exploration stack that combines Nav2-based autonomous navigation, a YOLO + Vision-Language (VLM) perception pipeline, and a human-in-the-loop clarification workflow. The robot navigates through configurable waypoints, detects objects, and pauses when confidence is low so that an operator can confirm or relabel the detection.

## Table of Contents

- [Features](#features)
- [System Architecture](#system-architecture)
- [Folder Structure](#folder-structure)
- [Prerequisites & Dependency Installation](#prerequisites--dependency-installation)
- [Configuration](#configuration)
- [Building the Workspace](#building-the-workspace)
- [Running the Stack](#running-the-stack)
   - [Quick start with `run.sh`](#quick-start-with-runsh)
   - [Python wrapper `run.py`](#python-wrapper-runpy)
   - [Manual launch (advanced)](#manual-launch-advanced)
- [Human-in-the-Loop Clarification Flow](#human-in-the-loop-clarification-flow)
- [Expected Inputs & Outputs](#expected-inputs--outputs)
- [Troubleshooting](#troubleshooting)

## Features

- **Nav2-driven autonomous exploration** with configurable waypoint sequences and recovery behaviors.
- **YOLO + VLM semantic mapping** that logs detections, world coordinates, and descriptive metadata.
- **Human-in-the-loop confirmation** whenever detection confidence drops below the configured threshold.
- **Persistence and resume support** via JSON logs for navigation progress, detections, and human clarifications.
- **Convenience launchers (`run.sh`, `run.py`)** to orchestrate navigation and perception pipelines.

## System Architecture

```text
Gazebo / Real Robot
    ├──> /scan (LaserScan)  ─┐
    ├──> /odom (Odometry)    ├──> Autonomous Navigation Node (ri_pkg/autonomous_navigation_node.py)
    └──> /camera/color/image_raw ─┘          │
                                             │ publishes /cmd_vel, /exploration_active
                                             │ samples YOLO /yolo/detection_result
                                             └── pause/resume control + human clarification prompts

YOLO + VLM Node (ri_pkg/yolo_improved_vlm_node.py)
    ├──> Subscribes to /camera/color/image_raw, /odom, /scan
    └──> Publishes detection results to /yolo/detection_result, writes JSON logs

Human Operator Interface
    └──> Receives OpenCV window + console prompt when clarification is required

Persistent Data
    ├── navigation_waypoints.json (progress + resume)
    ├── detected_objects_vlm.json (semantic map)
    └── human_clarification.json (confirmed labels & confidences)
```

## Folder Structure

```text
ri_pkg/
├── launch/
│   ├── autonomous_navigation.launch.py   # Starts navigation node + camera subscriber
│   └── camera_subscriber.launch.py
├── ri_pkg/
│   ├── autonomous_navigation_node.py     # Nav2 client, human-in-loop logic
│   ├── yolo_improved_vlm_node.py         # YOLO + VLM perception pipeline
│   ├── camera_subscriber.py              # Optional image viewer
│   ├── coordinate_mapping.py             # World-coordinate utilities
│   └── ... (ASR, TTS, and auxiliary nodes)
├── run.sh                                # Bash launcher for the full stack
├── run.py                                # Python launcher delegating to run.sh
├── requirements.txt                      # Python dependencies for perception
├── navigation_waypoints.json             # Persisted waypoint progress
├── human_clarification.json              # Saved human confirmation records
├── detected_objects_vlm.json             # Semantic map output
├── commands.md                           # Extended launch recipes
└── README.md                             # You are here
```

## Prerequisites & Dependency Installation

1. **System packages**
   - ROS 2 Jazzy (or newer) with Nav2
   - TurtleBot3 packages (`turtlebot3`, `turtlebot3_gazebo`, `turtlebot3_navigation2`)
   - Gazebo Harmonic or compatible simulator (optional but recommended)

2. **Workspace setup**
   ```bash
   cd ~/turtlebot3_ws
   git clone <repo-url> src/ri_pkg   # if not already present
   source /opt/ros/jazzy/setup.bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Python environment for perception** (recommended: dedicated venv)
   ```bash
   python3 -m venv ~/yolo_clip_env
   source ~/yolo_clip_env/bin/activate
   pip install --upgrade pip wheel
   pip install -r ~/turtlebot3_ws/src/ri_pkg/requirements.txt
   deactivate  # reactivate when running the perception node
   ```

## Configuration

- `TURTLEBOT3_MODEL` (env var): choose the robot model (`waffle_pi` default).
- `YOLO_ENV` (env var): path to the virtual environment for VLM (`~/yolo_clip_env`).
- `autonomous_navigation_node.py` parameters:
  - `camera_topic` (default `/camera/color/image_raw`)
  - `LOW_CONFIDENCE_THRESHOLD` (default `0.5`)
  - JSON outputs stored in the package root (`navigation_waypoints.json`, `human_clarification.json`, `detected_objects_vlm.json`).
- Waypoint definitions are loaded from `navigation_waypoints.json`. Edit this file to customize traversal order without changing code.

## Building the Workspace

```bash
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ri_pkg
source install/setup.bash
```

## Running the Stack

> **Before launching**: start Gazebo (or connect to a real robot) and bring up Nav2 with your map. Make sure `/odom`, `/scan`, and the camera topics are available.

### Quick start with `run.sh`

```bash
cd ~/turtlebot3_ws/src/ri_pkg
chmod +x run.sh
./run.sh all
```

- `all` (default): launches the autonomous navigation launch file and, after a short delay, the YOLO+VLM node (activates the virtual environment defined by `YOLO_ENV`).
- `./run.sh navigation`: only start the navigation stack.
- `./run.sh vlm`: only run the perception pipeline (requires the virtual environment).

Pass additional ROS launch arguments after `--`, e.g. `./run.sh navigation -- auto_start:=false`.

### Python wrapper `run.py`

```bash
cd ~/turtlebot3_ws/src/ri_pkg
python3 run.py all
```

`run.py` delegates to `run.sh` and offers the same modes. Example:

```bash
python3 run.py vlm -- --ros-args -p detector.model_variant:=yolov8n
```

### Manual launch (advanced)

1. **Sim / Nav2 terminals** (examples):

   ```bash
   # Terminal A
   source /opt/ros/jazzy/setup.bash
   export TURTLEBOT3_MODEL=waffle_pi
   ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py use_sim_time:=true

   # Terminal B
   source /opt/ros/jazzy/setup.bash
   source ~/turtlebot3_ws/install/setup.bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=$HOME/map_house.yaml
   ```

2. **Navigation + human-in-loop node**

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/turtlebot3_ws/install/setup.bash
   ros2 launch ri_pkg autonomous_navigation.launch.py auto_start:=true
   ```

3. **YOLO + VLM perception**

   ```bash
   source ~/yolo_clip_env/bin/activate
   source /opt/ros/jazzy/setup.bash
   source ~/turtlebot3_ws/install/setup.bash
   ros2 run ri_pkg yolo_improved_vlm_node
   ```

4. (Optional) Open a camera viewer:

   ```bash
   ros2 run ri_pkg camera_subscriber
   ```

## Human-in-the-Loop Clarification Flow

1. YOLO+VLM publishes detections to `/yolo/detection_result`.
2. `autonomous_navigation_node` evaluates confidence scores. If a detection is below the threshold and the label has not been confirmed before, it:
   - Cancels the current navigation goal and stops the robot.
   - Freezes the latest RGB frame, draws the bounding box, and displays it in an OpenCV window.
   - Prompts the operator in the console to confirm or provide the actual label.
3. The chosen label is logged to `human_clarification.json` with the detection metadata. Future detections with the same label or signature are ignored.
4. Navigation automatically resumes after the clarification is recorded.

## Expected Inputs & Outputs

### Inputs

- ROS topics: `/odom`, `/scan`, `/camera/color/image_raw`, `/yolo/detection_result`
- Configuration files: `navigation_waypoints.json`, optional parameter overrides
- Human input: console confirmation when prompted

### Outputs

- `/cmd_vel`, `/exploration_active` ROS topics from the navigation node
- `detected_objects_vlm.json`: semantic map with YOLO + VLM metadata
- `human_clarification.json`: chronological list of confirmed detections
- `navigation_waypoints.json`: updated progress checkpoints for resuming exploration
- Optional exploration summaries (`exploration_report.json`) and detection artifacts in `ri_pkg/`

## Troubleshooting

- **No camera frame in clarification window**: confirm the camera topic matches `camera_topic` parameter and that the camera pipeline is running.
- **VLM node fails to start**: ensure the `YOLO_ENV` virtual environment exists and dependencies are installed.
- **Navigation goal rejected**: initialize the robot pose in RViz before starting exploration.
- **Repeated prompts for the same object**: check that `human_clarification.json` is writable; the node suppresses repeated labels once the file is updated.
- **Environment sourcing errors**: rerun `colcon build`, then source `install/setup.bash` before launching scripts.

For extended command sequences and additional utilities, see `commands.md` in this directory.
