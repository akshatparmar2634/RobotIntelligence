# Camera Subscriber for TurtleBot3

This package contains a ROS 2 node that subscribes to the `/camera/image_raw` topic from the TurtleBot3 Waffle camera and displays the images using OpenCV.

## Prerequisites

Make sure you have the following dependencies installed:
- ROS 2 (Jazzy or compatible)
- OpenCV for Python (`pip install opencv-python`)
- cv_bridge ROS package
- TurtleBot3 packages

## Building

```bash
cd ~/turtlebot3_ws
colcon build --packages-select ri_pkg
source install/setup.bash
```

## Running

### Method 1: Using the launch file
```bash
ros2 launch ri_pkg camera_subscriber.launch.py
```

### Method 2: Running the node directly
```bash
ros2 run ri_pkg camera_subscriber
```

## Usage

1. Start your TurtleBot3 Waffle simulation in Gazebo:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py model:=waffle
   ```

2. Run the camera subscriber (using either method above)

3. An OpenCV window titled "TurtleBot3 Camera Feed" will open showing the live camera feed

4. Press 'q' in the image window to quit the application

## Notes

- The node subscribes to the `/camera/image_raw` topic
- Images are displayed in real-time using OpenCV
- The application will automatically handle ROS image to OpenCV format conversion
- Make sure the TurtleBot3 simulation is running before starting the camera subscriber