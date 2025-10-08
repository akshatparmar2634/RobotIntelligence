# TurtleBot3 House World — ROS 2 Jazzy + Gazebo Harmonic

This setup runs the TurtleBot3 in the **house world**, displays the **camera feed**, and provides **keyboard control**.

---

## Terminal 1 — Launch Gazebo World

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

## Terminal 2 — Run Camera Subscriber

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run ri_pkg camera_subscriber
```

## Terminal 3 — Control Robot with Keyboard

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run turtlebot3_teleop teleop_keyboard
```


## For the yolo node:

source ~/yolo_clip_env/bin/activate
chmod +x ~/turtlebot3_ws/src/ri_pkg/ri_pkg/yolo_clip_camera_node.py
source ~/yolo_clip_env/bin/activate
python3 ~/turtlebot3_ws/src/ri_pkg/ri_pkg/yolo_clip_camera_node.py