# Local path planning

## Overview
This package simulates a lunar rover navigating a moon-like terrain. It integrates:
- Gazebo Harmonic simulation
- ROS 2 Navigation Stack (NAV2)
- Obstacle avoidance using LiDAR and RGBD camera
- RViz2 visualization
- OpenCV

## Features
- **Autonomous Navigation**: Uses Nav2 to plan and follow paths.
- **Obstacle Avoidance**: Uses NAV2 parameters with highly tuned weights to the DWB controller to dynamically avoid obstacles using LiDAR, camera, and costmap data.
- **Simulated Environment**: Moon terrain modeled in Gazebo.
- **Visualization**: Supports RViz2 for monitoring robot state and environment.
- **OpenCV**: Custom node 'trajectory_plotting.py' supports OpenCV for trajectory plotting.

### Installation

Clone into your ROS2 workspace and build:

```bash
cd ~/ros2_ws/src
git clone https:https://github.com/lucasnaury/lunar-navigation.git
cd ..
colcon build --packages-select euro2moon
ros2 launch euro2moon local_path_planning.py use_moon:=True use_o2_rover:=True
ros2 run euro2moon fake_path --ros-args -p use_moon:=True
```

## Explanation
- `use_o2_rover:=True` : is used to import the URDF file of our rover from its CAD files. If set to false, it uses the simple rover.
- `use_moon:=True` : is used to run the simulation on the procedurally generated map. If set to false, it runs the simulations on the basic map with simple obstacles.
