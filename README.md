# SimplePlanner
A ROS2-integrated program that computes the path to a user selected goal from the current location of the robot

# Setup
Ubuntu 22.04 + ROS2 humble
## Installation of the [stage_ros2](https://github.com/tuw-robotics/stage_ros2) package:
```bash
sudo apt-get install git cmake g++ libjpeg8-dev libpng-dev libglu1-mesa-dev libltdl-dev libfltk1.1-dev
cd YOUR_ROS2_WORKSPACE
mkdir src
cd src
git clone --branch ros2 git@github.com:tuw-robotics/Stage.git
git clone --branch humble git@github.com:tuw-robotics/stage_ros2.git
cd YOUR_ROS2_WORKSPACE
colcon build --symlink-install --cmake-args -DOpenGL_GL_PREFERENCE=LEGACY
colcon build --symlink-install --packages-select stage_ros2        
```
# Build
```bash

```


# Run
From the root of the project folder:
```bash
source install/local_setup.bash
ros2 launch src/configuration/launch/launch_all.xml
```
## Commands to run each node individually:
### stage_ros2
```bash
ros2 run stage_ros2 stage_ros2 --ros-args -p world_file:=worlds/cappero_laser_odom_diag.world
```
### keyboard controller
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
It published ```geometry_msgs/msg/Twist``` messages in the ```/cmd_vel``` topic.

### map_server
```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=src/worlds/src/cappero/cappero_laser_odom_diag.yaml
```

to publish a static tf map->odom: 
```bash
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odo
```


### amcl
```bash
ros2 run 
```

### rviz2
```bash
ros2 run rviz2 rviz2 -d src/configuration/src/visualization.rviz
```