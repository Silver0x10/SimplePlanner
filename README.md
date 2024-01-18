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
```
```


# Run
```bash
source install/local_setup.bash
```

## stage_ros2
```bash
ros2 run stage_ros2 stage_ros2 --ros-args -p world_file:=worlds/cappero_laser_odom_diag_obstacle.world
```