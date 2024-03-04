# needle_pantograph_ros2
ROS2 stack to use a 2-Dof pantograph used at the ICube laboratory for needle insertion simulation.

## Installation

### Prepare the environment

1) Install ROS2 Humble and the usual tools (Gazebo, Colcon, etc.)

2) Install EtherLab as specified in the [documentation of ethercat_driver_ros2](https://icube-robotics.github.io/ethercat_driver_ros2/).

### Install this package

```bash
WS_PANTOGRAPH=~/dev/ws_pantograph_ros2/
mkdir -p $WS_PANTOGRAPH/src
cd $WS_PANTOGRAPH/src

git clone https://github.com/ICube-Robotics/needle_pantograph_ros2.git
vcs import . < needle_pantograph_ros2/needle_pantograph_ros2.repos
rosdep install --ignore-src --from-paths . -y -r

cd ..
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source install/setup.bash
```

## Getting started

```bash
cd $WS_PANTOGRAPH
source install/setup.bash
ros2 launch pantograph_bringup pantograph.launch.py
```
