# needle_pantograph_ros2
ROS2 stack to use a 2-Dof pantograph used at the ICube laboratory for needle insertion simulation.

## Installation

### Prepare the environment

1) Install ROS2 Humble and the usual tools (Gazebo, Colcon, etc.)

2) Install EtherLab as specified in the [documentation of ethercat_driver_ros2](https://icube-robotics.github.io/ethercat_driver_ros2/).

> [!TIP]
> use the `ethercat slaves` CLI command to check that all is OK andf that you can see your device.

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
```

```bash
# Launch with mock hardware
ros2 launch pantograph_bringup pantograph.launch.py

# Publish torque command
ros2 topic pub -r 200 \
     /forward_effort_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, 0.0]"

# Mock pHRI (start an interactive RVIZ marker to interact with the pantograph)
ros2 run pantograph_nodes interactive_fake_operator

```

```bash
# With the actual robot
sudo /etc/init.d/ethercat start  # start ETherLab daemon
ros2 launch pantograph_bringup pantograph.launch.py use_fake_hardware:=false
```
## Troubleshouting

To ensure that the pantograph movement match the real robot, change the parameters of the minimal and maximal angles of the joints `panto_a1` and `panto_a5` in the [urdf description](pantograph_description/urdf/pantograph.urdf.xacro)
If the movement of the proximal segments in the simulation does not match the real movement of the pantograph (axes are inversed), you can change the configuration of the etherCat files in the [config](pantograph_description/config) folder.
You must inverse the sign of the value `factor` in the 4 beckhoff files.
