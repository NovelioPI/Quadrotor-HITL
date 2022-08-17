# Quadrotor-HITL
Hardware-in-the-loop Simulation for Quadrotor using ROS and Gazebo

## Install
Clone from repository
$ ```
git clone --recurse-submodules https://github.com/NovelioPI/Quadrotor-HITL.git
```
Compile ROS
```
cd Quadrotor-HITL
catkin_make
source devel/setup.bash
```

## Launch simulator
```
roslaunch simulator_env start_env.launch
```
