# Sevenbot Project
Sevenbot is a prototype for setting up a ROS based simulation workspace. 

# Simple Instructions
This repo is already a catkin ws, clone build and source.

1. Launch Gazebo to get TF
```
roslaunch sevenbot_gazebo empty_world.launch
```

2. Launch RVIZ to see robot with Gazebo
```
roslaunch sevenbot_description rviz.launch
```

3. For control part, set 'key' to false and 'circle' to true to perform auto pilot
```
roslaunch sevenbot_control sevenbot_control.launch key:=true circle:=true dialength:=1.0
```

# Current Status
- Writing code using ROS navigation stack to perform a square webpoints.

# Current Issues
Robot controller can not response correctly.
Going to write my own controller later.

# Contact
Peng
he.p@me.com
