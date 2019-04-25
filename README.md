# DroneVision_Landing
Team project for the Robotics course at Duke: ME555-Advanced Robotics System Design

## Vision
Based on Aruco Marker system, use opencv marker detection algorithm to get the relative distance to the marker in x,y,z three dimensions.

![alt text](https://github.com/zhangjing1997/DroneVision_Landing/blob/master/readme_images/vision.png)

## Control
Taking PID as our basic control algorithm, we adapted two control schemes: 
1. simultaneous x,y,z velocity control 
2. only control x,y velocity before reaching to a certain range of the marker and then incorporate z control into the whole control system.

![alt text](https://github.com/zhangjing1997/DroneVision_Landing/blob/master/readme_images/control.png)

## State Estimation
Based on kalman filter, we only estimate the relative distance in x,y.

![alt text](https://github.com/zhangjing1997/DroneVision_Landing/blob/master/readme_images/state%20estimation.png)

## ROS system
Integrate the above modules on ROS framework and take simulation test in Gazebo world.

![alt text](https://github.com/zhangjing1997/DroneVision_Landing/blob/master/readme_images/ros_system.png)
