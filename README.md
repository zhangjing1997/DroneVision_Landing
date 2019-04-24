# DroneVision_Landing
Team project for the Robotics course at Duke: ME555-Advanced Robotics System Design

## Vision Part
Based on Aruco Marker system, use opencv marker detection algorithm to get the relative distance to the marker in x,y,z three dimensions.

## Control Part
Taking PID as our basic control algorithm, we adapted two control schemes: 
1. simultaneous x,y,z velocity control 
2. only control x,y velocity before reaching to a certain range of the marker and then incorporate z control into the whole control system.
