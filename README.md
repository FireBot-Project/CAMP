# Coordinated Autonomous Image-based Fire Detection System
Our goal is to build a moving image-based fire detection system that can answer the limitations of traditional fire detection technologies. FireBot moves along fire and provides a real-time report. These two key features can play a crucial role in reducing manual efforts to understand the dynamics of fire and losses by alarming users early through early fire detection.

## Project Depenendencies
* [Multimaster FKIE](http://wiki.ros.org/multimaster_fkie)
* [Python2 version of OpenCV](https://github.com/opencv/opencv)
* ROS Packages listed in the ROS Installation Document

The two Documents in this repository contain information on Getting Started with this project and even getting the specific ROS packages installed to be able to build the software stack.
* [ROS_Installation.txt](https://github.com/FireBot-Project/CAMP/raw/master/ROS_Installation.txt)
* [Jetson_Nano_Installation.txt](https://github.com/FireBot-Project/CAMP/raw/master/Jetson_Nano_Installation.txt)

## How to build project

1. Clone project directory
  git clone https://github.com/FireBot-Project/CAMP.git

2. Install ROS
  View ROS_Installation.txt for desktop/laptop
  View Jetson_Nano_Installation.txt for Jetson Nano 

3. Build project
  cd CAMP
  git submodule update --init
  catkin_make

## Project Contents
Here are our interesting or new contributions to the project. There are more packages in the srcfolder, but some of them come with the turtlebot or where inherited from previous groups.
### [Brian](https://github.com/CAMP-Project/CAMP/tree/master/src/brian) - The Multimaster Arbitrator
The Brian folder contains code related to the multimaster arbitrator. The arbitrator helps package topics up in a way where other robots can use the data.

Launch files:
* brian.launch - launches turtlebot bringup and map.

Source code:
* MMArbitrator.cpp - its the arbitrator, breif description

### [Camp](https://github.com/CAMP-Project/CAMP/tree/master/src/camp) - Goto, Pathfinding, and Maps.
#### [Camp Goto](https://github.com/CAMP-Project/CAMP/tree/master/src/camp/camp_goto) - Navigation to a Point
#### [Camp Map](https://github.com/CAMP-Project/CAMP/tree/master/src/camp/camp_map) - Mappping and Map Transforms
#### [Camp Multiagent](https://github.com/CAMP-Project/CAMP/tree/master/src/camp/camp_multiagent) - Working together
#### [Camp Pathfinding](https://github.com/CAMP-Project/CAMP/tree/master/src/camp/camp_pathfinding) - Finding a Way
### [Coop Test](https://github.com/CAMP-Project/CAMP/tree/master/src/coop_test) - Testing Cooperation
### [Kalman Filter](https://github.com/CAMP-Project/CAMP/tree/master/src/kalman_filter) - Smoothing Time-of-Flight Data
### [Lidar Listener](https://github.com/CAMP-Project/CAMP/tree/master/src/lidar_listener) - Our First Sensor Test
### [Roomba Navigation](https://github.com/CAMP-Project/CAMP/tree/master/src/roomba_navigation) - Minimum Deliverable Product
### [TF Test Environment](https://github.com/CAMP-Project/CAMP/tree/master/src/tf_test_environment) - Figuring Out Transforms
