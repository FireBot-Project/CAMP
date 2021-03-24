# Coordinated Autonomous Mapping & Pathfinding

## Project Depenendencies
* [OpenCV](https://github.com/opencv/opencv)
* [Intel Realsense Library](https://github.com/IntelRealSense/librealsense)
* [Multimaster FKIE](https://github.com/fkie/multimaster_fkie)

## How to build project

Before building this project, be sure to install the project dependencies listed above. Once setup, run the following commands in the terminal.
More information on building and installing multimaster_fkie can be found on its [Wiki page.](http://wiki.ros.org/multimaster_fkie)

```
git clone https://github.com/CAMP-Project/CAMP.git

cd CAMP

catkin_make
```

## LIDAR Data Structure
Variable Name| Data
-------------|-------
angle_min | Start angle of the scan [rad]
angle_max | End angle of the scan [rad]
angle_increment | angular distance between measurements [rad]
scan_time | Time between [seconds]
range_min | minimum range value [m]
range_max | maximum range value [m]
ranges | range data array [m]
intensities | intensity data array [device-specific units]
