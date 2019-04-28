# wave_collector

This module facilitates data collection for neural networks and testing.

## Isolated Quickstart
Start ROS

``` roscore ```

Start the camera node in this directory with

``` roslaunch camera.launch ```

Start the collector with python2

``` python src/collector.py ```


Drive the robot around and ensure ensure cmd_vel topic is correct in ``` collector.py ``` , if extracting from a rosbag then use ``` rosbag play drive.bag ```

Output will be visible in the ``` collector_x ``` directories where x is an incrementing number
