# **ROS2 Humble install**

ROS Installation will be made using Docker

The **official images** are mantained by Open Source Robotics Foundation (OSRF).

You can find them in:
- https://registry.hub.docker.com/_/ros/
- https://hub.docker.com/r/osrf/ros/tags

Let's install ROS2 Humble official image:
```shell
docker pull osrf/ros:humble-desktop-full
```
Let's create a container with shared folder:
```shell
docker run --name ROS2_Humble_osrf -e DISPLAY=host.docker.internal:0.0 --mount src="C:\Users\puigm\Desktop\ROS_github\myPC_shared",dst=/home/myDocker_shared,type=bind -it osrf/ros:humble-desktop-full
```
The documentation will be found in: https://docs.ros.org/en/humble/index.html

