# **Practice ROS2 with Turtlesim**

In this section we will practice the ROS2 concepts and different performances:
- Turtlesim environment
- Turtlesim control with a joystick (linebot)
- Turtlesim follow-me project: https://www.udemy.com/course/ros2-for-beginners/learn/lecture/21306698?start=1#overview

## **1. Turtlesim environment**

Turtlesim is a package that contains a 2D turtle robot simulation and is a good opportunity to apply what we have learnes of ROS2 in the previous chapter.

We will start with the main turtlesim node with:
```shell
ros2 run turtlesim turtlesim_node
```
![](./Images/02_ROS2_tutorial/01_turtlesim.png)

We can verify the nodes:
```shell
ros2 node list
```
We will start another node to control the turtlesim robot
```shell
ros2 run turtlesim turtle_teleop_key
```


## **2. Turtlesim control with a joystick**

You will install the package: https://index.ros.org/p/teleop_twist_joy/github-ros2-teleop_twist_joy/

You will use the github page (Humble branch): https://github.com/ros2/teleop_twist_joy/tree/humble

```shell
cd /home/mpuig/ros2_ws/src
git clone https://github.com/ros2/teleop_twist_joy.git -b humble
cd ..
colcon build --symlink-install
source install/setup.bash
```
Launch the node:
```shell
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```