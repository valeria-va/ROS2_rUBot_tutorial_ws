# **Practice ROS2 with Turtlesim**

In this section we will practice the ROS2 concepts and different performances:
- Turtlesim environment
- Turtlesim control with a joystick (linebot)
- Turtlesim follow-me project (Edouard Renard)

## **1. Turtlesim environment**


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