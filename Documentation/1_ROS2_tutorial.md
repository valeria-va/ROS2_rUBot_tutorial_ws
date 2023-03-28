# **ROS2 tutorial**

ROS1 is close to finish and you can switch to ROS2
![](./Images/7_ROS2_time.png)
ROS2 structure is based on the architecture:
![](./Images/7_ROS1_ROS2.png)
The main differences are:
![](./Images/7_ROS2_dif.png)

ROS2 is a very good choice.

Interesting references for courses:
- Edouard Renard: https://www.udemy.com/course/ros2-for-beginners/learn/lecture/20260476#overview
- https://www.udemy.com/course/learn-ros2-as-a-ros1-developer-and-migrate-your-ros-projects/learn/lecture/22003074#overview
- https://www.udemy.com/course/ros2-ultimate-mobile-robotics-course-for-beginners-opencv/learn/lecture/28143024#overview
- https://www.udemy.com/course/ros2-self-driving-car-with-deep-learning-and-computer-vision/learn/lecture/28236852#overview

Some interesting projects:
- https://github.com/noshluk2/ROS2-Raspberry-PI-Intelligent-Vision-Robot
- https://github.com/noshluk2/ROS2-Ultimate-Mobile-Robotics-Course-for-Beginners-OpenCV
- https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV

And the ROS2 reference sites:
- https://docs.ros.org/
- https://docs.ros.org/en/foxy/

Installation will be made using Docker

## **1. ROS2 workspace and packages**

The **ROS2 workspace** is the directory in your hard disk where your ROS2 packages reside to be usable by ROS2.

ROS2 uses **packages** to organize its programs. You can think of a package as all the files that a specific ROS2 program contains

In ROS2, you can create two types of packages: Python packages and CMake (C++) packages.

Every **Python package** will have the following structure of files and folders:

- package.xml - File containing meta-information about the package (maintainer of the package, dependencies, etc.).

- setup.py - File containing instructions for how to compile the package.

- setup.cfg - File that defines where the scripts will be installed.

- /<package_name> - This directory will always have the same name as your package. You will put all your Python scripts inside this folder. Note that it already contains an empty __init__.py file.

Some packages might contain extra folders. For instance, the launch folder contains the package's launch files 

We wil use the "ROS2_rUBot_ws" as starting workspace for this project.

You open a terminal in this Container using VS Code:
```shell
cd /home
git clone https://github.com/manelpuig/ROS2_rUBot_ws
```
**Create a new package**

Every time you want to create a package, you have to be in this directory src. Type into your Webshell the following command:
```shell
ros2 pkg create --build-type ament_python my_package --dependencies rclpy
For exemple
ros2 pkg create --build-type ament_python ros2_tutorial --dependencies rclpy
```
>the <package_dependencies> are the names of other ROS2 packages that your package depends on.

Now you have to build the created ws:
```shell
cd /home/ROS2_rUBot_ws
colcon build
```
> Important warning solutions:
>- If warnings related to "SetuptoolsDeprecationWarning: setup.py install is deprecated". Install setup tools version 58.2.0 (last version to work with ros2 python packages without any warnings)
>   - pip install setuptools==58.2.0
>- If warnings related to CMAKE_PREFIX_PATH, AMENT_PREFIX_PATH environment variables non existing values, reset them with:
>   - export AMENT_PREFIX_PATH=""
>   - export CMAKE_PREFIX_PATH=""

Source the workspace. Be sure in .bashrc file (in root folder) to have:
```shell
source /opt/ros/humble/setup.bash 
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
cd /home/ROS2_rUBot_ws
source /home/ROS2_rUBot_ws/install/setup.bash
```
You are ready to work with this workspace

Some interesting commands:
- ros2 pkg list: Gives you a list of all your ROS system packages.

- ros2 pkg list | grep my_package: Filters, from all of the packages located in the ROS system, the package is named my_package.

**Compile a Package**

Sometimes (for large projects), you will not want to compile all of your packages. This would take such a long time. So instead, you can use the following command to compile only the packages where you have made changes:
```shell
colcon build --packages-select <package_name>
```

## **2. ROS2 Nodes**
In ROS2, each node should be responsible for a single module (e.g., one node for controlling wheel motors, one for controlling a LIDAR control, etc.). Each node can communicate with other nodes through different methods.

A full robotic system is comprised of many nodes working together. In ROS2, a single executable (a C++ or Python program, etc.) can contain one node.

**Create first ROS2 Program**

You can create a program based on two nodes:
- A Publisher node that publish a string message "Hello world" to a "/pub_topic" topic
- A subscriber node that subscribes to "/pub_topic" and prints on the screen the read message.

Each node is created within a python program in "src/ros2_tutorial/ros2_tutorial" folder:
- "publisher_hello.py": creates a simple_publisher node
- "subscriber_hello.py": creates a simple_subscriber node

You can create Publisher and Subscriber python files using speciffic templates.

Next step is to generate an executable from the python scripts you created. This is done in the "setup.py" file. The **setup.py** file contains all the necessary instructions for properly compiling your package. To do that, you work with a dictionary named entry_points. Inside it, you find an array called console_scripts.

- Add entry points for Publisher and Subscriber
    
    Reopen setup.py and add the entry point for the subscriber node below the publisher’s entry point. The entry_points field should now look like this:
    ```python
    entry_points={
        'console_scripts': [
                'publisher_node = ros2_tutorial.publisher_hello:main',
                'subscriber_node = ros2_tutorial.subscriber_hello:main',
        ],
    },
    ```
You can see these lines as follows:    '<executable_name> = <package_name>.<script_name>:main'

- Compile inside ws: 
    ```shell
    colcon build
    ```
- Run in a separate terminals:
    ```shell
    ros2 run ros2_tutorial publisher_node
    ros2 run ros2_tutorial subscriber_node
    ```
## **3. Create Launch files**

ROS2 programs can be run:
- from executable files
- from launch files. 

When using executable files, we need one terminal to run each node. 

To make easier this process using a unique terminal to launch multiple nodes, we will use launch files.

The launch system in ROS 2 is responsible for helping the user describe the configuration of their system and then execute it as described. Launch files written in Python, XML, or YAML can start and stop different nodes as well as trigger and act on various events. 

To create a launch file we have to:
- create a "launch" folder in "src/ros2_tutorial/"
- Inside create the launch file in different formats:
- Launch file in Python: "hello_pub_sub.launch.py" file and make it executable
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_tutorial',
            executable='publisher_node',
            output='screen'),
        Node(
            package='ros2_tutorial',
            executable='subscriber_node',
            output='screen'),
    ])
```
- Launch file in XML: "hello_pub_sub.xml" file
```xml
<launch>
<node pkg="ros2_tutorial" exec="publisher_node"/>
<node pkg="ros2_tutorial" exec="subscriber_node"/>
</launch>
```
- Open setup.py file and make some modifications to:
    - Import some libraries
    ```shell
    import os
    from glob import glob
    ```
    - share this launch folder to the executable path. Add this line in "data_files"
    ```shell
    (os.path.join('share', package_name), glob('launch/*'))
    ```
- compile again

Now you can execute the launch file:
```shell
ros2 launch ros2_tutorial hello_pub_sub.launch.py
```
**Exercise:**

Make a ROS2 program based on 2 nodes:
- A Publisher node that publish a INT32 number to a "/pub_topic" topic
- A subscriber node that subscribes to "/pub_topic", multiplies this number by 2 and prints on the screen the resulting value.

Construc the corresponding launch file to execute the program


## **4.Client Libraries**
ROS client libraries allow nodes written in various programming languages to communicate. A core ROS client library (RCL) implements the standard functionality needed by various ROS APIs. This makes it easier to write language-specific client libraries.

The ROS2 team currently maintains the following client libraries:

- rclcpp = C++ client library
- rclpy = Python client library

Additionally, other client libraries have been developed by the ROS community. You can find some more information in:
https://docs.ros.org/en/humble/Concepts/About-ROS-2-Client-Libraries.html
