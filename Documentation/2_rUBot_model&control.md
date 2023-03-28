## **2. ROS2 rUBot model and Control**
The objectives of this section are:
- Develop a custom model of rUBot in virtual environment
- Create a Control package in virtual environment

Interesting project to take into account:
- https://github.com/noshluk2/ROS2-Raspberry-PI-Intelligent-Vision-Robot

### **2.1. ROS2 rUBot model**
The objectives of this section are:
- Create a new "rubot_description" package with 
    - the rubot model in URDF format
    - the different part models and custom designed worlds
- Spawn the rubot model in a proper virtual world in gazebo environment

#### **2.1.1. Create a new "rubot_description" package**
To create this package, type:
```shell
ros2 pkg create --build-type ament_python rubot_description --dependencies rclpy
```
Now proceed with the following instructions:
- add "launch" and "urdf" folders
- provide the path to these new folders. This can be done in setup.py file. We have to add some lines at the begining to import some libraries and specify the folder paths:
```python
from setuptools import setup
import os
from glob import glob 
package_name = 'rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
        
    ],
...
```
- Add urdf models for different robots and save them in urdf folder
- In launch folder add files to print the robot models in rviz and gazebo. Use the following templates:
- for rover_rviz.launch.py:
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('mr_rehri')
    urdf = os.path.join(package_dir,'1_rover.urdf')
    rviz_config_file=os.path.join(package_dir,'config.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf]),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            arguments=[urdf]),

        Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',rviz_config_file],
        output='screen'),
    ])
```
- for rover_gazebo_spawn.launch.py
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  urdf = os.path.join(get_package_share_directory('mr_rehri'),'1_rover.urdf')
  return LaunchDescription([
    #   publishes TF for links of the robot without joints
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf]),
    #  To publish tf for Joints only links
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            ),
#  Gazebo related stuff required to launch the robot in simulation
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "dolly"])
  ])
```
- Be sure to have installed the needed packages:
```shell
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-gazebo-ros-pkgs
```


- compile the final package

Now you can launch the rover in rviz and gazebo
```shell
ros2 launch rubot_description rover_rviz.launch.py
```


### **2.2. ROS2 rUBot control**
The objectives of this section are:
- Create a new "rubot_control" package  
- Create ROS2 programs to control the robot to:
    - move with teleop keyboard
    - move with joy
    - move with speciffic node
    - obstacle avoidance
    - wall follower
    - line follower
#### **2.2.1. Create a new "rubot_control" package**
To create this package, type:
```shell
ros2 pkg create --build-type ament_python rubot_control --dependencies rclpy
```
Now proceed with the following instructions:
- add "launch" folder