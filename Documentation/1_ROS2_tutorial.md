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

## **1. Docker Installation & Tools**
We will learn:
- Installation
- Starting docker ROS official images

### **1.1 ROS2 Installation**
Installation instructions could be found in:
- http://wiki.ros.org/docker
- https://docs.docker.com/get-docker/

In windows:
- download and install: https://docs.docker.com/desktop/install/windows-install/

    >   Open PowerShell terminal and type: systeminfo | find "Tipo de sistema"
    >
    >   The output has to be: x64-based PC
- restart your computer
- Open Docker Desktop
- Then you have to complete your installation with WSL 2 for kernell update in: https://learn.microsoft.com/ca-es/windows/wsl/install-manual#step-4---download-the-linux-kernel-update-package
- Stablish WSL2 as default by opening a powershell and typing: wsl --set-default-version 2
You need also to install Xlaunch for windows for GUI:
- https://sourceforge.net/projects/vcxsrv/

### **1.2 Starting docker ROS official images**
The official images are mantained by Open Source Robotics Foundation (OSRF).

You can find them in:
- https://registry.hub.docker.com/_/ros/
- https://hub.docker.com/r/osrf/ros/tags

Usefull images that could be opened with browser:
- ROS_Noetic: https://hub.docker.com/r/arvinskushwaha/ros-noetic-desktop-vnc/tags
- ROS2_Foxy: https://hub.docker.com/r/husarion/ros2-desktop-vnc

Interesting information is in:
- https://github.com/noshluk2/ros1_wiki/tree/main/docker

Let's install ROS1 and 2 official image:
```shell
docker pull osrf/ros:noetic-desktop-full
docker pull osrf/ros:foxy-desktop
```
To see all the images installed:
```shell
docker images -a
```
Creating a interactive container from image
```shell
docker run -it osrf/ros:noetic-desktop-full
```
Giving Name to a container while creating
```shell
docker run --name ROS1_Noetic_osrf -it osrf/ros:noetic-desktop-full
```
List of containers:
```shell
docker ps
```
Connect shell to running container
```shell
docker exec -it (container_id) bash
```
Source shell:
```shell
source /opt/ros/noetic/setup.bash
```
Start listener
```shell
rosrun rospy_tutorials listener
```
Start talker
```shell
rosrun rospy_tutorials talker
```
Stop  containers
```shell
docker kill 9a9b8eeaee46
```
Running a container with GUI enabled for Windows
```shell
docker run --name ROS1_Noetic_osrf -e DISPLAY=host.docker.internal:0.0 -it osrf/ros:noetic-desktop-full
```
Start a GUI
```shell
source /opt/ros/noetic/setup.bash
rosrun turtlesim turtlesim_node
```

**Some important trics:**
- To copy and paste text use the clipboard
- For home symbol use the "Extra keys"
- To copy files or folders from/to windows, open a Power Shell terminal and type:
```shell
docker cp c:/Users/puigm/Desktop/road1 Ros1_Noetic:/home/ubuntu/rUBot_mecanum_ws/src/rubot_mecanum_description/models
docker cp Ros1_Noetic:/home/ubuntu/rUBot_mecanum_ws/src/rubot_mecanum_description/worlds/road1.world c:/Users/puigm/Desktop
```

For HW image recording we will use "USB Image Tool".
This software will be used to create an image of SD card to share and copy to another SD card.
- Download the SW from: 
https://www.alexpage.de/usb-image-tool/download/


## 2. **Create workspace**

You can create a workspace with your desired name (usually finished with ws), for exemple "ROS2_rUBot_ws". Add a subfolder "src" where you will place the packages.

Usually you will clone a repositoty:
```shell
git clone https://github.com/manelpuig/ROS2_rUBot_ws
```

Proper documentation in: https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#



## 3. **Create first package**
You can create your first package inside the src folder with a name "ros2_tutorial"
```shell
ros2 pkg create --build-type ament_python ros2_tutorial
cd ..
colcon build --merge-install
```
> If colcon is not installed:
> - sudo apt install python3-colcon-common-extensions
>

Source the ws. Be sure in .bashrc file to have:
```shell
source /opt/ros/foxy/setup.bash 
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
cd /home/ubuntu/ROS2_rUBot_ws
source /home/ubuntu/ROS2_rUBot_ws/install/setup.bash
```

Create your first Package: https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html

## 4. **Create first Publisher and Subscriber nodes**
You can create your first Publisher and Subscriber using some templates.
- Create files "publisher_hello,py" and "subscriber_hello.py"
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
- Compile inside ws: 
    ```shell
    colcon build --merge-install
    ```
- Run:
    ```shell
    ros2 run ros2_tutorial publisher_node
    ros2 run ros2_tutorial subscriber_node
    ```


Create your first Publisher and Subscriber nodes: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

## 5. **Using parameters in a Class**

Detailed tutorial in: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html

## 6. **Create Launch files**

Detailed

## 7. **Githuib sync from docker**

When finished, **syncronize** the changes with your github. 
- Open a terminal in your local repository and type the first time:
```shell
git config --global user.email mail@alumnes.ub.edu
git config --global user.name 'your github username'
git config --global credential.helper store
```
- for succesive times, you only need to do:
```shell
git add -A
git commit -a -m 'message'
git push
```
- you will need to insert the username and the saved PAT password
- syncronize your repository several times to save your work in your github account
> - You can **update** your repository either in your local or **remote repository**:
>   - Local: with the previous instructions
>   - Remote: using web-based Visual Studio Code:
>       - pressing "·" key
>       - performing repository modifications
>       - typing "**git pull**" to syncronize