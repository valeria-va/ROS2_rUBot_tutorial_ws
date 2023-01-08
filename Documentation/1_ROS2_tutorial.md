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

## **1. Clone a starting workspace**
We wil use the "ROS2_rUBot_ws" as starting workspace for this project.

Run a Container:
```shell
docker run --name ROS2_Foxy_osrf -it osrf/ros:foxy-desktop
git clone https://github.com/manelpuig/ROS2_rUBot_ws
```
If you want to creat

## 2. **Create workspace**

You can create a workspace in your github account with your desired name (usually finished with ws), for exemple "ROS2_rUBot_ws". Add a subfolder "src" where you will place the packages.

Usually you will clone this created ws in a docker container:
```shell
docker run --name ROS2_Foxy_osrf -it osrf/ros:foxy-desktop
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