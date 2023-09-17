# **ROS2 Humble install on RaspberryPi4**

The procedure is:
- Install Ubuntu22.04 server (64bits) image using "Pi Imager"
- connect using SSH
- Install ubuntu-desktop
- Install ros2 Humble

You can follow the instructions on:
- https://roboticsbackend.com/install-ubuntu-on-raspberry-pi-without-monitor/
- https://roboticsbackend.com/install-ros2-on-raspberry-pi/
- https://github.com/ros-realtime/ros-realtime-rpi4-image

Make sure you have selected:
- user: pi
- Pass: 1234

Connect to the rbpi4:
- remotelly with SSH
```shell
cmd
ssh pi@192.168.54.219
```
- With screen and keyboard

    - Sometimes when you connect a USB the screen service is corrupted. Then reestart the service from the remote connection:
    ```shell
    sudo service gdm3 restart
    ```
    - Select the wifi to connect

**Install some needed packages**
- Install ROSDEP2
```shell
sudo apt install python3-rosdep2
rosdep update
```
- Install C++ compiler (g++)
```shell
sudo apt-get install g++
export CXX=g++
```
> add this last instruction "export" in ~/.bashrc

## 1. **microROS Installation**

Follow instructions in:
- https://www.udemy.com/course/ros2-for-beginners-build-your-first-robot-with-esp32/learn/lecture/39333626?start=45#overview

Official links:
- uROS official website: https://micro.ros.org/docs/overview/features/
- uROS arduino library: https://github.com/micro-ROS/micro_ros_arduino
- Setup ESP32 in arduino IDE: https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/
- uROS agent: https://github.com/micro-ROS/micro-ROS-Agent
- Install ROS dependencies: https://wiki.ros.org/rosdep


![](./Images/01_ROS2_install/4_uROS_connection.png)

uROS for Arduino supports ESP32, for that we will have to:
- Install arduino in rbpi4 board:
```shell
sudo snap install arduino
```
> You will have to add pi user to the group (this appears as warning when executing arduino first time)
```shell
sudo usermod -a -G dialout pi
```
- Install on arduino IDE the ESP32 board from Espressif Systems (version 2.0.2 or later)
https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html
- Connect ESP32 board
- Add arduino uROS libraries: https://github.com/micro-ROS/micro_ros_arduino
- Select Humble branch and download in zip
- Add this library in arduino IDE
- Select exemple "micro-ros_publisher.ino" and compile it
- Now we will connect ESP32 to arduino USB and upload the sketch

In order to properly communicate microROS sketch in ESP32 to ROS2 in RaspberryPi4, we need to install microROS Agent: https://github.com/micro-ROS/micro-ROS-Agent
- Go to the microROS Agent web site and select Humble
- copy the URL to clone this repository
- clone the repository to src folder in raspberrypi4 ros2_ws project
```shell
cd ~/ros2_ws/src
git clone https://github.com/micro-ROS/micro-ROS-Agent.git -b humble
```

- compile the new package
```shell
cd ..
colcon build --symlink-install
```
- If there are some errors because it not find some other packages, we have to install dependencies for packages in ws. Type for that, first install rosdep2 package:
```shell
sudo apt install python3-rosdep2
rosdep update
cd src
rosdep install --from-paths src --ignore-src -r -y
```
- Compile again and source:
```shell
colcon build --symlink-install
source install/local_setup.bash
```
- Now run the microROS app:
```shell
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```
- press enable button to stablish communication
- Verify the topic list and the publication of the sketch:
```shell
ros2 topic list
ros2 topic echo /micro_ros_arduino_node_publisher
```


