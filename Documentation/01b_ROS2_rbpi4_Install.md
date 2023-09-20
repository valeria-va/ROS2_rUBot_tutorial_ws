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

## **Install Ubuntu22 on Raspberrypi4**

Follow instructions:
- Use RaspberryPi Imager and choose Ubuntu22 Desktop (64bits)
- Connect ethernet cable to your raspberrypi4
- Insert SD card in RBPI4 and switch on
- select Language and keyboard 
- Select:
    - username: mpuig
    - Password: 1234
- Select your wifi
- Configure a hotspot:
    - Select wifi settings
    - Turn on hotspot
        - name: rubot_ros2
        - pass: rubot_ros2
    - Make the connection "Hotspot" start automatically:
    ```shell
    nmcli con mod Hotspot connection.autoconnect yes
    ```
- update, upgrade and Install ssh service:
```shell
sudo apt update
sudo apt upgrade
sudo apt-get install openssh-server
```
- reboot
- Change the Hotspot settings (name or password):
```shell
sudo nm-connection-editor
```

**Connect to the rbpi4:**
- remotelly with SSH
```shell
cmd
ssh pi@10.42.0.1
```
> Perhaps the security Key will be reestablished and you will need to delete the line for the old key in C:\Users\puigm\ .ssh\known_hosts
- With screen and keyboard

    - Sometimes when you connect a USB the screen service is corrupted. Then reestart the service from the remote connection:
    ```shell
    sudo service gdm3 restart
    ```
- With a remote Desktop **Nomachine** (recommended):
    - In your raspberryPi4 install Nomachine (arm64, DEB package): https://downloads.nomachine.com/download/?id=107&distro=Raspberry&hw=Pi4

    - In PC open NoMachine viewer
    - Select the raspberrypi IP address: 10.42.0.1
    - you have to specify:
        - user: mpuig
        - password: 1234
    - Choose "scale remote display to the window"
    - You will have the raspberrypi4 desktop on your windows NoMachine screen

## **Install ROS2 Humble on Raspberrypi4**

Follow instructions in: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

I suggest to install **ros-humble-desktop**

If you have installed "ros-humble-ros-base", you will need to:
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
> You will have to add mpuig user to the group (this appears as warning when executing arduino first time)
```shell
sudo usermod -a -G dialout mpuig
```
> You have to shutdown the RaspberryPi4 before continue with the installation!!
- run arduino
- Install on arduino IDE the ESP32 board from Espressif Systems (version 2.0.2 or later)
https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html
- Update the Boards firmware if needed
- Connect ESP32 board
- Select the ESP32 Dev Module board and port ttyUSB0
- Add arduino uROS libraries: 
    - https://github.com/micro-ROS/micro_ros_arduino
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

- we have to install dependencies for packages in ws. Later we compile the new package
```shell
cd ~/ros2_ws
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/local_setup.bash
```

- Now run the microROS agent:
```shell
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```
- press enable button to stablish communication
- Verify the topic list and the publication of the sketch:
```shell
ros2 topic list
ros2 topic echo /micro_ros_arduino_node_publisher
```
Congratulations!. You have now the micoros agent working and you have good communication between ESP32 and RaspberryPi4!!

