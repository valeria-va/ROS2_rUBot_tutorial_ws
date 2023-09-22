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

