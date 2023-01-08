# This is an example Docker File
#  Command to build it
# docker built -t <image name > .
FROM osrf/ros:foxy-desktop

RUN apt-get update
RUN apt-get install -y git && apt-get install -y python3-pip
RUN mkdir -p /home/ROS2_rUBot_ws/

RUN git clone https://github.com/manelpuig/ROS2_rUBot_ws /home/ROS2_rUBot_ws
RUN cd /home/ROS2_rUBot_ws/

RUN echo "ALL Done"