# **ROS2 Exercise Turtlesim**

The objective of this section is to solve the exercise.

**Exercise:**

Create a new package "ros2_move_turtle" to control the movement of the previous Turtlesim robot.

![](./Images/02_ROS2_tutorial/02_move_turtle.png)

The program functionality will be based on 2 nodes:
- The "/turtlesim" node we have already practice in last section
- A new "/move_turtle" node that:
    - subscribes to the "/turtle1/Pose" topic
    - Publish to the "/turtle1/cmd_vel" topic a message Twist
    - if the running time is greater to 10s, the robot stops
    - if the position in x direction of the robot is greater than 10m, the robot stops

**Solution**

- Create the proper package:
````bash
cd src
ros2 pkg create --build-type ament_python --license Apache-2.0 ros2_move_turtle --dependencies rclpy turtlesim geometry_msgs
````
- Compile
````bash
cd ..
colcon build
````
- Create the new "/move_turtle" node: move_turtle.py
- 
