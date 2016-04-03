# Lab 2: Introduction to ROS

## Learning Outcomes
- Understand ROS
- Read Sensor data
- Control motors
- Build a ROS node

## Section 1: Getting Started
### Where to go for Help

Information about ROS: http://wiki.ros.org/

Information about C++: http://www.cplusplus.com/

C++ for C programmers tutorial: http://www.4p8.com/eric.brasseur/cppcen.html

C++ tutorials: http://www.cprogramming.com/tutorial.html

### Accessing the robot

The Teaching Kit robot broadcasts its own WiFi network that you will need to connect
to in order to interact with the robot.  Ask your professor for the wifi network that corresponds to your robot.  Now you can join the network on your computer.

Once you are connected to the network, you can log into the robot.  To do this, open a
 terminal shell and type `ssh ubuntu@[ip]` where you replace [ip] with the robot's IP address.

Now you need to start ROS.  Enter the workspace launch directory: `cd nvidia_robot/rosjetson/launch`.  Start ROS by typing `roslaunch rosjetson.launch`.  Leave this terminal open.

### Inspecting ROS
On your personal computer, open a new terminal window and ssh into the robot.  Let's take a look at what ROS programs are currently running.

Type
`rosnode list` and look at the output.  This is all of the ROS nodes that are currently active.

ROS nodes are processes that communicate by sharing information in topics.  We can view all of the available topics by typing `rostopic list` in the terminal.  To learn more about a specific topic, you can use `rostopic info [topic_name]` where [topic_name] is replaced by the topic.  The rostopic utility also has other
features that can help you debug ros node behavior.  Use `rostopic -h` to find out what options exist and use these commands to answer the following questions:

What is the bandwidth of cv_camera/image_raw?

What is the message type of cmd_vel?

Rostopic also allows us to view the messages on the topic.  Enter `rostopic echo arduino/sonar_1`.  A list of data entries will appear on the screen.  Move your hand or
another item in front of the left sonar sensor.  Notice the change in the sonar readings.
Type Ctrl-C to quit.

### Using the Dashboard

From your personal computer, open a web browser and go to the url `http:[ip]:5000` where ip is the IP address of your robot.  This webpage can be used to control the robot and view the raw camera data as well as user-generated video feeds.  Open the raw camera data by selecting cv_camera/image_raw from the Video Source dropdown.

Try moving the robot around with the arrows on the screen.

## Section 2: ROS Programming Environment

### Required Tools

In this course, we will make the robot perform a variety of tasks by creating rosnodes.
Rosnodes can be built using Python or C++.  The process for building nodes is handled by Catkin (http://wiki.ros.org/catkin/conceptual_overview).  Catkin is a convenient build tool that handles dependency management and mult-project or multi-node builds.
