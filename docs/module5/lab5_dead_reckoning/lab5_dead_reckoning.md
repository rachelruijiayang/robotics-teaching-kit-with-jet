rostopic pub /dead_reckoning/goal geometry_msgs/Pose '{position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}'

# Lab 5: Dead Reckoning

## Learning Outcomes
- Understand Physics of robot movement
- Learn how ROS represents robot position/orientation
- Implement Dead Reckoning with encoder data
- Perform basic navigation to goal points

## Section 1: Requirements and Design

In this lab, you will use Jet's encoder data to estimate the velocity, position, and orientation of Jet.  Then you will use this information to navigate the robot towards
a goal.

## Section 2: Installing the Lab

To perform this lab, you will need to get the lab5_dead_reckoning template into your catkin workspace.  First ensure that your Jet has internet access by connecting it using WiFi or ethernet.  Next ssh into Jet and enter the following command:

```
wget http://instructor-url/lab5_dead_reckoning/lab5_dead_reckoning-code.zip
```

Where the url should be replaced by the URL provided by your instructor.  Now unzip the lab:
```
unzip lab8_line_follower-code.zip -d ~/catkin_ws/src/jetlabs/lab5_dead_reckoning
```

Delete the zip file:
```
rm lab5_dead_reckoning-code.zip
```

To build the code, use the following command when you are in `~/catkin_ws/`:
```
catkin_make --pkg lab5_dead_reckoning && source devel/setup.sh
```

To run the system, execute the following
```
roslaunch lab5_dead_reckoning lab5.launch
```

## Section 3: Odometry in ROS

Odometry is the relative position, orientation, and velocity of the robot with respect
to an origin.  In our case, the origin is wherever the robot is when you initialize ROS
with `roslaunch`, but we could adapt the origin by specifying an offset at startup.  In this
lab, you will write code that produces the odometry of the robot based on the robot's sensors.

In ROS, there is an Odometry message that contains the following fields:
```
std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

The final component should be familiar to you: `Twist` is the same type of message
as the velocity topic that we use to publish to the motors.  `TwistWithCovariance` includes
a covariance matrix that specifies the uncertainty of the velocity measurements.  The
`PoseWithCovariance` type has a covariance matrix to identify the uncertainty, and a `Pose`
message.  The `Pose` type contains the `position` (in x, y, and z coordinates) as well
as the `orientation` (in x, y, z, w).

You can read the official documentation for the Odometry message here: http://docs.ros.org/kinetic/api/nav_msgs/html/msg/Odometry.html

## Section 4: Calculating Odometry

Open the lab5 source code folder (`catkin_ws/jetlabs/lab5_dead_reckoning/src`).  The first objective is to publish a topic called `myodom`.  The calculation and publishing of the odometry will occur in the `publishOdom` method in the `DeadReckoning` class.  In order to calculate the odometry, you will
need to maintain the state of the robot in several class fields.  You can add these variable declarations to the class definition and initialize them in the constructor.

What state should be maintained?  The current value of the encoders is already maintained for you in `left_count` and `right_count`.  You will need to know the difference in time between updates, so you should store the previous time in a class field.  Additionally, you need to maintain the current x_pos, y_pos, and angle of the robot.

The code at the end of the `publishOdom` method provides a hint as to what variables you should use.  Feel free to modify the odometry message publishing if you devise a different way of tracking the state of the robot.  

## Section 5: Moving to Goals



## Section 6: Challenge

With the current system, only the encoders are used to determine the location of the robot.  Try incorporating the gyroscope or accelerometer to refine the position and orientation estimates.  Hint: the gyroscope can be used to accurately estimate changes in orientation, so you could use encoders for estimating linear velocity and the gyroscope for angular velocity.

Can Jet adapt to changing goals?  Publish a goal for Jet and then quickly send a different goal before Jet is able to reach the first destination.  Modify your code if necessary in order to make sure Jet can smoothly switch between these goals.
