# Lab 8: Line Follower

## Learning Outcomes
- Use HSV Segmentation
- Calculate Center of Mass with Moments
- Implement PID Control
- Tune PID parameters

## Section 1: Requirements and Design

In this lab, you will use PID control to make Jet follow a line.

## Section 2: Installing the Lab

To perform this lab, you will need to get the lab8_line_follower template into your catkin workspace.  First ensure that your Jet has internet access by connecting it using WiFi or ethernet.  Next ssh into Jet and enter the following command:

```
wget http://instructor-url/lab8_line_follower/code.zip
```

Where the url should be replaced by the URL provided by your instructor.  Now unzip the lab:
```
unzip code.zip
```

Move the resulting folder into your catkin workspace:
```
mv lab8_line_follower ~/catkin_ws/src/jetlabs/
```

Delete the zip file:
```
rm code.zip
```

To build the code, use the following command when you are in `~/catkin_ws/`:
```
catkin_make --pkg lab8_line_follower && source devel/setup.sh
```

To run the system, execute the following
```
roslaunch lab8_line_follower lab8.launch
```

## Section 3: Calculating the Error

## Section 4: Implementing PID

## Section 5: Tuning PID Parameters
