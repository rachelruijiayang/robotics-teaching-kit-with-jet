# Lab 5: Object Tracking

## Learning Outcomes
- use Image Moments for Tracking
- use HAAR Cascade Classifiers for Face Detection

## Section 1: Requirements and Design

In this lab, you will build a rosnode that detects and tracks objects and human faces.

## Section 2: Installing the Lab

To perform this lab, you will need to get the moment_tracker and face_tracker templates into your catkin workspace.  First ensure that your Jetbot has internet access by connecting it using WiFi or ethernet.  Next ssh into the Jetbot and enter the following command:

```
wget http://instructor-url/lab5_object_tracking/code.zip
```

Where the url should be replaced by the URL provided by your instructor.  Now unzip the lab:
```
unzip code.zip
```

Move the resulting folder into your catkin workspace:
```
mv moment_tracker ~/catkin_ws/src/ -r
mv face_tracker~/catkin_ws/src/ -r
```

Delete the zip file:
```
rm code.zip
```

To build the code, use the following command when you are in `~/catkin_ws/`:
```
catkin_make && source devel/setup.sh
```

To run the nodes, first launch the Jetbot platform (see lab2 for instructions), then use the commands below:
```
rosrun face_tracker face_tracker
rosrun moment_tracker moment_tracker
```
## Section 3: Object Tracking

## Section 4: Face Tracking

## Section 5: Optical Flow
