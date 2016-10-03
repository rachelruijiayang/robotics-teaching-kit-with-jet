#include <math.h>
#include <vector>
#include <utility>
#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

#define CENTIMETERS_PER_METER 100.0

using namespace std;

class Mapping
{
public:
  Mapping();

private:
  void loop();
  void sonarLeftCallback(const std_msgs::Int16::ConstPtr& msg);
  void sonarCenterCallback(const std_msgs::Int16::ConstPtr& msg);
  void sonarRightCallback(const std_msgs::Int16::ConstPtr& msg);

  ros::Publisher point_cloud_pub;

  ros::Subscriber sonar_left_sub, sonar_center_sub, sonar_right_sub;

  int sonarLeftVal, sonarCenterVal, sonarRightVal;

  sensor_msgs::PointCloud point_msg;

  ros::NodeHandle nh;
};

Mapping::Mapping() {
  //Publisher for the point cloud
  point_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/point_cloud", 10);

  //Subscribers for sonar
  sonar_left_sub = nh.subscribe("/arduino/sonar_3", 10, &Mapping::sonarLeftCallback, this);
  sonar_center_sub = nh.subscribe("/arduino/sonar_2", 10, &Mapping::sonarCenterCallback, this);
  sonar_right_sub = nh.subscribe("/arduino/sonar_1", 10, &Mapping::sonarRightCallback, this);

  loop();
}

void Mapping::sonarLeftCallback(const std_msgs::Int16::ConstPtr& msg) {
  sonarLeftVal = msg->data;
}

void Mapping::sonarRightCallback(const std_msgs::Int16::ConstPtr& msg) {
  sonarRightVal = msg->data;
}

void Mapping::sonarCenterCallback(const std_msgs::Int16::ConstPtr& msg) {
  sonarCenterVal = msg->data;
}


void Mapping::loop() {
    ros::Rate r(1.0);
    int ind = 0;
    while(nh.ok()){
      ros::spinOnce();
      geometry_msgs::Point32 leftPoint, centerPoint, rightPoint;

      vector<geometry_msgs::Point32> cur_points;

      if (sonarLeftVal > 0) {
        double left_len = pow(sonarLeftVal / 2, 2) / CENTIMETERS_PER_METER;
        leftPoint.x = left_len;
        leftPoint.y = left_len;
        cur_points.push_back(leftPoint);
      }

      if (sonarRightVal > 0) {
        double right_len = pow(sonarRightVal / 2, 2) / CENTIMETERS_PER_METER;
        rightPoint.x = right_len;
        rightPoint.y = -right_len;
        cur_points.push_back(rightPoint);
      }

      if (sonarCenterVal > 0) {
        double center_len = sonarCenterVal / CENTIMETERS_PER_METER;
        centerPoint.x = center_len;
        centerPoint.y = 0;
        cur_points.push_back(leftPoint);
      }

      point_msg.header.frame_id = "base_link";

      point_msg.points.resize(cur_points.size());
      for (int i = 0; i < cur_points.size(); i++) {
        point_msg.points[i] = cur_points[i];
      }

      point_cloud_pub.publish(point_msg);

      r.sleep();
    }
}

int main(int argc, char * argv[]) {
  ros::init(argc, argv, "mapping");

  Mapping Mapping;
}
