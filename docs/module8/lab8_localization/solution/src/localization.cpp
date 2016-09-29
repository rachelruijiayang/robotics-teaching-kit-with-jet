#include "Localization.h"

void Localization::loop() {
    bool stopped = false;
    go_follow_pub.publish(std_msgs::Empty());
    ros::Rate r(10.0);
    while(nh.ok()){
      ros::spinOnce();

      stopped = analyzeState();
      if (!stopped) {
        moveUpdate();
        sensorUpdate();
        resample();
      }
      else {
        stop_follow_pub.publish(std_msgs::Empty());
      }

      r.sleep();
    }
}

void Localization::moveUpdate() {
  double encoderDelta = encoderVal - prevEncoderVal;
  double mapDelta = encoderDelta / courseLength * MAP_SIZE;
  for (int i = 0; i < particles.size(); i++) {
    particles[i] += mapDelta + move_error_distribution(generator);
    if (particles[i] > MAP_SIZE) {
      particles[i] -= MAP_SIZE;
    }
  }
  prevEncoderVal = encoderVal;
}

void Localization::sensorUpdate() {
  double weightsum = 0;
  bool obj_detected = sonarVal > 0 && sonarVal < 20;
  for (int i = 0; i < particles.size(); i++) {
    int closest_value = round(particles[i]);
    if (closest_value == MAP_SIZE) {
      closest_value = 0;
    }
    if ((obj_detected && obstacle_map[closest_value]) ||
        (!obj_detected && !obstacle_map[closest_value])) {
      weights[i] = hitWeightRatio;
    }
    else {
      weights[i] = 1.0;
    }
    weightsum += weights[i];
  }
  for(int i = 0; i < particles.size(); i++) {
    weights[i] /= weightsum;
  }
}

void Localization::resample() {
  vector<double> nextparticles;
  for (int i = 0; i < particles.size(); i++) {
    double rand_val = static_cast <double> (rand()) / (static_cast <float> (RAND_MAX));
    int j = 0;
    while (rand_val > 0) {
      rand_val -= weights[++j];
    }
    nextparticles.push_back(particles[j]);
  }
  particles = nextparticles;
}

bool Localization::analyzeState() {
  tuple<double, double> stats = getCircularStats(particles, MAP_SIZE);
  double mean = get<0>(stats);
  double stddev = get<1>(stats);

  pose_msg.pose.position.x = mean;
  pose_msg.covariance[0] = stddev;
  pose_pub.publish(pose_msg);

  return stddev < stoppingThreshold && mean < 0.5;
}


int main(int argc, char * argv[]) {
  ros::init(argc, argv, "localization");

  Localization localization;
}
