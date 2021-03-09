#include <ros/ros.h>
#include "azure_sensor_angle/AngleCalculator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_angle_calcualtor");
  ros::NodeHandle nodeHandle("~");
  ros_sensor_angle::RosAngleCalculator RosAngleCalculator(nodeHandle);
  
  ros::spin();
  return 0;
}
