#include "azure_sensor_angle/AngleCalculator.hpp"

// STD
#include <string>

namespace ros_sensor_angle {

RosAngleCalculator::RosAngleCalculator(ros::NodeHandle& nodeHandle)
 : nodeHandle_(nodeHandle), tfListener_(tfBuffer_)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  pos_sub_ = nodeHandle_.subscribe(posSubTopic_, 100,
                                      &RosAngleCalculator::posCallback, this);
                                      
  scan_sub_ = nodeHandle_.subscribe(scanSubTopic_, 100,
                                      &RosAngleCalculator::scanCallback, this);

  feedback_sub_ = nodeHandle_.subscribe(feedbackSubTopic_, 100,
                                    &RosAngleCalculator::feedbackCallback, this);
  // serviceServer_ = nodeHandle_.advertiseService("get_average",
  //                                               &RosAngleCalculator::serviceCallback, this);

  angle_pub_ = nodeHandle_.advertise<std_msgs::Float64>("/camera_angle", 100, true);
  // score_pub_ = nodeHandle_.adviserte<std_msgs::Float64>("/angle_score", 100, true);
  feedback_pub_ = nodeHandle_.advertise<sensor_msgs::JointState>("/feedback_angle", 100, true);
  // timer1_ = nodeHandle_.createTimer(ros::Duration(0.02),&RosAngleCalculator::timer1Callback,this);  

  ROS_INFO("Successfully launched azure sensor angle node.");
}

RosAngleCalculator::~RosAngleCalculator()
{
}

bool RosAngleCalculator::readParameters()
{
  if (!nodeHandle_.getParam("pos_subscriber_topic", posSubTopic_)) return false;
  if (!nodeHandle_.getParam("scan_subscriber_topic", scanSubTopic_)) return false;
  if (!nodeHandle_.getParam("feedback_subscriber_topic", feedbackSubTopic_)) return false;
  return true;
}

void RosAngleCalculator::posCallback(const visualization_msgs::MarkerArray& message)
{
  body_msgs_ = message;
  std_msgs::Float64 msg;
  msg.data = calculateAngle(body_msgs_, laser_msgs_);
  angle_pub_.publish(msg);
}

void RosAngleCalculator::scanCallback(const sensor_msgs::LaserScan& message)
{
   laser_msgs_ = message;
}

void RosAngleCalculator::feedbackCallback(const geometry_msgs::Vector3Stamped& message)
{
  static long seq=0;static double yaw_offset = 0;
  if(seq > 10)
  {
    sensor_msgs::JointState msg;
    msg.header.seq = seq;
    msg.header.stamp = ros::Time::now();
    msg.name.push_back("robot_base_to_camera");
    msg.position.push_back(message.vector.z - yaw_offset);
    feedback_pub_.publish(msg);
  }
  else
  {
    yaw_offset = message.vector.z; 
  }
  seq++;
}

void RosAngleCalculator::timer1Callback(const ros::TimerEvent& e)
{
  std_msgs::Float64 msg; double score;
  score = calculateScore(nav_msgs_, laser_msgs_);
  msg.data = score;
  score_pub_.publish(msg);
}

bool RosAngleCalculator::serviceCallback(std_srvs::Trigger::Request& request,
                                         std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "The average is " + std::to_string(algorithm_.getAverage());
  return true;
}

double RosAngleCalculator::calculateAngle(const visualization_msgs::MarkerArray& body_message, const sensor_msgs::LaserScan& laser_message,float fov_angle) 
{
  geometry_msgs::PoseStamped pose_in;
  geometry_msgs::PoseStamped pose_out;
  double x=0, y=0, angle;
  static uint8_t pre_id = 0;
  static double target_angle = 0;
  bool target_flag = false;

  for(auto iter:body_message.markers)
  {
    if(iter.id/100 == pre_id && iter.id%100 == 0)
    {
      pose_in.header = iter.header;
      pose_in.pose  = iter.pose;
      target_flag = true;
      break;
    }
  }

  if(!target_flag)
  {
    for(auto iter:body_message.markers)
    {
      if(iter.id%100 == 0)
      {
        pose_in.header = iter.header;
        pose_in.pose  = iter.pose;
        pre_id = iter.id/100;
        target_flag = true;
        break;
      }
    }
  }
  
  if(target_flag)
  {
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer_.lookupTransform("base_link", pose_in.header.frame_id, ros::Time(0),ros::Duration(1.0));
      tf2::doTransform(pose_in,pose_out,transformStamped);
      x = pose_out.pose.position.x;
      y = pose_out.pose.position.y;
    } 
    catch (tf2::TransformException &ex) {
      ROS_WARN("Could NOT transform body data to base_link: %s", ex.what());
    }

    // try
    // {
    //   tfBuffer_.transform(pose_in,pose_out,"base_link",ros::Duration(1.0));
    //   x = pose_out.pose.position.x;
    //   y = pose_out.pose.position.y;
    //   // x = pose_in.pose.position.x;
    //   // y = pose_in.pose.position.y;
    //   // ROS_WARN_STREAM("Target pose is:"<<x<<','<<y<<';'<<pose_in.pose.position.x<<','<<pose_in.pose.position.y);
      if(sqrt(x*x+y*y) > 1) target_angle = atan2(y,x);
      else target_angle = 0;
    // }
    // catch (tf2::TransformException &ex) 
    // {
    //   ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    // }

  }


  //Fixed Policy
  // target_angle=0;
  // angle = target_angle;
  
  //Visual Servo Policy
  angle = target_angle;

  // 3 Stage Policy
  // if(abs(target_angle)<fov_angle/2) 
  //   angle = target_angle;
  // else if (abs(target_angle) < fov_angle)
  // {
  //   angle = fov_angle/2 * copysign(1.0,target_angle);
  // }
  //   else
  // {
  //   angle = (abs(target_angle) - fov_angle/2) * copysign(1.0,target_angle);
  // }

  // Half Policy 
  // if (abs(target_angle) < fov_angle)
  // {
  //   angle = target_angle/2;
  // }
  // else
  // {
  //   angle = (abs(target_angle) - fov_angle/2) * copysign(1.0,target_angle);
  // }

  // Test Policy 
  // float n = 2;
  // if (abs(target_angle) < fov_angle*n/(2*(n-1)))
  // {
  //   angle = target_angle/n;
  // }
  // else
  // {
  //   angle = (abs(target_angle) - fov_angle/2) * copysign(1.0,target_angle);
  // }

  return angle;

}

double RosAngleCalculator::calculateScore(const nav_msgs::Path& nav_message, const sensor_msgs::LaserScan& laser_message,float fov_min, float fov_max, float fov_angle, float laser_min, float laser_incre) //laserscan paramater(rad)
{
  int len=0, count=0,start=0; double x, y, angle, score, target_score, path_score;
  geometry_msgs::PoseStamped pose_out;
  start = nav_message.poses.size()>200 ? nav_message.poses.size()-200 : 0; // remove unnessary nav goals
  // ROS_INFO_STREAM("Start pos is:"<<start);
  for(uint i=start;i<nav_message.poses.size();++i){
    try
    {
      tfBuffer_.transform(nav_message.poses[i],pose_out,"camera_link",ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("Transform Failure %s\n", ex.what()); //Print exception which was caught
      break;
    }
    x = pose_out.pose.position.x;
    y = pose_out.pose.position.y;
    angle = atan2(y,x);
    if(x>fov_min && x<fov_max && abs(angle) < fov_angle/2) //consider whether path point is in 
    {
      float point_distance,scan_distance; uint index;
      index = round((angle - laser_min)/laser_incre);
      if(index < 0) //prevent out_of_range error
        index = 0;
      else if(index >= laser_message.ranges.size()) 
        index = laser_message.ranges.size()-1;
      scan_distance = laser_message.ranges[index];
      if (!isinf(scan_distance))
      {
        point_distance = sqrt(x*x+y*y);
        if(point_distance < scan_distance) count+=1;
      }
      else
        count +=1;
    } 
    if(x>fov_min) len++;  //only count path > fov_min
  }

  if(sqrt(x*x+y*y)>fov_min)
  {
    if(len==0) len+=1;
    path_score = (double)count/len;
    
    // if(x>fov_min && x<fov_max && abs(angle) < fov_angle/2) target_score = 1;  //extra score for successful target tracking
    // else target_score = 0; 
    target_score =  - angle*angle/(fov_angle*fov_angle)  + 1; // score for successful target tracking, 
    target_score = (target_score > 0.75) ? target_score : 0;
    if( target_score < 0.75) 
    {
      target_score = 0;
    }
    score = target_score + path_score;
  }

  else
    score = 2;
  // ROS_INFO_STREAM("Score is:"<<score);
  return score;
}



} /* namespace */