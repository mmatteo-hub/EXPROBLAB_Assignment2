#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include "std_msgs/Float64.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include <aruco/aruco.h>
#include "control_msgs/JointControllerState.h"
#include "joint_state_controller/joint_state_controller.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <list>
#include <algorithm>
#include "detect_marker.h"

// poses store
std_msgs::Float64 msg;

// declaring list for marker storing
std::list<int> marker_list;

aruco::MarkerDetector mDetector_;
std::vector<aruco::Marker> markers_;
aruco::CameraParameters camParam_;

cv_bridge::CvImagePtr cv_ptr;
cv::Mat inImage_;
double marker_size_ = 0.05;

MarkerDetectClass::MarkerDetectClass(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
  ROS_INFO("in class constructor of MarkerDetectClass");
  initializePublishers();
  initializeSubscribers();

  counter = 0;
}

void MarkerDetectClass::initializePublishers()
{
  joint_1_pose_pub = nh_.advertise<std_msgs::Float64>("/arm/yb_arm_joint_1_position_controller/command",1000);
  joint_2_pose_pub = nh_.advertise<std_msgs::Float64>("/arm/yb_arm_joint_2_position_controller/command",1000);
  joint_3_pose_pub = nh_.advertise<std_msgs::Float64>("/arm/yb_arm_joint_3_position_controller/command",1000);
  joint_4_pose_pub = nh_.advertise<std_msgs::Float64>("/arm/yb_arm_joint_4_position_controller/command",1000);
  joint_5_pose_pub = nh_.advertise<std_msgs::Float64>("/arm/yb_arm_joint_5_position_controller/command",1000);

  list_pub = nh_.advertise<std_msgs::Int32MultiArray>("/list_topic", 1000);
}

void MarkerDetectClass::initializeSubscribers()
{
  cam_sub = nh_.subscribe("/camera/rgb/image_raw", 1000, &MarkerDetectClass::cb_cam, this);
}

void MarkerDetectClass::cb_cam(const sensor_msgs::Image::ConstPtr& msg)
{
  aruco::MarkerDetector mDetector_;
  std::vector<aruco::Marker> markers_;
  camParam_ = aruco::CameraParameters();

  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  inImage_ = cv_ptr->image;

  markers_.clear();

  mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);

  std::cout << "ID detected: ";
  for (std::size_t i = 0; i < markers_.size(); ++i)
  {
    int _id = markers_.at(i).id;
    std::cout << _id << " ";
    if(!(std::find(marker_list.begin(), marker_list.end(), _id) != marker_list.end()))
      marker_list.push_back(_id);
  }
  std::cout << std::endl;
}

void MarkerDetectClass::_marker_find()
{
  if(counter != -1)
  {
    if(counter < 15)
    {
      msg.data = 3.14;
      joint_1_pose_pub.publish(msg);

      msg.data = 0.9;
      joint_4_pose_pub.publish(msg);
      counter++;
    }

    else if(counter >= 15 and counter < 40)
    {
      msg.data = -3.14;
      joint_1_pose_pub.publish(msg);
      counter++;
    }

    else if(counter >= 40 and counter < 65)
    {
      msg.data = 3.14;
      joint_1_pose_pub.publish(msg);

      msg.data = 0.0;
      joint_4_pose_pub.publish(msg);
      counter++;
    }
    else
    {
      msg.data = 0.0;
      joint_1_pose_pub.publish(msg);

      msg.data = 0.0;
      joint_2_pose_pub.publish(msg);

      msg.data = 0.0;
      joint_3_pose_pub.publish(msg);

      counter = -1;    
    } 
  }
}

int main(int argc,char **argv)
{
  ros::init(argc, argv, "marker_detector");

  ros::NodeHandle nh;

  ROS_INFO("main: instantiating an object of type MarkerDetectClass");
  MarkerDetectClass MarkerDetectClass(&nh);

  ros::Rate loop_rate(0.5);

  while(ros::ok())
  {
    if(MarkerDetectClass.counter != -1)
      MarkerDetectClass._marker_find();
    else
    {
      std_msgs::Int32MultiArray _listArr;
      int k = 0;
      for (int const &i: marker_list)
        _listArr.data.push_back(i);

      // publish the list
      MarkerDetectClass.list_pub.publish(_listArr);
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}