/**
 * \file detect_marker.cpp
 * \brief Node responsible of moving the arm of the robot to detect the ArUco markers in the environment the robot is deployes into.
 * \author Matteo Maragliano
 * \version 1.0
 * \date 05/12/2023
 * 
 * \details
 * 
 * * Subscriber to: <BR>
 * /camera/rgb/image_raw Topic to retrieve the robot vision through the camera mounted on the 5-th link of the arm
 * 
 * Publisher to: <BR>
 * /arm/yb_arm_joint_1_position_controller/command Topic to pub the joint 1 angle for the robot arm
 * /arm/yb_arm_joint_2_position_controller/command Topic to pub the joint 2 angle for the robot arm
 * /arm/yb_arm_joint_3_position_controller/command Topic to pub the joint 3 angle for the robot arm
 * /arm/yb_arm_joint_4_position_controller/command Topic to pub the joint 4 angle for the robot arm
 * /arm/yb_arm_joint_5_position_controller/command Topic to pub the joint 5 angle for the robot arm
 * 
 * /list_topic Topic in which it is published the list of all the markers detected by the robot
 * 
 * Description:
 * The node is used to detect the ArUco markers that are in the environment. In particular the robot is deployed in the E room and it has to remain still and detect
 * all the 7 markers around him. In order to do so, it is provided an arm with a cam to have a better field of view.
 * The arm is moved by publishing the angle of the respective 5 joints in order to cover 360 degrees and to detect all the markers correctly.
 * Once the robot has finished its detection phase it publishes through the dedicated topic the list of all the markers such that the FSM running on the robot can 
 * process the information and starting its patrolling behaviour
**/

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
/**
 * \brief Contructor of the MarkerDetectClass
 * \param nodehandle
 * 
 * \return
 * 
 * This is the constrctor of the MarkerDetectClass. It simpli initializes publishers and subscribers that will be used in the code
**/
  ROS_INFO("in class constructor of MarkerDetectClass");
  initializePublishers();
  initializeSubscribers();

  counter = 0;
}

void MarkerDetectClass::initializePublishers()
{
/**
 * \brief Method to initialize the publishers
 * \param
 * 
 * \return
 * 
 * This method initializes the publishers that will be used in the code.
 * It initializes both the ones related to the joint arm and the one related to the marker list
**/
  joint_1_pose_pub = nh_.advertise<std_msgs::Float64>("/arm/yb_arm_joint_1_position_controller/command",1000);
  joint_2_pose_pub = nh_.advertise<std_msgs::Float64>("/arm/yb_arm_joint_2_position_controller/command",1000);
  joint_3_pose_pub = nh_.advertise<std_msgs::Float64>("/arm/yb_arm_joint_3_position_controller/command",1000);
  joint_4_pose_pub = nh_.advertise<std_msgs::Float64>("/arm/yb_arm_joint_4_position_controller/command",1000);
  joint_5_pose_pub = nh_.advertise<std_msgs::Float64>("/arm/yb_arm_joint_5_position_controller/command",1000);

  list_pub = nh_.advertise<std_msgs::Int32MultiArray>("/list_topic", 1000);
}

void MarkerDetectClass::initializeSubscribers()
{
/**
 * \brief Method to initialize the subscriber
 * \param
 * 
 * \return
 * 
 * This method initializes the subscriber that will be used in the code.
 * It initializes the one that is used to retrieve information from the camera
**/
  cam_sub = nh_.subscribe("/camera/rgb/image_raw", 1000, &MarkerDetectClass::cb_cam, this);
}

void MarkerDetectClass::cb_cam(const sensor_msgs::Image::ConstPtr& msg)
{
/**
 * \brief Callback for the camera
 * \param
 * 
 * \return
 * 
 * This method is the callback of the camera, used by the respective subscriber.
 * It reads the information from the camera and thanks to the ArUco libraries processes it and detect the respective ArUco marker.
 * It prints on the terminal the detected marker ID.
**/
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
/**
 * \brief Method to find the marker
 * \param
 * 
 * \return
 * 
 * This method is used to move the robot arm and to detect the markers.
 * In pariculare the robot arm is move looking at the ground from 0, the initial position it starts into, to 3.14, then on the other side so to -3.14.
 * In this way it looks to all markers placed on the ground.
 * With the same methodology it is performed the same action but looking upward, thus detecting the markers placed on the walls.
 * Once all the actions for detecting are performed the robot is put back into its "home" position, all angles to 0, and the resulting list is published on the topic.
 * To manage the timing, it is used a counter starting from 0 that is periodically incremented to allow the robot moving as long as needed to correctly monitor all the 
 * area.
 * 
 * All the angles are set in radians.
**/
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
/**
 * \brief Main
 * \param
 * 
 * \return
 * 
 * This is the main method. It is responsible of making the routing working correctly.
 * In order to monitor the duration of the arm movement the rate loop is set to 0.5 [Hz], so the loop is executed every 2 seconds.
 * 
 * Once the arm finishes its monitoring action the list is put into an array and then published.
**/
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