#ifndef MARKER_DETECT_CLASS_H_
#define MARKER_DETECT_CLASS_H_

#include <ros/ros.h>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include "sensor_msgs/Image.h"
#include <std_msgs/Bool.h> 
#include <std_msgs/Float64.h>

class MarkerDetectClass
{
public:
    MarkerDetectClass(ros::NodeHandle* nodehandle);
public:
    ros::NodeHandle nh_;
    
    ros::Publisher joint_1_pose_pub;
    ros::Publisher joint_2_pose_pub;
    ros::Publisher joint_3_pose_pub;
    ros::Publisher joint_4_pose_pub;
    ros::Publisher joint_5_pose_pub;

    ros::Publisher list_pub;

    ros::Subscriber cam_sub;
    
    int counter;
    
    void initializePublishers();
    void initializeSubscribers();
    
    void cb_cam(const sensor_msgs::Image::ConstPtr& msg);

    void _marker_find();
    
};
#endif