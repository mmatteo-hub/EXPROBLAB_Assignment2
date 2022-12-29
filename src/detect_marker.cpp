#include <ros/ros.h>
#include <iostream>
#include "std_msgs/Float64.h"
#include <aruco/aruco.h>
#include "control_msgs/JointControllerState.h"
#include "joint_state_controller/joint_state_controller.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

// array with all the poses fort he marker detection
float poses[8][5] = {{0.0,0.0,0.0,0.0,0.0},		// home position
					 {1.7,2.0,-2.0,0.8,0.0},  	// ground-right (16)
					 {-1.7,2.0,-2.0,0.8,0.0},	// ground-left (12)
					 {-3.14,2.0,-2.0,0.0,0.0},	// ground-back (14)
					 {-0.2,1.15,-2.5,1.9,0.0},	// up-front (13)
					 {-1.7,1.15,-2.5,1.9,0.0},	// up-left (11)
					 {1.7,1.15,-2.5,1.9,0.0},	// up-right ()
					 {-2.5,2.8,-2.6,0.55,0.0}};	// back-lateral ()

// starting pose 1 in the array
int actualPose = 1;
float err1=1, err2=1, err3=1, err4=1, err5=1;
// threshold for the errors
float thr = 0.05;
// poses store
std_msgs::Float64 msg;

aruco::MarkerDetector mDetector_;
std::vector<aruco::Marker> markers_;
aruco::CameraParameters camParam_;

void cb_joint1(const control_msgs::JointControllerState::ConstPtr& msg)
{
	if(msg->set_point < poses[actualPose][0] + thr or msg->set_point > poses[actualPose][0] - thr)
		err1 = msg->error;
	else
		err1 = 1;
}

void cb_joint2(const control_msgs::JointControllerState::ConstPtr& msg)
{
	if(msg->set_point < poses[actualPose][1] + thr or msg->set_point > poses[actualPose][1] - thr)
		err2 = msg->error;
	else
		err2 = 1;
}

void cb_joint3(const control_msgs::JointControllerState::ConstPtr& msg)
{
	if(msg->set_point < poses[actualPose][2] + thr or msg->set_point > poses[actualPose][2] - thr)
		err3 = msg->error;
	else
		err3 = 1;
}

void cb_joint4(const control_msgs::JointControllerState::ConstPtr& msg)
{
	if(msg->set_point < poses[actualPose][3] + thr or msg->set_point > poses[actualPose][3] - thr)
		err4 = msg->error;
	else
		err4 = 1;
}

void cb_joint5(const control_msgs::JointControllerState::ConstPtr& msg)
{
	if(msg->set_point < poses[actualPose][4] + thr or msg->set_point > poses[actualPose][4] - thr)
		err5 = msg->error;
	else
		err5 = 1;
}

void cb_res(const sensor_msgs::Image::ConstPtr& msg)
{

  std::cout << "ID: " << msg->header.frame_id << std::endl;
}

void updatePose(float p[5], int i)
{
	for(int a = 0; a < 5; a++)
		p[a] = poses[i][a];
}

cv_bridge::CvImagePtr cv_ptr;
cv::Mat inImage_;
double marker_size_ = 0.05;

void cb_cam(const sensor_msgs::Image::ConstPtr& msg)
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
    std::cout << markers_.at(i).id << " "; 

  std::cout << std::endl;
}

int main(int argc,char **argv)
{
	ros::init(argc, argv, "pub");

	ros::NodeHandle nh;

	ros::Publisher joint_1_pose_pub = nh.advertise<std_msgs::Float64>("/arm/yb_arm_joint_1_position_controller/command",1000);
	ros::Publisher joint_2_pose_pub = nh.advertise<std_msgs::Float64>("/arm/yb_arm_joint_2_position_controller/command",1000);
	ros::Publisher joint_3_pose_pub = nh.advertise<std_msgs::Float64>("/arm/yb_arm_joint_3_position_controller/command",1000);
	ros::Publisher joint_4_pose_pub = nh.advertise<std_msgs::Float64>("/arm/yb_arm_joint_4_position_controller/command",1000);
	ros::Publisher joint_5_pose_pub = nh.advertise<std_msgs::Float64>("/arm/yb_arm_joint_5_position_controller/command",1000);

	ros::Subscriber jont_1_state_sub = nh.subscribe("/arm/yb_arm_joint_1_position_controller/state", 1000, cb_joint1);
	ros::Subscriber jont_2_state_sub = nh.subscribe("/arm/yb_arm_joint_2_position_controller/state", 1000, cb_joint2);
	ros::Subscriber jont_3_state_sub = nh.subscribe("/arm/yb_arm_joint_3_position_controller/state", 1000, cb_joint3);
	ros::Subscriber jont_4_state_sub = nh.subscribe("/arm/yb_arm_joint_4_position_controller/state", 1000, cb_joint4);
	ros::Subscriber jont_5_state_sub = nh.subscribe("/arm/yb_arm_joint_5_position_controller/state", 1000, cb_joint5);

  ros::Subscriber cam_sub = nh.subscribe("/camera/rgb/image_raw", 1000, cb_cam);

	ros::Rate loop_rate(10);

	float p[5];
	updatePose(p, actualPose);

	while(ros::ok())
	{
		//ROS_INFO("Actual pose [%i]", actualPose);

		if(abs(err1) < thr and abs(err2) < thr and abs(err3) < thr and abs(err4) < thr and abs(err5) < thr)
		{
			if(actualPose <= 7)
				actualPose++;
			else
				actualPose = 0;
		}

		updatePose(p, actualPose);

		std_msgs::Float64 msg;

		msg.data = p[0];
		joint_1_pose_pub.publish(msg);

		msg.data = p[1];
		joint_2_pose_pub.publish(msg);

		msg.data = p[2];
		joint_3_pose_pub.publish(msg);

		msg.data = p[3];
		joint_4_pose_pub.publish(msg);

		msg.data = p[4];
		joint_5_pose_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}