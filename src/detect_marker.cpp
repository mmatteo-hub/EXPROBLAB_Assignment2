#include <ros/ros.h>
#include <iostream>
#include <aruco/aruco.h>
#include <opencv2/highgui/highgui.hpp>

int main(int argc,char **argv)
{
	if (argc != 2 )
	{
		std::cerr<<"Usage: inimage"<<std::endl;
		return -1;
	}

	cv::Mat image=cv::imread(argv[1]);
	aruco::MarkerDetector MDetector;

	//detect
	std::vector<aruco::Marker> markers=MDetector.detect(image); //print info to console

	std::cout << "The id detected is: ";
	for(size_t i=0;i<markers.size();i++)
	{
		std::cout<<markers[i]<<std::endl; //draw in the image
		markers[i].draw(image);
		std::cout << markers.at(i).id << " ";
	}
	cv::imshow("image",image);
	cv::waitKey(0);
}