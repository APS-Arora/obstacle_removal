#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"

#define Pixel(M,x,y,c) *(M.data+M.step[0]*x+M.step[1]*y+c)
/**
 * Global Declarations
 */
sensor_msgs::LaserScan scan;
bool scan_rec=false;

using namespace std;

void con_fusion(const sensor_msgs::LaserScan::ConstPtr& rec_scan)
{
  scan=*rec_scan;
  scan_rec=true;
}

int main(int argc, char **argv)
{
/**
 * Declaration of local Variables
 */
  int idx, threshold;  
  double min_obstacle_rad,max_obstacle_rad,inflation,theta,r;
  cv::Mat mask=cv::Mat::zeros(2241,2241,CV_8UC1),acc=cv::Mat::zeros(2241,2241,CV_8UC1);
/**
 * ROS initialisations
 */
  ros::init(argc, argv, "obstacle_removal");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan", 1000, con_fusion);
/**
 *  Accepting all the Parameters to be used by this node
 */
  if(n.getParam("min_obstacle_rad",min_obstacle_rad))
  {
	ROS_ERROR("Please specify the minimum threshold for obstacle radius!\nExiting....");
	return -1;	
  }
  if(n.getParam("max_obstacle_rad",max_obstacle_rad))
  {
	ROS_ERROR("Please specify the maximum threshold for obstacle radius!\nExiting....");
	return -1;
  }
  if(n.getParam("scan_threshold",threshold))
  {	
	ROS_WARN("No minimum scan points threshold for obstacle detection specified.\nUsing default value!");
	threshold=100;
  }
  if(n.getParam("inflation_factor",inflation))
  {
	ROS_WARN("Inflation factor for obstacles not defined.\nUsing default value!");
	inflation=1;	
  }
/**
 * Error Handling!
 */
  if(min_obstacle_rad>=max_obstacle_rad)
  {
	ROS_ERROR("Minimum Obstacle Radius cannot be greater than Maximum Obstacle Radius.\nExiting....");
	return -1;
  }
/**
 * Code begins here!
 */
  cv::namedWindow("Obstacle Detection DEBUG 1",CV_WINDOW_NORMAL);
  while(ros::ok())
  {
	if(!scan_rec)
	{
		ros::spinOnce();
		continue;
	}
	for(theta=scan.angle_min;theta<=scan.angle_max;theta+=scan.angle_increment)
	{
	idx=cvRound((theta-scan.angle_min)/scan.angle_increment);
	r=scan.ranges[idx];
	cv::Point center(cvRound(1120.0+(r*sin(theta)*1120.0/56.0)),cvRound(1120.0-(r*cos(theta)*1120.0/56.0)));
	circle(mask,center,cvRound(min_obstacle_rad*1120.0/56.0),cv::Scalar(1),cvRound((max_obstacle_rad-min_obstacle_rad)*1120.0/56.0),8,0);
	acc=acc+mask;
	//cv::imshow("Obstacle Detection DEBUG 1",mask);
	mask=cv::Mat::zeros(2241,2241,CV_8UC1);
	}
/**
 * DEBUG TEST 1: Displaying the Accumulator Matrix as an Image and comparing it with Scan Data by running on a .bag file
 */
        cout<<idx<<endl;
	cv::imshow("Obstacle Detection DEBUG 1",acc);
	cv::waitKey(100);
/**
 * DEBUG TEST 1 STATUS: Output not as expected (Failure)
 *             PROBLEM: Accumulator Matrix shows a zero matrix
 */	
	scan_rec=false;
  }
  return 0;
}
