#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"
#include "vector"
#include <math.h>

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
  uchar intensity;
  int idx, threshold,i,j,k,l,sum_i,sum_xi,sum_yi,*count;  
  double min_obstacle_rad,max_obstacle_rad,inflation,theta,r,x_c,y_c,r_c;
  vector<cv::Vec3f> circles;
  cv::Mat mask=cv::Mat::zeros(1121,2241,CV_8UC1),acc;
/**
 * ROS initialisations
 */
  ros::init(argc, argv, "obstacle_removal");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan", 1000, con_fusion);
/**
 *  Accepting all the Parameters to be used by this node
 *  DEBUG PROBLEM: Parameters are not being input properly
 */
  if(ros::param::get("min_obstacle_rad",min_obstacle_rad))
  {
	ROS_ERROR("Please specify the minimum threshold for obstacle radius!\nExiting....");
	return -1;	
  }
  if(ros::param::get("max_obstacle_rad",max_obstacle_rad))
  {
	ROS_ERROR("Please specify the maximum threshold for obstacle radius!\nExiting....");
	return -1;
  }
  if(ros::param::get("scan_threshold",threshold))
  {	
	ROS_WARN("No minimum scan points threshold for obstacle detection specified.\nUsing default value!");
	threshold=100;
  }
  if(ros::param::get("inflation_factor",inflation))
  {
	ROS_WARN("Inflation factor for obstacles not defined.\nUsing default value!");
	inflation=1.0;	
  }
/**
 * Error Handling!
 */
  //Omit after fixing parameters input
  min_obstacle_rad=0.6;
  max_obstacle_rad=0.9;
  //
  if(min_obstacle_rad>=max_obstacle_rad)
  {
	ROS_ERROR("Minimum Obstacle Radius(%g) cannot be greater than Maximum Obstacle Radius(%g).\nExiting....",min_obstacle_rad,max_obstacle_rad);
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
	acc=cv::Mat::zeros(1121,2241,CV_8UC1);
	for(theta=-1.570796327;theta<=1.570796327;theta+=scan.angle_increment)
	{
		idx=cvRound((theta-scan.angle_min)/scan.angle_increment);
		if(scan.ranges[idx]>25.0)
		  continue;
		r=scan.ranges[idx];
		cv::Point center(cvRound(1120.0+(r*sin(theta)*1120.0/25.0)),cvRound(1120.0-(r*cos(theta)*1120.0/25.0)));
		cv::circle(mask,center,cvRound(min_obstacle_rad*1120.0/25.0),cv::Scalar(1),cvRound((max_obstacle_rad-min_obstacle_rad)*1120.0/25.0),8,0);
		acc=acc+mask;
		mask=cv::Mat::zeros(1121,2241,CV_8UC1);
	}
/**
 * DEBUG TEST 1: Displaying the Accumulator Matrix as an Image and comparing it with Scan Data by running on a .bag file
 */
 	cv::imshow("Obstacle Detection DEBUG 1",acc);
 	cv::waitKey(1);
/**
 * DEBUG TEST 1 RESULT: Expected Output obtained after certain modifications
 *              STATUS: Unable to obtain parameters
 */	
	for(i=0;i<1121;i++)
	  for(j=0;j<2241;j++)
	  {
		if(acc.at<uchar>(i,j)>=threshold)
		{
			sum_i=sum_xi=sum_yi=0;
			for(k=-2;k<=2;k++)
			  for(l=-2;l<=2;l++)
			  {
				intensity=acc.at<uchar>(i+k,j+l);
				acc.at<uchar>(i+k,j+l)=0;
				sum_i+=intensity;
				sum_xi+=(j+l)*intensity;
				sum_yi+=(i+k)*intensity;
			  }
			x_c=double(sum_xi)/double(sum_i);
			y_c=double(sum_yi)/double(sum_i);
			circles.push_back(cv::Vec3f((x_c-1120.0)*25.0/1120.0,(1120.0-y_c)*25.0/1120.0,0));
		}
	  }
/**
 * DEBUG TEST 2: Analysis of the contents of the Vector circles and comparing it with the Accumulator Matrix
 */	

/**
 * DEBUG TEST 2 STATUS: Compiled Successfully
 */
	count=(int*)calloc(circles.size(),sizeof(int));
	for(theta=-1.570796327;theta<=1.570796327;theta+=scan.angle_increment)
	{
		idx=cvRound((theta-scan.angle_min)/scan.angle_increment);
		if(scan.ranges[idx]>25.0)
		  continue;
		r=scan.ranges[idx];
		for(i=0;i<circles.size();i++)
		{
			r_c=sqrt(pow(circles[i][0]-r*sin(theta),2.0)+pow(circles[i][1]-r*cos(theta),2.0));
			if(r_c<=max_obstacle_rad && r_c>=min_obstacle_rad)
			{			
				count[i]++;
				circles[i][2]+=r_c;
			}
		}	
	}
	for(i=0;i<circles.size();i++)
	  circles[i][2]/=count[i];
/**
 * DEBUG TEST 3: Analysis of No. of points giving a particular center and comparing the obtained radius with Accumulator data
 */

/**
 * DEBUG TEST 3 STATUS: Compiled Successfully
 */
	scan_rec=false;
  }
  return 0;
}
