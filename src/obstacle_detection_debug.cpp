#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"
#include "vector"
#include <math.h>
#include "string"
#include "deque"
#include "iostream"
#include "fstream"

#define Pixel(M,x,y,c) *(M.data+M.step[0]*x+M.step[1]*y+c)
/**
 * Global Declarations
 */
sensor_msgs::LaserScan scan;
bool scan_rec=false;
cv::Mat inputImg;

using namespace std;

void con_fusion(const sensor_msgs::LaserScan::ConstPtr& rec_scan)
{
  scan=*rec_scan;
  scan_rec=true;
}

void getImg(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        cv_bridge::CvImagePtr bridge;
        bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        inputImg = bridge->image;
        cv::waitKey(10);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Error in converting image");
    }
}

int main(int argc, char **argv)
{
/**
 * Declaration of local Variables
 */
  uchar intensity;
  int idx, threshold,i,j,k,l,sum_i,sum_xi,sum_yi,*count,nscans;  
  double min_obstacle_rad,max_obstacle_rad,inflation,theta,r,x_c,y_c,r_c,range;
  vector<cv::Vec3f> circles;
  deque<cv::Mat> accs;
  cv::Mat mask=cv::Mat::zeros(1121,2241,CV_16SC1),acc;
  cv::Point center;
  string node_name;
/**
 * ROS initialisations
 */
  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan", 1000, con_fusion);
  node_name=ros::this_node::getName();
  image_transport::ImageTransport image_transport(node);
  image_subscriber = image_transport.subscribe("/sensors/camera/1", 2, getImg);
/**
 *  Accepting all the Parameters to be used by this node
 */
  if(!ros::param::get(node_name+"/min_obstacle_rad",min_obstacle_rad))
  {
	ROS_ERROR("Please specify the minimum threshold for obstacle radius!\nExiting....");
	return -1;	
  }
  if(!ros::param::get(node_name+"/max_obstacle_rad",max_obstacle_rad))
  {
	ROS_ERROR("Please specify the maximum threshold for obstacle radius!\nExiting....");
	return -1;
  }
  if(!ros::param::get(node_name+"/scan_threshold",threshold))
  {	
	ROS_WARN("No minimum scan points threshold for obstacle detection specified.\nUsing default value!");
	threshold=500;
  }
  if(!ros::param::get(node_name+"/inflation_factor",inflation))
  {
	ROS_WARN("Inflation factor for obstacles not defined.\nUsing default value!");
	inflation=1.0;	
  }
  if(!ros::param::get(node_name+"/combine_scans",nscans))
  {
	ROS_WARN("No. of scans to combine for obstacle detection not defined.\nUsing default value!");
	nscans=5;	
  }
  if(!ros::param::get(node_name+"/max_detect_range",range))
  {
	ROS_WARN("Maximum range for obstacle detection not defined.\nUsing default value!");
	range=25.0;	
  }
/**
 * Error Handling!
 */
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
	acc=cv::Mat::zeros(1121,2241,CV_16SC1);
	for(theta=-1.570796327;theta<=1.570796327;theta+=scan.angle_increment)
	{
		idx=cvRound((theta-scan.angle_min)/scan.angle_increment);
		if(scan.ranges[idx]>range)
		  continue;
		r=scan.ranges[idx];
		center= cv::Point(cvRound(1120.0+(r*sin(theta)*1120.0/range)),cvRound(1120.0-(r*cos(theta)*1120.0/range)));
		cv::circle(mask,center,cvRound(min_obstacle_rad*1120.0/range),cv::Scalar(1),cvRound((max_obstacle_rad-min_obstacle_rad)*1120.0/range),8,0);
		acc=acc+mask;
		mask=cv::Mat::zeros(1121,2241,CV_16SC1);
	}
	
	if(accs.size()>=nscans)
	  accs.pop_front();
	for(i=0;i<accs.size();i++)
	  acc+=accs[i];
	accs.push_back(acc);
/**
 * DEBUG TEST 1: Displaying the Accumulator Matrix as an Image and comparing it with Scan Data by running on a .bag file
 */
	if(accs.size()>=nscans)
	{
		cv::imshow("Obstacle Detection DEBUG 1",acc);
 		cv::waitKey(0);
	}
/**
 * DEBUG TEST 1 RESULT: Expected Output obtained after certain modifications
 *              STATUS: Unable to obtain parameters
 */
	circles.clear();
	ROS_ERROR("No. of Obstacles detected=%d",circles.size());	
	for(i=0;i<1121;i++)
	  for(j=0;j<2241;j++)
	  {
		if(acc.at<uchar>(i,j)>=threshold)
		{
			sum_i=sum_xi=sum_yi=0;
			for(k=-5;k<=4;k++)
			  for(l=-5;l<=4;l++)
			  {
				intensity=acc.at<uchar>(i+k,j+l);
				acc.at<uchar>(i+k,j+l)=0;
				sum_i+=intensity;
				sum_xi+=(j+l)*intensity;
				sum_yi+=(i+k)*intensity;
			  }
			x_c=double(sum_xi)/double(sum_i);
			y_c=double(sum_yi)/double(sum_i);
			circles.push_back(cv::Vec3f((x_c-1120.0)*range/1120.0,(1120.0-y_c)*range/1120.0,0));
		}
	  }
	ROS_ERROR("No. of Obstacles detected=%d",circles.size());
	count=(int*)calloc(circles.size(),sizeof(int));	
	for(theta=-1.570796327;theta<=1.570796327;theta+=scan.angle_increment)
	{
		idx=cvRound((theta-scan.angle_min)/scan.angle_increment);
		if(scan.ranges[idx]>range)
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
 * DEBUG TEST 3: Analysis of No. of scan points giving a particular center and comparing the obtained radius with Accumulator data
 */
	cv::Mat debug=cv::Mat::zeros(1121,2241,CV_8UC3);
	cv::Point deb_cent;
	for(i=0;i<circles.size();i++)
	{
		deb_cent=cv::Point(cvRound(1120.0+(circles[i][0]*1120.0/range)),cvRound(1120.0-(circles[i][1]*1120.0/range)));
		cv::circle(mask,deb_cent,cvRound(circles[i][2]*1120.0/range),cv::Scalar(255,255,0),20,8,0);
	}
	if(accs.size()>=nscans)
	{
		cv::imshow("Obstacle Detection DEBUG 1",debug);
 		cv::waitKey(10);
	}
/**
 * DEBUG TEST 3 STATUS: Compiled Successfully
 */
	scan_rec=false;
  }
  return 0;
}
