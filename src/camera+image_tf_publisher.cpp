#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <pthread.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>
#include "cmath"
#include "iostream"
#include "fstream"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>

#define CAMERA_OFFSET_X 0
#define CAMERA_OFFSET_Y 0
#define CAMERA_OFFSET_Z 0

using namespace std;

cv::Mat inputImg;
int done=3;
tf::Vector3 img_axis[3];
tf::Vector3 camera_offset(CAMERA_OFFSET_X,CAMERA_OFFSET_Y,CAMERA_OFFSET_Z);     //TODO: update after measuring transformation
float roll=0,pitch=0,yaw=0;

void drawCrossHair(cv::Mat& img, cv::Point pos)
{
	cv::circle(img,pos,3,cv::Scalar(0,0,255),-1);
	cv::circle(img,pos,20,cv::Scalar(0,0,0),3);
	cv::line(img,pos+cv::Point(6,0),pos+cv::Point(16,0),cv::Scalar(0,255,0),3);
	cv::line(img,pos+cv::Point(-6,0),pos+cv::Point(-16,0),cv::Scalar(0,255,0),3);
	cv::line(img,pos+cv::Point(0,6),pos+cv::Point(0,16),cv::Scalar(0,255,0),3);
	cv::line(img,pos+cv::Point(0,-6),pos+cv::Point(0,-16),cv::Scalar(0,255,0),3);
}

void* parallelInput(void* noargs)
{
   float x,y,z;
   //FILE* 3Dto2Dpt_data = fopen(ros::package::getPath("obstacle_removal")+"/data/axis_pts.dat", "w");
   ofstream new_pt_data;
   string new_pt_data_path=ros::package::getPath("obstacle_removal")+"/data/new_pt.dat";
   new_pt_data.open(new_pt_data_path.c_str());
   for(;done>0;done--)
   {
	cout<<"Enter the X-Y-Z Co-ordinates w.r.t frame base_link of a point in line with the point shown in the Image: ";
	cin>>x>>y>>z;
	new_pt_data<<x<<'\t'<<y<<'\t'<<z<<endl;
	img_axis[done]= tf::Vector3(x,y,z)-camera_offset;
   }
   new_pt_data.close();
   pthread_exit(0);
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

int main(int argc, char** argv){
  tf::Vector3 rotatedVec;
  ros::init(argc, argv, "camera+image_tf_broadcaster");
  ros::NodeHandle node;
  tf::TransformBroadcaster br;
  tf::Transform transform;
  ros::Rate rate(10.0);
  tf::Quaternion final_rotate;
  image_transport::ImageTransport image_transport(node);
  image_transport::Subscriber image_subscriber = image_transport.subscribe("/sensors/camera/1", 2, getImg);
  cv::namedWindow("Parameters Input");
  pthread_t IOThread;
  int rc = pthread_create(&IOThread, NULL, parallelInput, NULL);
  if (rc)
  {
  	ROS_ERROR("ERROR: Thread Creation Failed \n return code from pthread_create() is %d\n", rc);
	exit(-1);
  }
  while(done)
  {
	switch(done)
	{
	  case 1:
		drawCrossHair(inputImg,cv::Point(inputImg.cols*5/8,inputImg.rows/2));
		break;
	  case 2:
		drawCrossHair(inputImg,cv::Point(inputImg.cols/2,inputImg.rows*5/8));
		break;
	  case 3:
		drawCrossHair(inputImg,cv::Point(inputImg.cols/2,inputImg.rows/2));
		break;
	}
	cv::imshow("Parameters Input",inputImg);
	cv::waitKey(10);
	ros::spinOnce();
  }
  img_axis[1]=img_axis[1]-img_axis[3].dot(img_axis[1]-img_axis[3])*img_axis[3]-img_axis[3];
  img_axis[1].normalize();
  img_axis[2]=img_axis[2]-img_axis[3].dot(img_axis[2]-img_axis[3])*img_axis[3]-img_axis[3];
  img_axis[2].normalize();
  img_axis[3].normalize();
  yaw=atan(img_axis[1].y()/img_axis[1].x());
  img_axis[1].rotate(tf::Vector3(0,0,1),-yaw);
  pitch=-atan(img_axis[1].z()/img_axis[1].x());
  img_axis[2].rotate(tf::Vector3(0,0,1),-yaw);
  img_axis[2].rotate(tf::Vector3(0,1,0),-pitch);
  roll=atan(img_axis[2].z()/img_axis[2].y());
  img_axis[3].rotate(tf::Vector3(0,0,1),-yaw);
  img_axis[3].rotate(tf::Vector3(0,1,0),-pitch);
  roll-=atan(img_axis[3].y()/img_axis[3].z());
  roll/=2;
  final_rotate.setRPY(roll,pitch,yaw);
  while (node.ok()){
    transform.setOrigin(camera_offset);
    transform.setRotation(tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera"));
    transform.setOrigin(tf::Vector3(0,0,0));
    transform.setRotation(final_rotate);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera", "image"));    
    rate.sleep();
  }
  return 0;
};

