//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv/cxcore.hpp>

//#include <LinearMath/btQuaternion.h> // Needed to convert rotation ...
//#include <LinearMath/btMatrix3x3.h>  // ... quaternion into Euler angles

using namespace std;
namespace enc = sensor_msgs::image_encodings;

//----------------------------------------------------------------------------

ros::Publisher pos_points_pub;

//----------------------------------------------------------------------------

#define		WIDTH_MAX 	8400
#define		WIDTH_MIN	3700
#define 	HEIGHT		7300
#define		FRAME_WIDTH	320
#define		FRAME_HEIGHT	240
//----------------------------------------------------------------------------

static const char WINDOW[] = "Image window";
char *filename = (char*) calloc(256, sizeof(char));
int counter = 0, skip_counter = 0, frame_counter = 0, frameThreshold = 2;

//----------------------------------------------------------------------------

void send_pos(float rx, float ry, float rz, float gx, float gy, float gz)	{
  std_msgs::Float32MultiArray pts;

  pts.data.push_back(rx);
  pts.data.push_back(ry);
  pts.data.push_back(rz);
  
  pts.data.push_back(gx);
  pts.data.push_back(gy);
  pts.data.push_back(gz);
  
  pos_points_pub.publish(pts);
}

//----------------------------------------------------------------------------
// Release Data

void ReleaseData()	{
  //if( camImage )	camImage->image.release();  
}

//----------------------------------------------------------------------------

void transformPoints(cv::Point redPt, cv::Point greenPt, bool redPointFound, bool greenPointFound)	{
  double r_x, r_y, g_x, g_y, width;
  
  if(redPointFound)	{
    //r_y = FRAME_HEIGHT - redPt.y;
    //r_x = redPt.x-(FRAME_WIDTH/2);
    r_x = -5000;
    r_y = -5000;
  }
  else	{
    r_x = -5000;
    r_y = -5000;
  }
  
  if(greenPointFound)	{
    g_x=greenPt.x-(FRAME_WIDTH/2);
    g_y=FRAME_HEIGHT - greenPt.y;    
  }
  else	{
    g_x = -5000;
    g_y = -5000;
  }
  
  if (redPointFound || greenPointFound)
  	{
  		frame_counter ++;
  		if (frame_counter > frameThreshold)
  			send_pos(r_x,r_y,0,g_x,g_y,0);
  	}
  else
  	frame_counter = 0;

  //send_pos(r_x, r_y, 0, g_x, g_y, 0);
  cout <<"send pos..."<<endl;
}

//----------------------------------------------------------------------------

void filterColor( cv::Point & redPt, cv::Point & greenPt, bool & redPointFound, bool & greenPointFound, cv::Mat & imageHSV )	{
  cv::namedWindow("ColorFilter");
  cout <<"Filter Start"<<endl;

  vector <cv::Mat>  hsvChannel(3);
  hsvChannel[0] = cv::Mat(imageHSV.size().height, imageHSV.size().width, imageHSV.depth());
  hsvChannel[1] = cv::Mat(imageHSV.size().height, imageHSV.size().width, imageHSV.depth());
  hsvChannel[2] = cv::Mat(imageHSV.size().height, imageHSV.size().width, imageHSV.depth());
  
  cv::split(imageHSV, hsvChannel);
  cout <<"Hue Separated" <<endl;
//******************************************************************************************************
//************************Green Color*******************************************************************
//******************************************************************************************************
//   cv::imshow("ColorFilter", hsvChannel[0]);
//   cv::waitKey(0);

    sprintf(filename, "/home/cooplab/jianxin/rosbuild_ws/sandbox/follower/images/image_hue.png");
    imwrite( filename, hsvChannel[0] );

    sprintf(filename, "/home/cooplab/jianxin/rosbuild_ws/sandbox/follower/images/image_sat.png");
    imwrite( filename, hsvChannel[1] );
    
    //exit(1);
  cout <<"Hue Saved" <<endl;  
  //Thresholding
  cv::threshold( hsvChannel[0], hsvChannel[0], 140, 255, CV_THRESH_TOZERO_INV);
  cv::threshold( hsvChannel[0], hsvChannel[0], 80, 255, CV_THRESH_TOZERO);
  cv::threshold( hsvChannel[0], hsvChannel[0], 0, 255, CV_THRESH_BINARY);

  cv::Mat myThresholdG;
  cv::Scalar glowerb(50,70,50);
  cv::Scalar gupperb(100,255,255);
  cv::inRange(imageHSV,glowerb,gupperb,myThresholdG);
  cv::threshold(myThresholdG,myThresholdG,0,255,CV_THRESH_BINARY);

  sprintf(filename, "/home/cooplab/jianxin/rosbuild_ws/sandbox/follower/images/image_green.png");
  imwrite( filename, myThresholdG );

  cv::Mat myThresholdR;

  cv::Scalar rlowerb(20,0,0);
  cv::Scalar rupperb(160,255,255);
  cv::inRange(imageHSV,rlowerb,rupperb,myThresholdR);
  cv::threshold(myThresholdR,myThresholdR,0,255,CV_THRESH_BINARY_INV);

  sprintf(filename, "/home/cooplab/jianxin/rosbuild_ws/sandbox/follower/images/image_red.png");
  imwrite( filename, myThresholdR );


  cout <<"Thresholded" <<endl;
  //cv::erode( hsvChannel[0], hsvChannel[0], cv::Mat(),cv::Point(-1,-1), 2);

//   cv::imshow("ColorFilter", hsvChannel[0]);
//   cv::waitKey(0);

  //Bounding Box based filtering
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;

  findContours( myThresholdG, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );  
  int idx = 0;
  cv::Mat tempImage = cv::Mat::zeros(imageHSV.size().height, imageHSV.size().width, imageHSV.depth());
  cout <<"Contoured once" <<endl;
  if ( !contours.empty() && !hierarchy.empty() ) 	
  {
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
      cv::Rect bbRect = cv::boundingRect(contours[idx]);
      if(bbRect.width/bbRect.height>0.33 || bbRect.width/bbRect.height<3)
	cv::drawContours( tempImage, contours, idx, cv::Scalar(255), CV_FILLED, 8, hierarchy );
    }
  }
  tempImage.copyTo(myThresholdG);

//   cv::imshow("ColorFilter", hsvChannel[0]);
//   cv::waitKey(0);
  
//   //Area based filtering
  findContours( myThresholdG, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
  idx = 0;
  double area = 0, maxArea = 0;
  tempImage = cv::Scalar(0);
  if ( !contours.empty() && !hierarchy.empty() ) 	
  {
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
      area = cv::contourArea(contours[idx]);
      if( area > maxArea )	maxArea = area;
      if(area > 50 && area < 20000)	{
	//cout<<"Area = "<<cv::contourArea(contours[idx])<<endl;
 	//cv::drawContours( tempImage, contours, idx, cv::Scalar(255), CV_FILLED, 8, hierarchy );
      }
    }
  }
  cout <<"COntoured twice" <<endl;
//   tempImage.copyTo(hsvChannel[0]);

//   cv::imshow("ColorFilter", hsvChannel[0]);
//   cv::waitKey(0);

  //Max area based filtering
  //  findContours( hsvChannel[0], contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );


  findContours( myThresholdG, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
  idx = 0;
  area = 0;
  tempImage = cv::Scalar(0);
  cv::Moments areaMoments;
  if ( !contours.empty() && !hierarchy.empty() ) 	
  {
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
      area = cv::contourArea(contours[idx]);
      if(area == maxArea && area > 50)	{
	//cout<<"Area = "<<cv::contourArea(contours[idx])<<endl;
	cv::drawContours( tempImage, contours, idx, cv::Scalar(255), CV_FILLED, 8, hierarchy );
	
	greenPointFound = true;
	areaMoments = cv::moments(contours[idx]);
	greenPt.x = (int) areaMoments.m10/areaMoments.m00;
	greenPt.y = (int) areaMoments.m01/areaMoments.m00;
	break;
      }
    }
  }
  tempImage.copyTo(myThresholdG);
  imshow("green",myThresholdG);  
//   cv::imshow("ColorFilter", hsvChannel[0]);
//   cv::waitKey(0);
  
  //Erosion
  //cv::erode( hsvChannel[0], hsvChannel[0], cv::Mat(),cv::Point(-1,-1), 5);

//******************************************************************************************************
//************************Red Color*******************************************************************
//******************************************************************************************************

  //Thresholding
  //cv::threshold( hsvChannel[1], hsvChannel[1], 150, 255, CV_THRESH_BINARY);
  //cv::erode( hsvChannel[0], hsvChannel[0], cv::Mat(),cv::Point(-1,-1), 2);
  
  //Green Region Mask
  //tempImage = cv::Scalar(255);
  //cv::subtract(tempImage, hsvChannel[0], tempImage);
  //cv::Mat tempImage1 = cv::Mat::zeros(imageHSV.size().height, imageHSV.size().width, imageHSV.depth());
  //hsvChannel[1].copyTo(tempImage1, tempImage);
  //tempImage1.copyTo(hsvChannel[1]);
  //cv::erode( hsvChannel[1], hsvChannel[1], cv::Mat(),cv::Point(-1,-1), 2);
  
  //Bounding Box based filtering
  findContours( myThresholdR, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );  
  idx = 0;
  tempImage = cv::Scalar(0);
  if ( !contours.empty() && !hierarchy.empty() ) 	
  {
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
      cv::Rect bbRect = cv::boundingRect(contours[idx]);
      if(bbRect.width/bbRect.height>0.33 || bbRect.width/bbRect.height<3)
	cv::drawContours( tempImage, contours, idx, cv::Scalar(255), CV_FILLED, 8, hierarchy );
    }
  }
  tempImage.copyTo(myThresholdR);

//   //Area based filtering
  findContours( myThresholdR, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
  idx = 0;
  area = 0;
  maxArea = 0;
  tempImage = cv::Scalar(0);
  if ( !contours.empty() && !hierarchy.empty() ) 	
  {
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
      area = cv::contourArea(contours[idx]);
      if( area > maxArea )	maxArea = area;
      if(area > 50 && area < 20000)	{
	//cout<<"Area = "<<cv::contourArea(contours[idx])<<endl;
// 	cv::drawContours( tempImage, contours, idx, cv::Scalar(255), CV_FILLED, 8, hierarchy );
      }
    }
  }
//   tempImage.copyTo(hsvChannel[1]);

  //Max area based filtering
  //findContours( hsvChannel[1], contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
  findContours( myThresholdR, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
  idx = 0;
  area = 0;
  tempImage = cv::Scalar(0);
  if ( !contours.empty() && !hierarchy.empty() ) 	
  {
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
      area = cv::contourArea(contours[idx]);
      if(area == maxArea && area > 50)	{
	//cout<<"Area = "<<cv::contourArea(contours[idx])<<endl;
	cv::drawContours( tempImage, contours, idx, cv::Scalar(255), CV_FILLED, 8, hierarchy );

	redPointFound = true;
	areaMoments = cv::moments(contours[idx]);
	redPt.x = (int) areaMoments.m10/areaMoments.m00;
	redPt.y = (int) areaMoments.m01/areaMoments.m00;
	break;
      }
    }
  }
  tempImage.copyTo(myThresholdR);
  imshow("red",myThresholdR);
    
//   cv::imshow("ColorFilter", hsvChannel[0]);
//   cv::waitKey(0);
//   cv::imshow("ColorFilter", hsvChannel[1]);
//   cv::waitKey(0);
//   cv::imshow("ColorFilter", hsvChannel[2]);
//   cv::waitKey(0);

   // exit(1);
  cout <<"Filter ..."<<endl;
}

//----------------------------------------------------------------------------

void imageCallback(const sensor_msgs::Image::ConstPtr& camImageMsg)	{
  cout<<"Image received ..."<<endl;
  cv_bridge::CvImagePtr camImage;
  cv::Point redPt, greenPt;
  bool redPointFound = false, greenPointFound = false;
  
  try
  {
    camImage = cv_bridge::toCvCopy(camImageMsg, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat camImageHSV(camImage->image.size().height, camImage->image.size().width, camImage->image.depth());
  cv::cvtColor(camImage->image, camImageHSV, CV_BGR2HSV);
  std::cout <<"Depth: "<<camImage->image.depth()<<endl;
  filterColor(redPt, greenPt, redPointFound, greenPointFound, camImageHSV);
  transformPoints(redPt, greenPt, redPointFound, greenPointFound);
  
  int lineThickness = 10;
  int lineRadius = 20;
  cv::Scalar redColor(0,0,255);
  cv::Scalar greenColor(0,255,0);
  cv::Scalar blackColor(0,0,0);
  if(redPointFound && frame_counter > frameThreshold)	{
    circle(camImage->image, redPt, lineRadius, redColor, lineThickness);
  }
  else	{
    cerr<<"Oopsy! No red point found"<<endl;
  }
  if(greenPointFound && frame_counter > frameThreshold)	{
    circle(camImage->image, greenPt, lineRadius, greenColor, lineThickness);
  }
  else	{
    cerr<<"Oopsy! No green point found"<<endl;
  }
  if(redPointFound && greenPointFound && frame_counter > frameThreshold)	{
    line(camImage->image, redPt, greenPt, blackColor, lineThickness);
  }

  skip_counter++;
  cout<<"Skip = "<<skip_counter<<endl;
  //if(skip_counter == 10)		{
    skip_counter = 0;
    counter++;
    sprintf(filename, "/home/cooplab/jianxin/rosbuild_ws/sandbox/follower/images/image%05d.png",counter);
    imwrite( filename, camImage->image );
  //}

  //cout<<"Red Point\t:\t"<<redPt.x<<"\t"<<redPt.y<<endl;
  //cout<<"Green Point\t:\t"<<greenPt.x<<"\t"<<greenPt.y<<endl;
  //exit(1);
  
  cv::namedWindow(WINDOW);
  cv::imshow(WINDOW, camImageHSV);
  cv::imshow(WINDOW, camImage->image);
  cv::waitKey(3);
  
  cout<<endl;
    
  cvDestroyWindow("cam");
  ReleaseData();
}

//----------------------------------------------------------------------------

int main(int argc, char **argv)	{
  ros::init(argc, argv, "follower_temp");
  
  system("rm -r /home/cooplab/jianxin/rosbuild_ws/sandbox/follower/images");
  system("mkdir /home/cooplab/jianxin/rosbuild_ws/sandbox/follower/images");
  cout<<"Starting follower routine ..."<<endl;
  ros::NodeHandle n;

  //image_transport::ImageTransport it(nh);
  //image_transport::Publisher pub = it.advertise("robot_image", 1);
  
  pos_points_pub = n.advertise<std_msgs::Float32MultiArray>("pos_points_temp", 1000);
  
  //ros::Subscriber camera_sub = n.subscribe("camera/image", 10, imageCallback);
  //ros::Timer timer = n.createTimer(ros::Duration(control_period), controlCallback);
  image_transport::ImageTransport it(n);
  image_transport::Subscriber camera_sub = it.subscribe("/camera/image", 10, imageCallback);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
