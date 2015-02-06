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
#include "time.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/video/tracking.hpp"
//#include <opencv/cxcore.hpp>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

//----------------------------------------------------------------------------

ros::Publisher pos_points_pub;
ros::Publisher com_pub;
bool detect = false;
bool lastdetect = false;
std::vector<KeyPoint> keypoints_template, keypoints_last;
Mat descriptors_template, descriptors_last;

int minHessian = 400;
SurfFeatureDetector detector( minHessian,4,2,false,true );
SurfDescriptorExtractor extractor;
FlannBasedMatcher matcher;

int feature_size_threshold = 5;
cv::Mat robotTemplate = imread( "/home/jianxin/rosbuild_ws/sandbox/follower/template/template2.png", IMREAD_GRAYSCALE );

cv::Mat lastFrame;

//----------------------------------------------------------------------------

#define		WIDTH_MAX 	8400
#define		WIDTH_MIN	3700
#define 	HEIGHT		7300
#define		FRAME_WIDTH	160
#define		FRAME_HEIGHT	120
//----------------------------------------------------------------------------

static const char WINDOW[] = "Image window";
char *filename = (char*) calloc(256, sizeof(char));
int counter = 0, skip_counter = 0, frame_counter = 0, frameThreshold = 2;

//----------------------------------------------------------------------------
// Release Data

void ReleaseData()	{
  //if( camImage )	camImage->image.release();  
}

//----------------------------------------------------------------------------

void COMPublish(const cv::Point2f pt)
{
  std_msgs::Float32MultiArray pts;

  if ((pt.x != 0) || (pt.y != 0))
    {
      //ROS_INFO("DISPLACEMENT X %f Y %f", transform.at<double>(0,2), transform.at<double>(1,2));
      pts.data.push_back(pt.x - FRAME_WIDTH / 2.);
      pts.data.push_back(FRAME_HEIGHT / 2. - pt.y);
    }
  else
    {
      //ROS_INFO("DISPLACEMENT X %f Y %f", 0., 0.);
      pts.data.push_back(0);
      pts.data.push_back(0);
    }
  com_pub.publish(pts);
}

//----------------------------------------------------------------------------

void transformPublish(const cv::Mat transform)
{
  std_msgs::Float32MultiArray pts;

  if (!transform.empty())
    {
      //ROS_INFO("DISPLACEMENT X %f Y %f", transform.at<double>(0,2), transform.at<double>(1,2));
      pts.data.push_back(transform.at<double>(0,2));
      pts.data.push_back(transform.at<double>(1,2));
    }
  else
    {
      //ROS_INFO("DISPLACEMENT X %f Y %f", 0., 0.);
      pts.data.push_back(0);
      pts.data.push_back(0);
    }
  pos_points_pub.publish(pts);
}

//----------------------------------------------------------------------------
std::vector<KeyPoint> detectRobot(const cv::Mat & current, const cv::Mat & currentFrame, boolean & detect)
{
  cv::Point2f cen(0,0);

  resize(current, currentFrame, Size(), 0.5, 0.5, CV_INTER_AREA);

  std::vector<KeyPoint> keypoints_current;
  detector.detect( currentFrame, keypoints_current );
  Mat descriptors_current;
  extractor.compute( currentFrame, keypoints_current, descriptors_current );
  std::vector< DMatch > matches;      
  matcher.match( descriptors_current,descriptors_template, matches );

  double max_dist = 0; double min_dist = 100;
  for( int i = 0; i < descriptors_current.rows; i++ )
    { 
      double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
    }

  if (min_dist < 0.2)
    {
      std::vector< DMatch > good_matches;
  
      for( int i = 0; i < descriptors_current.rows; i++ )
	{ if( matches[i].distance <= max(2*min_dist, 0.02) )
	    { good_matches.push_back( matches[i]); }}
      /*
      Mat img_matches;

      drawMatches( currentFrame, keypoints_current, robotTemplate, keypoints_template,
		   good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		   vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
      */

      
      if ( good_matches.size() > feature_size_threshold )
	{
	  detect = true;
	  ROS_INFO("DETECT");

	  std::vector<KeyPoint> keypoints_current_tmp;
	  Mat descriptors_current_tmp;
	  for( int i = 0; i < (int)good_matches.size(); i++ )
	    keypoints_current_tmp.push_back(keypoints_current[good_matches[i].queryIdx]);
	  
	  extractor.compute( currentFrame, keypoints_current_tmp, descriptors_current_tmp );
	  descriptors_current = descriptors_current_tmp;
	  keypoints_current.clear();
	  keypoints_current = keypoints_current_tmp;
	 
	  std::vector<cv::Point2f> p2s;
  
	  for( int i = 0; i < (int)good_matches.size(); i++ )
	    {
	      p2s.push_back(keypoints_current[good_matches[i].queryIdx].pt);
	    }
	  
	  for ( size_t i=0; i< p2s.size(); i++ )
	    {
	      cen.x += p2s[i].x;
	      cen.y += p2s[i].y;
	    }
	  cen.x /= p2s.size(); 
	  cen.y /= p2s.size(); 
	  COMPublish(cen);
	  /*
	  cv::circle( img_matches, cen, 10, Scalar(0,0,255), 2 );

	  imshow("COM",img_matches);
	  */
	}
      else {
	detect = false;
	ROS_INFO("Not detect");
      }
    }
  else
    {
      detect = false;
      ROS_INFO("Not detect");
    }

  return keypoints_current;
}

//----------------------------------------------------------------------------
void trackRobot(const cv::Mat & current, cv::Mat & transform)
{
  cv::Mat currentFrame;
  cv::Point2f cen(0,0);

  resize(current, currentFrame, Size(), 0.5, 0.5, CV_INTER_AREA);

  //-- Step 1: Detect the keypoints using SURF Detector
  std::vector<KeyPoint> keypoints_current;
  detector.detect( currentFrame, keypoints_current );
  //-- Step 2: Calculate descriptors (feature vectors)
  Mat descriptors_current;
  extractor.compute( currentFrame, keypoints_current, descriptors_current );

  //-- Step 3: Calculate matches between current image and template
  std::vector< DMatch > matches;      
  matcher.match( descriptors_current,descriptors_template, matches );

  //-- Quick calculation of max and min distances between keypoints
  double max_dist = 0; double min_dist = 100;
  for( int i = 0; i < descriptors_current.rows; i++ )
    { 
      double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
    }
  if (min_dist < 0.2)
    {
      std::vector< DMatch > good_matches;
  
      for( int i = 0; i < descriptors_current.rows; i++ )
	{ if( matches[i].distance <= max(2*min_dist, 0.02) )
	    { good_matches.push_back( matches[i]); }}
      
      if ( good_matches.size() > feature_size_threshold )
	{
	  detect = true;
	  ROS_INFO("DETECT");

	  std::vector<KeyPoint> keypoints_current_tmp;
	  Mat descriptors_current_tmp;
	  for( int i = 0; i < (int)good_matches.size(); i++ )
	    keypoints_current_tmp.push_back(keypoints_current[good_matches[i].queryIdx]);
	  
	  extractor.compute( currentFrame, keypoints_current_tmp, descriptors_current_tmp );
	  descriptors_current = descriptors_current_tmp;
	  keypoints_current.clear();
	  keypoints_current = keypoints_current_tmp;
	 
	  std::vector<cv::Point2f> p2s;
  
	  for( int i = 0; i < (int)good_matches.size(); i++ )
	    {
	      p2s.push_back(keypoints_current[good_matches[i].queryIdx].pt);
	    }
	  
	  for ( size_t i=0; i< p2s.size(); i++ )
	    {
	      cen.x += p2s[i].x;
	      cen.y += p2s[i].y;
	    }
	  cen.x /= p2s.size(); 
	  cen.y /= p2s.size(); 
	  COMPublish(cen);

	}
      else {
	detect = false;
	ROS_INFO("Not detect");
      }
    }
  else
    {
      detect = false;
      ROS_INFO("Not detect");
    }

  if (detect && lastdetect)
    {
      matches.clear();
      max_dist = 0;
      min_dist = 100;

      if ( keypoints_last.size() == 0 || keypoints_current.size() == 0 )
	{
	  printf("target out of view, featureless \n");
	  //transform = Mat::zeros(2,3,0);
	  return;
	}
      
      //-- Step 3: Matching descriptor vectors using FLANN matcher

      matcher.match( descriptors_last, descriptors_current, matches );
      
      //-- Quick calculation of max and min distances between keypoints
      for( int i = 0; i < descriptors_last.rows; i++ )
	{ 
	  double dist = matches[i].distance;
	  if( dist < min_dist ) min_dist = dist;
	  if( dist > max_dist ) max_dist = dist;
	}
      
      std::vector< DMatch > good_matches;
  
      for( int i = 0; i < descriptors_last.rows; i++ )
	{ if( matches[i].distance <= max(2*min_dist, 0.02) )
	    { good_matches.push_back( matches[i]); }
	}
  
      /*
      Mat img_matches;
      drawMatches( lastFrame, keypoints_last, currentFrame, keypoints_current,
		   good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		   vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
      */

      std::vector<cv::Point2f> p1s,p2s;
  
      for( int i = 0; i < (int)good_matches.size(); i++ )
	{
	  p1s.push_back(keypoints_last[good_matches[i].queryIdx].pt);
	  p2s.push_back(keypoints_current[good_matches[i].trainIdx].pt);
	}
  
      transform = cv::estimateRigidTransform(p1s,p2s,true);

      //-- Show detected matches
      //imshow( "Good Matches", img_matches );
    }

  keypoints_last = keypoints_current;
  descriptors_last = descriptors_current;
  lastFrame = currentFrame;
  lastdetect = detect;

}


//----------------------------------------------------------------------------

void imageCallback(const sensor_msgs::Image::ConstPtr& camImageMsg)	{
  cout<<"Image received ..."<<endl;
  cv_bridge::CvImagePtr camImage;
  
  try
  {
    camImage = cv_bridge::toCvCopy(camImageMsg, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat transform;
  clock_t start = clock();
  trackRobot(camImage->image, transform);
  clock_t end = clock();
  double time = (double) (end-start) / CLOCKS_PER_SEC * 1000.0;
  std::cout <<"run time: "<<time<<"\n";
  
  transformPublish(transform);

  skip_counter++;
  cout<<"Skip = "<<skip_counter<<endl;
  if(skip_counter == 10)		{
    skip_counter = 0;
    counter++;
    sprintf(filename, "/home/jianxin/rosbuild_ws/sandbox/follower/images/track%05d.png",counter);
    imwrite( filename, camImage->image );
  }
  
  cv::waitKey(3);
  
  imshow("raw",camImage->image);
  ReleaseData();
}

//----------------------------------------------------------------------------

int main(int argc, char **argv)	{
  ros::init(argc, argv, "follower_temp");
  
  system("rm -r /home/jianxin/rosbuild_ws/sandbox/follower/images");
  system("mkdir /home/jianxin/rosbuild_ws/sandbox/follower/images");
  ros::NodeHandle n;

  pos_points_pub = n.advertise<std_msgs::Float32MultiArray>("pos_points_temp", 1000);
  com_pub = n.advertise<std_msgs::Float32MultiArray>("com_temp", 1000);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber camera_sub = it.subscribe("/camera/image", 10, imageCallback);
  //image_transport::Subscriber camera_sub = it.subscribe("/ardrone/bottom/image_raw", 10, imageCallback);

  //cv::Mat robotTemplate = imread( "/home/jianxin/rosbuild_ws/sandbox/follower/template/frame0000.jpg", IMREAD_GRAYSCALE );
  imshow( "template", robotTemplate );
  waitKey(0);
  detector.detect( robotTemplate, keypoints_template );
  extractor.compute( robotTemplate, keypoints_template, descriptors_template );

  lastFrame = Mat::zeros(320,240,0);
  //lastFrame = Mat::zeros(320,640,0);
  detector.detect( lastFrame, keypoints_last );
  extractor.compute( lastFrame, keypoints_last, descriptors_last );

  cout<<"Starting follower routine ..."<<endl;

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
