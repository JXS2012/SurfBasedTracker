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
bool lastdetect = false;
std::vector<KeyPoint> keypoints_template, keypoints_last;
Mat descriptors_template, descriptors_last;

int minHessian = 800;
SurfFeatureDetector detector( minHessian,4,2,true,true );
SurfDescriptorExtractor extractor;

FlannBasedMatcher matcher;

int feature_size_threshold = 1;
cv::Mat robotTemplate;

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
cv::Mat getHSVImage(cv::Mat raw)
{
  cv::Mat rawHSV(raw.size().height, raw.size().width, raw.depth());
  
  cv::cvtColor(raw, rawHSV, CV_BGR2HSV);
  vector <cv::Mat>  hsvChannel(3);
  hsvChannel[0] = cv::Mat(rawHSV.size().height, rawHSV.size().width, rawHSV.depth());
  hsvChannel[1] = cv::Mat(rawHSV.size().height, rawHSV.size().width, rawHSV.depth());
  hsvChannel[2] = cv::Mat(rawHSV.size().height, rawHSV.size().width, rawHSV.depth());
  
  cv::split(rawHSV, hsvChannel);

  return hsvChannel[0];
  
  //cv::cvtColor(raw, rawHSV, CV_BGR2GRAY);
  //return rawHSV;
}

//----------------------------------------------------------------------------
cv::Mat getGrayImage(cv::Mat raw)
{
  cv::Mat rawGray(raw.size().height, raw.size().width, raw.depth());
  
  cv::cvtColor(raw, rawGray, CV_BGR2GRAY);
  return rawGray;
}

//----------------------------------------------------------------------------
cv::Mat processImage(cv::Mat raw)
{
  return getHSVImage(raw);
  //return getGrayImage(raw);
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
void checkDist(const std::vector<DMatch> & matches, const cv::Mat & descriptors_image, double & min_dist, double & max_dist)
{
  for( int i = 0; i < descriptors_image.rows; i++ )
    { 
      double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
    }
}

//----------------------------------------------------------------------------
std::vector< DMatch > filterGoodMatches(const std::vector<DMatch> & matches, const cv::Mat & descriptors_image, double min_dist)
{
  std::vector<DMatch> good_matches;
  for( int i = 0; i < descriptors_image.rows; i++ )
    { if( matches[i].distance <= max(2*min_dist, 0.02) )
	{ good_matches.push_back( matches[i]); }}
  return good_matches;
}

//----------------------------------------------------------------------------
void showGoodMatches( const cv::Mat & imageFrame, const std::vector<KeyPoint> & keypoints_image, const cv::Mat & exampleFrame, const std::vector<KeyPoint> & keypoints_example, const std::vector< DMatch > & good_matches, String title)
{
  Mat img_matches;
  drawMatches( imageFrame, keypoints_image, exampleFrame, keypoints_example,
	       good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
	       vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
  imshow( title, img_matches );
}

//----------------------------------------------------------------------------
cv::Point2f computeCOM(const std::vector<cv::Point2f> p)
{
  cv::Point2f cen(0,0);
  for ( size_t i=0; i< p.size(); i++ )
    {
      cen.x += p[i].x;
      cen.y += p[i].y;
    }
  cen.x /= p.size(); 
  cen.y /= p.size(); 
  return cen;
}

//----------------------------------------------------------------------------
void detectRobot(const cv::Mat & current, std::vector<KeyPoint> & keypoints_current,  cv::Mat & descriptors_current, cv::Mat & currentFrame, bool & detect)
{
  cv::Point2f cen(0,0);

  resize(current, currentFrame, Size(), 0.5, 0.5, CV_INTER_AREA);

  detector.detect( currentFrame, keypoints_current );
  extractor.compute( currentFrame, keypoints_current, descriptors_current );
  
  std::vector< DMatch > matches;      
  matcher.match( descriptors_current,descriptors_template, matches );

  double max_dist = 0; double min_dist = 100;
  checkDist(matches, descriptors_current, min_dist, max_dist);

  if (min_dist < 0.2)
    {
      std::vector< DMatch > good_matches = filterGoodMatches(matches, descriptors_current, min_dist);
      
      if ( good_matches.size() > feature_size_threshold )
	{
	  detect = true;
	  ROS_INFO("DETECT");
	  showGoodMatches(currentFrame, keypoints_current, robotTemplate, keypoints_template, good_matches, "Detection");
	  /*
	  std::vector<KeyPoint> keypoints_current_tmp;
	  Mat descriptors_current_tmp;
	  for( int i = 0; i < (int)good_matches.size(); i++ )
	    keypoints_current_tmp.push_back(keypoints_current[good_matches[i].queryIdx]);
	  
	  extractor.compute( currentFrame, keypoints_current_tmp, descriptors_current_tmp );
	  descriptors_current = descriptors_current_tmp;
	  keypoints_current.clear();
	  keypoints_current = keypoints_current_tmp;
	  */
	  std::vector<cv::Point2f> p2s;
  
	  for( int i = 0; i < (int)good_matches.size(); i++ )
	    {
	      p2s.push_back(keypoints_current[good_matches[i].queryIdx].pt);
	    }

	  cen = computeCOM(p2s);

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

}

//----------------------------------------------------------------------------
void computeRobotTransform(const std::vector<KeyPoint> & keypoints_current, const cv::Mat & descriptors_current, const cv::Mat & currentFrame)
{
    std::vector< DMatch > matches;      

    if ( keypoints_last.size() == 0 || keypoints_current.size() == 0 )
      {
	printf("target out of view, featureless \n");
	return;
      }
      
    matcher.match( descriptors_last, descriptors_current, matches );

    double max_dist = 0;     double min_dist = 100;
    checkDist(matches, descriptors_last, min_dist, max_dist);
    
    std::vector< DMatch > good_matches = filterGoodMatches(matches, descriptors_last, min_dist);
    
    showGoodMatches(lastFrame, keypoints_last, currentFrame, keypoints_current, good_matches,"Transition");

    std::vector<cv::Point2f> p1s,p2s;
    
    for( int i = 0; i < (int)good_matches.size(); i++ )
      {
	p1s.push_back(keypoints_last[good_matches[i].queryIdx].pt);
	p2s.push_back(keypoints_current[good_matches[i].trainIdx].pt);
      }
    
    cv::Mat transform = cv::estimateRigidTransform(p1s,p2s,true);
      
    transformPublish(transform);

}

//----------------------------------------------------------------------------
void trackRobot(const cv::Mat & current)
{
  std::vector<KeyPoint> keypoints_current;
  cv::Mat currentFrame, transform, descriptors_current;

  bool detect = false;
  detectRobot(current, keypoints_current, descriptors_current, currentFrame, detect);
  if (detect && lastdetect)
    computeRobotTransform(keypoints_current, descriptors_current, currentFrame);

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
  cv::Mat processedImage = processImage(camImage->image);

  clock_t start = clock();
  //trackRobot(camImage->image);
  trackRobot(processedImage);
  clock_t end = clock();
  double time = (double) (end-start) / CLOCKS_PER_SEC * 1000.0;
  std::cout <<"run time: "<<time<<"\n";


  skip_counter++;
  cout<<"Skip = "<<skip_counter<<endl;
  //  if(skip_counter == 10)		{
    skip_counter = 0;
    counter++;
    sprintf(filename, "/home/jianxin/rosbuild_ws/sandbox/follower/images/track%05d.png",counter);
    imwrite( filename, camImage->image );
    // }
  
  cv::waitKey(3);
  
  imshow("raw",camImage->image);
  ReleaseData();
}

//----------------------------------------------------------------------------

int main(int argc, char **argv)	{
  string templateName;
  if (argc >= 2)
    templateName = argv[2];
  else
    templateName = "/home/jianxin/rosbuild_ws/sandbox/follower/template/template2.png";

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
  robotTemplate = imread( templateName );

  robotTemplate = processImage(robotTemplate);

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
