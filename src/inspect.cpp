//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
// #include <stdio.h>
// #include <stdlib.h>
// #include <vector>
// #include <iostream>
// #include <math.h>

#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Twist.h"

//#include <LinearMath/btQuaternion.h> // Needed to convert rotation ...
//#include <LinearMath/btMatrix3x3.h>  // ... quaternion into Euler angles

//----------------------------------------------------------------------------

using namespace std;

#define PI 3.14159265

ros::Publisher cmd_vel_pub;
ros::Publisher marker_pub;

double kp_turn = 0.5;

double control_period = 0.1;   // a cmd_vel message is published every this many seconds

double default_speed = 0.5;
double default_turn = 0.2;

double current_speed_command = 0.0;
double current_turn_command = 0.0;

double current_dist=0; 
double current_heading_error=0;
double current_local_theta=0; 

double NO_READING = -500;

double DIST_MAX = 800; // [mm]
double DIST_MIN = 200; // [mm]
double CAUTION = 50;  // [mm]
double HEADING_ERROR_MAX = PI/6; // in [rad] (~20 degrees)
float Goal[6];

/*double goal_angle_threshold = 0.01;

double goal_dist_threshold = 0.05;          // how close to goal point do we have to be?

double obstacle_dist_stop_threshold = 0.5;  // stop when this close to any obstacle
*/
bool goal_exists = false;


//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// draw sphere with text in rviz

void send_sphere_marker(string frame_id, int marker_id, 
			float x, float y, float z, float radius,
			float r, float g, float b, 
			string text, float text_z_offset)
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = marker_id;
  
  // Set the marker type.  
  marker.type = visualization_msgs::Marker::SPHERE;
  
  // Set the marker action.  Options are ADD and DELETE
  marker.action = visualization_msgs::Marker::ADD;
  
  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = radius;
  marker.scale.y = radius;
  marker.scale.z = radius;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  
  marker.lifetime = ros::Duration();
  
  // Publish the marker
  marker_pub.publish(marker);

  // add text annotation 

  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.text = text;
  marker.id = marker_id+1;
  marker.pose.position.z = z + text_z_offset;

  marker_pub.publish(marker);
}

//----------------------------------------------------------------------------

// publish velocity command

void send_cmd_vel(float lx, float ly, float lz, float ax, float ay, float az)
{
  geometry_msgs::Twist tw_msg;

  tw_msg.linear.x = lx;   // always go forward
  tw_msg.linear.y = ly;
  tw_msg.linear.z = lz;
  
  tw_msg.angular.x = ax;
  tw_msg.angular.y = ay;
  tw_msg.angular.z = az;
  
  cmd_vel_pub.publish(tw_msg);
}

//----------------------------------------------------------------------------

// dist = distance to goal (midpoint)

void compute_distance_and_orientation(double & dist, double & heading_error, double & local_theta)
{
  double theta_head;
  double theta_tail;
  //double local_theta;
  if (Goal[0] == NO_READING || Goal[3] == NO_READING)
    return;
  else {
  dist = sqrt( pow((Goal[0]+Goal[3])/2,2) + pow((Goal[1]+Goal[4])/2,2.0));
  
  theta_head = atan2(Goal[1],Goal[0]);
  theta_tail = atan2(Goal[4],Goal[3]);
  if (theta_head <= PI/2)
    heading_error = theta_head - (PI/2-HEADING_ERROR_MAX);
  else
    heading_error = theta_head - (PI/2+HEADING_ERROR_MAX);
  printf("C debug: heading_error: [%.3lf]\n", heading_error);
  local_theta = atan2((Goal[1]-Goal[4]),(Goal[0]-Goal[3]));
  }
}
//----------------------------------------------------------------------------

void inspect()
{
  if (goal_exists) {
    
    compute_distance_and_orientation(current_dist, current_heading_error, current_local_theta);
    
    if (current_dist < DIST_MAX && current_dist > (DIST_MIN+CAUTION)) {
      printf("C: Inspecting... Please stand by.\n");
      
      if (fabs(current_heading_error) > HEADING_ERROR_MAX) {  // || current_heading_error > HEADING_ERROR_MIN) {
      current_speed_command = 0.0;
      current_turn_command = kp_turn * current_heading_error;
      return;
      }
    }
    if (current_dist > DIST_MAX) {
      printf("C: Following... [%.3lf m, %.3lf rads] away\n", current_dist, current_heading_error);
      current_speed_command = default_speed;
      current_turn_command = kp_turn * current_heading_error;
      return;
    }
    if (current_dist < DIST_MIN+CAUTION && current_dist > DIST_MIN) {
      printf("C: Caution... Collision imminent!![%.3lf m, %.3lf rads] away\n", current_dist, current_heading_error);
      current_speed_command = 0.0;
      current_turn_command = 0.0;
      return;
    }
    if (current_dist < DIST_MIN) {
      printf("C: Avoiding... [%.3lf m, %.3lf rads] away\n", current_dist, current_heading_error);
      // First attempt: Just move backwards
      current_speed_command = -default_speed;
      current_turn_command = kp_turn * current_heading_error;
      return; // this function needs caution since the create may hit an object when moving backwards
    }
  }
}
//----------------------------------------------------------------------------

// this is called at a constant rate and is responsible for 
// sending motion commands based on the what the current controller specifies

void controlCallback(const ros::TimerEvent& t)
{
  //  printf("sending %.3lf %.3lf\n", current_speed_command, current_turn_command); fflush(stdout);

  send_cmd_vel(current_speed_command, 0, 0, 0, 0, current_turn_command);
}
//----------------------------------------------------------------------------
 
void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array) 
{
  // let the current controller know that we *have* a goal
  
  goal_exists = true;

  // reset straight controller mode
  
  int i = 0;
	// print all the remaining numbers
	for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		Goal[i] = *it;
		//cout<<Goal[i]<<endl;
		i++;
	}
	
  // and visualize it in rviz (with random id)

  send_sphere_marker("/odom", 66, 
		     Goal[0], Goal[1], Goal[2],
		     0.25,
		     1, 0, 0, 
		     "HEAD", 0.5);
  
  send_sphere_marker("/odom", 66, 
		     Goal[3], Goal[4], Goal[5],
		     0.25,
		     0, 1, 0, 
		     "TAIL", 0.5);
  
  inspect();
}

//----------------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inspect");

  ros::NodeHandle n;

  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::Subscriber sub3 = n.subscribe("pos_points", 100, arrayCallback);

  ros::Timer timer = n.createTimer(ros::Duration(control_period), controlCallback);

  ros::Rate loop_rate(10);

  while (ros::ok()) {


    ros::spinOnce();

    loop_rate.sleep();

  }

  return 0;
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

