/**
  * Large Map 
  * Laser based memorybased (learning) navigating  jockey
  *
  */

#include <iostream>
#include <fstream>
#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PolygonStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "lama_interfaces/lama_learning_jockey.h"
#include "lama_interfaces/lama_navigating_jockey.h"
#include "lama_interfaces/lmi_laser_descriptor_set.h"
#include "lama_interfaces/lmi_laser_descriptor_get.h"

#include "hist.h"
namespace li = lama_interfaces;
ros::Publisher pub;
ros::Publisher draw;
ros::Publisher drawRef;
sensor_msgs::LaserScan first;
sensor_msgs::LaserScan last;
std::vector<sensor_msgs::LaserScan> descriptor;

bool assigned;
bool learning = false;
bool navigating = false;
bool interrupted = false;
int angleHistBin=100;
float max_error = 0.2;
float max_distance = 0.3;
float min_distance = 0.05;
unsigned int navigation_index = 0;


void handleLaser(sensor_msgs::LaserScan msg) {
   if (learning && !interrupted) {
      if (!assigned) {
	 first = msg;
	 descriptor.push_back(first);
	 assigned = true; 
      }
       ROS_DEBUG("LEARNING laser arrived %lu",msg.ranges.size());
      hist(msg,100);
      geometry_msgs::Pose2D goal = localize(msg, first, angleHistBin);
      geometry_msgs::Pose2D fix;
      fix.x= fix.y=fix.theta = 0;
      float err = error (msg,first,goal,draw,drawRef);    
      float distance = sqrt (goal.x*goal.x+goal.y*goal.y) ;
      if (err > max_error) {
	 ROS_INFO("error exceeds max limit");
	 first = last;
	 descriptor.push_back(first);
	 
      } 
      if (distance > max_distance) {
	 ROS_DEBUG("distance is longer than max");
	 first = msg;
	 descriptor.push_back(first);
      }


     /* float deviation = atan2(goal.y, goal.x)*0.5;
      if (deviation > 0.8) deviation = 0.8;
      if (deviation < -0.8) deviation = -0.8;
      float speed = 2*distance *(0.5-fabs(deviation)) ; 
      if (speed < 0.0) speed = 0.00;


      geometry_msgs::Twist pub_msg;
      pub_msg.linear.x = 0.8*speed;
      pub_msg. angular.z = 0.8*deviation;
      pub.publish(pub_msg);*/
      last = msg;
   }

   if (navigating) {
      if (!assigned) {
	 navigation_index = 0; 
	 first = descriptor[navigation_index++];
	 assigned = true; 
      }
      hist(msg,100);
      geometry_msgs::Pose2D goal = localize(msg, first, angleHistBin);
      geometry_msgs::Pose2D fix;
      fix.x= fix.y=fix.theta = 0;
      float err = error (msg,first,goal,draw,drawRef);    
      float distance = sqrt (goal.x*goal.x+goal.y*goal.y) ;
       ROS_INFO("error %f goal %f %f ", err, goal.x, goal.y);
   /*   if (err > max_error) {
	 ROS_INFO("error exceeds max limit");
	 first = descriptor[navigation_index++];
	 //descriptor.push_back(first);

      } */
      if (distance < min_distance) {
	 ROS_INFO("distance shorter than min");
	 if (navigation_index < descriptor.size()-1) {
            first = descriptor[navigation_index++];
	 } else {
	    navigating = false;
	 }
	 //first = ;
	 //descriptor.push_back(first);
      }


      float deviation = atan2(goal.y, goal.x)*0.5;
      if (deviation > 0.8) deviation = 0.8;
      if (deviation < -0.8) deviation = -0.8;
      float speed = 2*distance *(0.8-fabs(deviation)) ; 
      if (speed < -0.05) speed = -0.05;


      geometry_msgs::Twist pub_msg;
      pub_msg.linear.x = 0.9*speed;
      pub_msg. angular.z = 1.1*deviation;
      pub.publish(pub_msg);
      last = msg;

   }

}

bool navigationCallback(li::lama_navigating_jockeyRequest& request, li::lama_navigating_jockeyResponse& response){
   li::lmi_laser_descriptor_get ldg;
   switch (request.request_action.action)  {
      case request.request_action.STOP: 	// stop learn
	 ROS_INFO("STOP");
	 navigating = false;
	 interrupted = false;
	 response.actual_state.state = response.actual_state.DONE;
	 break;
      case request.request_action.TRAVERSE:  	// start learn
	 if (!interrupted) {
	    ROS_INFO("navigation START");
	    descriptor.clear();
	    ldg.request.id = request.descriptor;
	    ros::service::call("lmi_laser_descriptor_getter",ldg);
            descriptor = ldg.response.descriptor;
	    assigned = false;
	    navigating = true;
	    response.actual_state.state = response.actual_state.TRAVERSING;
	 } else {
	    response.actual_state.state = response.actual_state.INTERRUPTED;
	 }

	 break;
   }
   return true;
}

bool serviceCallback(li::lama_learning_jockeyRequest& request, li::lama_learning_jockeyResponse& response){

   li::LamaDescriptorIdentifier desc_id;
   li::lmi_laser_descriptor_set lds;

   switch (request.request_action.action)  {
      case request.request_action.STOP_LEARN: 	// stop learn
	 ROS_INFO("STOP");
	 learning = false;
	 interrupted = false;
	 lds.request.descriptor = descriptor;
	 ros::service::call("lmi_laser_descriptor_setter",lds);
	 desc_id = lds.response.id;
	 response.descriptor = desc_id;
	 response.actual_state.state = response.actual_state.DONE;
	 break;
      case request.request_action.START_LEARN:  	// start learn
	 if (!interrupted) {
	    descriptor.clear();
	    assigned = false;
	    learning = true;
	    response.actual_state.state = response.actual_state.LEARNING; 
	 } else {
	    response.actual_state.state = response.actual_state.INTERRUPTED;
	 }

	 break;
      case request.request_action.INTERRUPT: 	// interrupt
	 if (learning) {
	    interrupted = true;
	    learning = false;
	    response.actual_state.state = response.actual_state.INTERRUPTED;
	 } else {
	    response.actual_state.state = response.actual_state.DONE;
	 }
	 break;
      case request.request_action.CONTINUE: //continue
	 if (interrupted) { 
	    learning = true;
	    interrupted = false;
	    response.actual_state.state = response.actual_state.LEARNING;	 
	 } else {
	    response.actual_state.state = response.actual_state.DONE;
	 }	    
	 break;
      default:
	 if (interrupted) {
	    response.actual_state.state = response.actual_state.INTERRUPTED;
	 }
	 else { 
	    if (learning) {
	       response.actual_state.state = response.actual_state.LEARNING;
	    } else {
	       response.actual_state.state = response.actual_state.DONE; 
	    }
	 }
   }
   return true;

}

int main(int argc, char **argv) {

   ros::init(argc, argv, "laser_navigating_jockey");
   assigned = false;
   ros::NodeHandle n;
   ros::Subscriber laserHandler = n.subscribe<sensor_msgs::LaserScan> ("base_scan", 50 , handleLaser);
   ros::ServiceServer learningJockey = n.advertiseService ( "learning_jockey", serviceCallback);
   ros::ServiceServer navigatingJockey = n.advertiseService ( "navigating_jockey", navigationCallback);

   pub = n.advertise<geometry_msgs::Twist> ("nav_cmd",1,false);
   draw = n.advertise<geometry_msgs::PolygonStamped> ("draw",1,false);
   drawRef = n.advertise<geometry_msgs::PolygonStamped> ("draw_reference",1,false);
   ROS_INFO("started");
   ros::spin();

}

#undef intro

