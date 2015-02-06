/**
 * ROS node using localizing and navigating jockeys for test purposes
 *
 */

#include <ros/ros.h>
#include <ros/console.h>

#include <nlj_laser/nj_laser.h>
#include <nlj_laser/lj_laser.h>

#include <lama_interfaces/AddInterface.h>

ros::Publisher pub;

int main(int argc, char **argv)
{
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc, argv, "laser_jockey");
  ros::NodeHandle n;

  // Create the getter and setter services for Laser descriptors.
  ros::ServiceClient client = n.serviceClient<lama_interfaces::AddInterface>("interface_factory");
  client.waitForExistence();
  ROS_DEBUG("setting descriptor services");
  lama_interfaces::AddInterface srv;
  srv.request.interface_name = "laser_descriptor";
  srv.request.interface_type = lama_interfaces::AddInterfaceRequest::CLEARTEXT;
  srv.request.get_service_message = "nlj_laser/GetLaserDescriptor";
  srv.request.set_service_message = "nlj_laser/SetLaserDescriptor";
  ROS_DEBUG("setting descriptor services ....");
  if (!client.call(srv))
  {
    ROS_ERROR("Failed to create the Lama interface %s", srv.request.interface_name.c_str());
    return 1;
  }

  ROS_DEBUG("starting jockeys ....");
  // Run the jockeys.
  LJLaser loc_jockey("localizing_jockey", srv.request.interface_name, srv.response.set_service_name);
  NJLaser nav_jockey("navigating_jockey", srv.response.get_service_name);
  ROS_DEBUG("jockeys started");

  //  ros::Subscriber laserHandler = n.subscribe<sensor_msgs::LaserScan> ("base_scan", 50 , handleLaser);
//  pub = n.advertise<geometry_msgs::Twist> ("nav_cmd",1,false);
  


  ROS_DEBUG("laser_jockey started");
  ros::spin();
  return 0;
}

