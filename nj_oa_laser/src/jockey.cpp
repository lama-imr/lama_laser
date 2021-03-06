#include <nj_oa_laser/jockey.h>

namespace nj_oa_laser
{

Jockey::Jockey(const std::string& name, const double robot_radius) :
  lama_jockeys::NavigatingJockey(name),
  twist_handler_(robot_radius)
{
  initTwistHandlerParam(twist_handler_);
}

void Jockey::initTwistHandlerParam(TwistHandler& twist_handler)
{
  private_nh_.getParam("robot_radius", twist_handler.robot_radius);
  private_nh_.getParam("min_distance", twist_handler.min_distance);
  private_nh_.getParam("long_distance", twist_handler.long_distance);
  private_nh_.getParam("turnrate_collide", twist_handler.turnrate_collide);
  private_nh_.getParam("vel_close_obstacle", twist_handler.vel_close_obstacle);
  private_nh_.getParam("turnrate_factor", twist_handler.turnrate_factor);
  private_nh_.getParam("max_linear_velocity", twist_handler.max_linear_velocity);
  private_nh_.getParam("max_angular_velocity", twist_handler.max_angular_velocity);
  private_nh_.getParam("short_lateral_distance", twist_handler.short_lateral_distance);
  private_nh_.getParam("long_lateral_distance", twist_handler.long_lateral_distance);
  private_nh_.getParam("force_turn_left_factor", twist_handler.force_turn_left_factor);
}

void Jockey::onTraverse()
{
  ROS_DEBUG("Received action TRAVERSE or CONTINUE");

  ros::Subscriber laser_handler = private_nh_.subscribe<sensor_msgs::LaserScan>("base_scan", 1, &Jockey::handleLaser, this);
  pub_twist_ = private_nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
  // Loop until the goal is preempted.
  ros::Rate r(100);
  while (ros::ok())
  {
    ros::spinOnce();
    if (server_.isPreemptRequested() && !ros::ok())
    {
      ROS_INFO("%s: Preempted", jockey_name_.c_str());
      // set the action state to preempted
      server_.setPreempted();
      break;
    }

    r.sleep();
  }

  pub_twist_.shutdown();
}

void Jockey::onStop()
{
  ROS_DEBUG("Received action STOP or INTERRUPT");
  result_.final_state = lama_jockeys::NavigateResult::DONE;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void Jockey::onInterrupt()
{
  ROS_DEBUG("Received action INTERRUPT");
  onStop();
}

void Jockey::onContinue()
{
  ROS_DEBUG("Received action CONTINUE");
  onTraverse();
}

/* Callback for the LaserScan topic
 */
void Jockey::handleLaser(const sensor_msgs::LaserScanConstPtr& msg)
{
  geometry_msgs::Twist twist = twist_handler_.getTwist(*msg);
  pub_twist_.publish(twist);
}

} // namespace nj_oa_laser
