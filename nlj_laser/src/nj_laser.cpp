#include <nlj_laser/nj_laser.h>
#include <nlj_laser/hist.h>
NJLaser::NJLaser(const std::string& name, const std::string& get_service_name) :
  lama_jockeys::NavigatingJockey(name),
  get_service_name_(get_service_name),
  mean_traversing_time_(2.0),
  max_traversing_delta_(0.5)
{
}

void NJLaser::onStop()
{
  ROS_DEBUG("NJLaser STOP");
  laser_scan_subscriber_.shutdown();
  result_.final_state = lama_jockeys::NavigateResult::STOPPED;
  result_.completion_time = ros::Duration(0);
  server_.setSucceeded(result_);
}



void NJLaser::onTraverse()
{
  ROS_DEBUG("NJLaser TRAVERSING");
  ros::Time start_time = ros::Time::now();
  nlj_laser::GetLaserDescriptor dg;
  assigned_ = false;
  completed_ = false;
  dg.request.id = goal_.descriptor_link.descriptor_id;
  ROS_INFO("goal.descriptor: %d", goal_.descriptor_link.descriptor_id);
  ros::service::call(get_service_name_, dg);
  actual_trajectory_descriptor_ = dg.response.descriptor;
  laser_scan_subscriber_ = nh_.subscribe<sensor_msgs::LaserScan> ("base_scan",50,&NJLaser::handleLaser,this);
  cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist> ("nav_cmd",1,false);

  // start navigating.
  ros::Rate r(10);
  while (ros::ok())
  {
    if (server_.isPreemptRequested() && !ros::ok())
    {
      ROS_INFO_STREAM(jockey_name_ << ": Preempted");
      break;
    }
    ros::spinOnce();
    r.sleep();
    ros::Time current_time = ros::Time::now();
    ros::Duration real_time_elapsed = current_time - start_time;
    ros::Duration time_elapsed_for_task;
    if (isInterrupted())
    {
      // time_elapsed_for_task is frozen.
      time_elapsed_for_task = getInterruptionTime() - start_time - getInterruptionsDuration();
    }
    else
    {
      // time_elapsed_for_task runs.
      time_elapsed_for_task = real_time_elapsed - getInterruptionsDuration();
    }
    feedback_.current_state = lama_jockeys::NavigateFeedback::TRAVERSING;
    feedback_.time_elapsed = time_elapsed_for_task;
    feedback_.completion = completion();
    server_.publishFeedback(feedback_);

    // Eventually get the result.
    if (completed_)
    {
      result_.final_state = lama_jockeys::NavigateResult::DONE;
      result_.completion_time = real_time_elapsed;
      server_.setSucceeded(result_);
      break;
    }
  }
  laser_scan_subscriber_.shutdown();
  ROS_DEBUG("NAVIGATION completed");
}


void NJLaser::onInterrupt() 
{
  laser_scan_subscriber_.shutdown();
}

void NJLaser::onContinue() 
{
  ROS_DEBUG_STREAM(jockey_name_ << ": continue");
  laser_scan_subscriber_ = nh_.subscribe<sensor_msgs::LaserScan> ("base_scan",50,&NJLaser::handleLaser,this);
}

void NJLaser::handleLaser(sensor_msgs::LaserScan msg) {
  if (!assigned_) 
  {
    completed_ = false;
    navigation_index_ = 0; 
    reference_laser_scan_ = actual_trajectory_descriptor_.scans[navigation_index_++];
    assigned_ = true; 
  }
  int angle_histogram_bin_number = 100;
  float min_distance = 0.05;

  geometry_msgs::Pose2D goal = localize(msg, reference_laser_scan_, angle_histogram_bin_number);
  geometry_msgs::Pose2D fix;
  fix.x= fix.y=fix.theta = 0;
  //  float err = error (msg,reference_laser_scan_,goal,NULL,NULL);    
  float distance = sqrt (goal.x*goal.x+goal.y*goal.y) ;
  //  ROS_DEBUG("error %f goal %f %f ", err, goal.x, goal.y);
  if (distance < min_distance) 
  {
    ROS_DEBUG_STREAM(jockey_name_<< "distance shorter than min nav index" << navigation_index_ <<" from "<< actual_trajectory_descriptor_.scans.size());
    if (navigation_index_ < actual_trajectory_descriptor_.scans.size()) 
    {
      reference_laser_scan_ = actual_trajectory_descriptor_.scans[navigation_index_++];
    } 
    else 
    {
      completed_  = true;
      ROS_DEBUG("completed");
    }
  }
  if (!completed_) 
  {
    //controller
    float deviation = atan2(goal.y, goal.x)*0.5;
    if (deviation > 0.8) 
      deviation = 0.8;
    if (deviation < -0.8) 
      deviation = -0.8;
    float speed = 2*distance *(0.8-fabs(deviation)) ; 
    if (speed < -0.05) 
      speed = -0.05;


    geometry_msgs::Twist pub_msg;
    pub_msg.linear.x = 0.9*speed;
    pub_msg. angular.z = 1.1*deviation;
    ROS_DEBUG("speed x %f, y %f",pub_msg.linear.x, pub_msg.angular.z);
    cmd_vel_publisher_.publish(pub_msg);
  } 
  else 
  {
    geometry_msgs::Twist pub_msg;
    pub_msg.linear.x = 0;
    pub_msg. angular.z = 0 ;
    cmd_vel_publisher_.publish(pub_msg);
  }
}

double NJLaser::completion()
{
  return (1.0*navigation_index_-1)/actual_trajectory_descriptor_.scans.size();
}

