#include <nlj_laser/lj_laser.h>
#include <nlj_laser/hist.h>

LJLaser::LJLaser(const std::string& name,
    const std::string& interface_name,
    const std::string& set_service_name) :
  lama_jockeys::LearningJockey(name),
  interface_name_(interface_name),
  set_service_name_(set_service_name)
{
}


void LJLaser::onLearn() 
{
  ROS_DEBUG(" LJLaser LEARNING");
  ros::Time start_time = ros::Time::now();
  assigned_ = false;
  laser_scan_subscriber_ = nh_.subscribe<sensor_msgs::LaserScan> ("base_scan",50,&LJLaser::handleLaser,this);
  // start navigating.
  ros::Rate r(10);
/*  ros::spin();
  ROS_DEBUG("SPIN ENDS");*/
  while (ros::ok())
  {
    ros::spinOnce();
    if (server_.isPreemptRequested() )
    {
      ROS_INFO_STREAM(jockey_name_ << ": Preempted from LEARNING");
      break;
    }
      if (!ros::ok())
    {
      ROS_INFO_STREAM(jockey_name_ << ": ros not ok");
      break;
    }
    // update the feedback every 0.5 s.
    if (!server_.isActive()) 
    {
      ROS_DEBUG("Goal succeeded");
      break;
    }
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
    
    /*feedback_.current_state = lama_jockeys::LearnFeedback::LEARNING;
    feedback_.time_elapsed = time_elapsed_for_task;
    feedback_.completion = completion();
    server_.publishFeedback(feedback_);*/
    r.sleep();
  }
  ROS_DEBUG("LEARNING ENDS");
}

void LJLaser::onStop()
{
  ROS_DEBUG("NJLaser STOP");
  laser_scan_subscriber_.shutdown();
  
  nlj_laser::SetLaserDescriptor ds;
  ds.request.descriptor.scans = descriptor_;
  ros::service::call(set_service_name_,ds);

  result_.final_state = lama_jockeys::LearnResult::DONE;
  result_.completion_time = ros::Duration(0);
  server_.setSucceeded(result_);
}

void LJLaser::handleLaser(sensor_msgs::LaserScan msg) {
  if (!assigned_) {
    reference_laser_scan_ = msg;
    descriptor_.push_back(reference_laser_scan_);
    assigned_ = true; 
  }
  int angle_hist_bin = 100;
  float max_distance = 0.3;
  float max_error = 0.2; 
  ROS_DEBUG("LEARNING laser arrived %lu",msg.ranges.size());
  geometry_msgs::Pose2D goal = localize(msg, reference_laser_scan_, angle_hist_bin);
  geometry_msgs::Pose2D fix;
  fix.x= fix.y=fix.theta = 0;
  float err = error (msg,reference_laser_scan_,goal);    
  float distance = sqrt (goal.x*goal.x+goal.y*goal.y) ;
  if (err > max_error) {
    ROS_INFO("error exceeds max limit");
    reference_laser_scan_ = previous_laser_scan_;
    descriptor_.push_back(reference_laser_scan_);

  } 
  if (distance > max_distance) {
    ROS_DEBUG("distance is longer than max");
    reference_laser_scan_ = msg;
    descriptor_.push_back(reference_laser_scan_);
  }

  previous_laser_scan_ = msg;

}

double LJLaser::completion()
{
  return 0;
}

