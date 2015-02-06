#ifndef _NLJ_LASER_NJ_LASER_H_
#define _NLJ_LASER_NJ_LASER_H_

#include <cstdlib>
#include <ctime>

#include <lama_jockeys/navigating_jockey.h>

#include <nlj_laser/GetLaserDescriptor.h>
#include <nlj_laser/LaserDescriptor.h>

#include <sensor_msgs/LaserScan.h>


class NJLaser : public lama_jockeys::NavigatingJockey
{
  public:

    NJLaser(const std::string& name, const std::string& get_service_name);

    virtual void onTraverse();
    virtual void onStop();
    virtual void onInterrupt();
    virtual void onContinue();

  private:

    // Name of the getter service as interface to Lama.
    std::string get_service_name_;
                
    ros::Subscriber laser_scan_subscriber_; ///<keep the subscriber to allow subscribe/un-subscribe as start/stop/interrupt/continue is called 
    ros::Publisher cmd_vel_publisher_; ///< publisher to provide cmd_vel command
    
    const double mean_traversing_time_;
    const double max_traversing_delta_;
    nlj_laser::LaserDescriptor actual_trajectory_descriptor_; ///< actual trajectory descriptor holds all the laser scans in a array
    
    bool assigned_; ///< if assigned is true, robot have a goal to navigate towards 
    int navigation_index_;  ///<index of reference scan which robot use for localize itself and navigate towards it
    sensor_msgs::LaserScan reference_laser_scan_; ///< reference scan

    bool completed_;

    void handleLaser(sensor_msgs::LaserScan msg); ///< callback function handling the LaserScan messages 
    double completion();
};

#endif // _NLJ_LASER_NJ_LASER_H_
