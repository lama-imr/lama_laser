#ifndef _NLJ_LASER_LJ_LASER_H_
#define _NLJ_LASER_LJ_LASER_H_

#include <cstdlib>
#include <ctime>
#include <math.h>  // for lround()

#include <sensor_msgs/LaserScan.h>

#include <lama_msgs/DescriptorLink.h>
#include <lama_jockeys/learning_jockey.h>

#include <nlj_laser/SetLaserDescriptor.h>

class LJLaser : public lama_jockeys::LearningJockey
{
  public:

    LJLaser(const std::string& name,
        const std::string& interface_name,
        const std::string& set_service_name);

    void onLearn();
    void onStop();

  private:

    // Name of the interface and setter service as interface to Lama.
    std::string interface_name_;
    std::string set_service_name_;
    
    ros::Subscriber laser_scan_subscriber_; ///<keep the subscriber to allow subscribe/un-subscribe as start/stop/interrupt/continue is called 

    bool assigned_; ///< true as first laser scan is added to the descriptor

    std::vector<sensor_msgs::LaserScan> descriptor_; ///< descriptor to save on stop
    sensor_msgs::LaserScan reference_laser_scan_; ///< laser scan added to the descriptor at last
    sensor_msgs::LaserScan previous_laser_scan_; ///< previously received sensor message kept

    void handleLaser(sensor_msgs::LaserScan msg); ///< callback function processing laser scans
    double completion();
};

#endif // _NLJ_LASER_LJ_LASER_H_
