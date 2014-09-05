/* Action server for the memory-less navigating jockey based on LaserScan.
 *
 * The role of this jockey is to travel to the next crossing.
 * The action is done when the robot reaches the crossing center.
 *
 * Interaction with the map:
 * - Getter: none
 * - Setter: none.
 *
 * Subscribers (other than map-related):
 * - sensor_msg::LaserScan, "~/base_scan", 360-deg laser-scan.
 *
 * Publishers (other than map-related):
 * - visualization_msgs::Marker, "~/crossing_marker", a sphere at the crossing center.
 * - visualization_msgs::Marker, "~/exits_marker", lines from crossing center towards exits.
*/

#ifndef _NJ_LASER_NJ_LASER_H_
#define _NJ_LASER_NJ_LASER_H_

#include <sensor_msgs/LaserScan.h>

#include <lama_jockeys/navigating_jockey.h>

#include <nj_laser/claloc.h>
#include <nj_laser/visualization.h>

namespace lama {
namespace nj_laser {

class NJLaser : public lama::NavigatingJockey
{
  public:

    NJLaser(std::string name);

    virtual void onTraverse();
    virtual void onStop();
    virtual void onInterrupt();
    virtual void onContinue();

  private:

    void handleLaser(const sensor_msgs::LaserScan msg);

    ros::Subscriber laserHandler_;
    ros::Publisher pub_crossing_marker_;
    ros::Publisher pub_exits_marker_;
    ros::Publisher pub_twist_;
    lama::nj_laser::CLaloc cross_detector;
};

} // namespace nj_laser
} // namespace lama

#endif // _NJ_LASER_NJ_LASER_H_

