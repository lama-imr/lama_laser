#ifndef _NJ_LASER_CROSSDETECT_H_
#define _NJ_LASER_CROSSDETECT_H_

#include <vector>

#include <nj_laser/laloc_utils.h>

/**
  * routines for cross detection from laser scan. It is assumed
  * that two laser scanners are used to capture data 
  */

namespace lama {
namespace Laloc {

/* SFrontier is a line segment through which the robot can go
 * p1 First point
 * p2 Second point, so that angle(r-p1, r-p2) is positive, where r is the laser base
 * width Segment length, i.e. width of free space
 * angle ?
 */
struct SFrontier {

  SFrontier(const Point2& ip1, const Point2& ip2, const double iwidth, const double iangle) :
    p1(ip1),
    p2(ip2),
    width(iwidth),
    angle(iangle)
  {
  }

  Point2 p1, p2;
  double width;
  double angle;
};

/** @brief cross detection with frontiers.
  */
void cdPanoramatic3(
    const std::vector<double> &scan, const double rt, const double dt,
    const double minPhi, const double fov, const double maxFrontierAngle,
    std::vector<SFrontier> &frontiers);

/** @brief return center of a cross by finding largest circle. works fine
  */
void getCrossCenterVoronoi(const std::vector<Point2> &pts, const double rt,
    const double dt, double &cx, double &cy, double &radius);

void getCrossCenterVoronoiWithKDTree(
    const vector<Point2> &pts,  const double rt, const double dt,
    double &cx, double &cy, double &radius);

} // namespace Laloc
} // namespace lama

#endif // _NJ_LASER_CROSSDETECT_H_

