#include <nj_oa_laser/twist_handler.h>

#include <ros/ros.h>

namespace nj_oa_laser
{

TwistHandler::TwistHandler(const double robot_radius) :
  robot_radius(robot_radius),
  min_distance(1.5 * robot_radius),
  long_distance(5 * robot_radius),
  short_lateral_distance(2 * robot_radius),
  long_lateral_distance(1.5 * short_lateral_distance),
  force_turn_left_factor(3),
  turnrate_collide(0.4),
  vel_close_obstacle(0.5),
  turnrate_factor(0.9),
  max_linear_velocity(1),
  max_angular_velocity(1)
{
}

/** Return the twist to avoid obstacles
 *
 * The algorithm considers only beams in [-pi, pi]. The robot will turn in the direction
 * where the mean obstacle distance is larger.
 * The algorithm has 4 behaviors:
 * - if the robot is in a corner (mean distance in front < 2 collide distance), turn left.
 * - if obstacles are close (collide distance), only turn towards free space.
 * - if obstacles are close but farther than the collide distance, go forward
 *     and turn proportionnally to the mean side distance to obstacles.
 * - if obstacles are far, go strictly forward.
 */
geometry_msgs::Twist TwistHandler::getTwist(const sensor_msgs::LaserScan& scan)
{
   bool collide = false;
   bool go_straight = true;
   double sum_distance_front = 0;
   unsigned int count_distance_front = 0;
   double sum_y = 0;
   unsigned int count_y = 0;
   double integral_collide = 0;

   double x;
   double y;
   // Start and end values for the integral in collide rectangle.
   double previous_x_yplus = min_distance;
   double previous_x_yminus = 0;

   for (unsigned int i = 0; i < scan.ranges.size(); ++i)
   {
     const double angle = angles::normalize_angle(scan.angle_min + i * scan.angle_increment);
     if ((angle < -M_PI_2) || (angle > M_PI_2))
     {
       // Do no consider a beam directed backwards.
       continue;
     }

     if ((-M_PI_4 < angle) && (angle < M_PI_4))
     {
       sum_distance_front += scan.ranges[i];
       count_distance_front++;
     }

     x = scan.ranges[i] * std::cos(angle);
     y = scan.ranges[i] * std::sin(angle);

     if (x < min_distance)
     {
       if ((-short_lateral_distance < y) && (y < short_lateral_distance))
       {
         if (y > 0)
         {
           integral_collide += y * (previous_x_yplus - x); 
           previous_x_yplus = x;
         }
         else
         {
           integral_collide += y * (x - previous_x_yminus); 
           previous_x_yminus = x;
         }
         collide = true;
       }
       else
       {
         // Front distance is short, lateral distance is large.
         // Add the cut distance to the integral.
         if (y > 0)
         {
           integral_collide += short_lateral_distance * (previous_x_yplus - x);
           previous_x_yplus = x;
         }
         else
         {
           integral_collide -= short_lateral_distance * (x - previous_x_yminus);
           previous_x_yminus = x;
         }
       }
     } 

     if ((x < long_distance) && (-long_lateral_distance < y) && (y < long_lateral_distance))
     {
       go_straight = false;
     }
     sum_y += y;
     count_y++;
   }

   // Complete the integral.
   integral_collide += short_lateral_distance * previous_x_yplus;
   integral_collide -= short_lateral_distance * (min_distance - previous_x_yminus);

   // Corner detection.
   const bool force_turn_left = (count_distance_front > 0) &&
     (sum_distance_front / count_distance_front < force_turn_left_factor * min_distance);

   geometry_msgs::Twist twist;
   if (force_turn_left)
   {
     twist.angular.z = turnrate_collide;
     ROS_DEBUG("Mean dist to obstacle within %.3f, force left turn",
         force_turn_left_factor * min_distance);
   }
   else if (collide)
   { 
     twist.linear.x = 0;
     if (integral_collide > 0)
     { 
       // Obstacle distance is larger, i.e. space is more free, go there.
       twist.angular.z = turnrate_collide;
       ROS_DEBUG("Obstacle on the right, turn left");
     }
     else
     {
       twist.angular.z = -turnrate_collide;
       ROS_DEBUG("Obstacle on the left, turn right");
     }      
   }
   else if (go_straight)
   {
     twist.linear.x = max_linear_velocity;
     twist.angular.z = 0;
     ROS_DEBUG("No obstacle");
   }
   else
   {
     twist.linear.x = vel_close_obstacle;
     twist.angular.z = turnrate_factor * sum_y / ((double) count_y);
     ROS_DEBUG("Obstacle seen within %.3f, %.3f",  long_distance, long_lateral_distance);
   }

   if (twist.linear.x < -max_linear_velocity)
   {
     twist.linear.x = -max_linear_velocity;
   }
   else if (twist.linear.x > max_linear_velocity)
   {
     twist.linear.x = max_linear_velocity;
   }

   if (twist.angular.z < -max_angular_velocity)
   {
     twist.angular.z = -max_angular_velocity;
   }
   else if (twist.angular.z > max_angular_velocity)
   {
     twist.angular.z = max_angular_velocity;
   }

   return twist;
}

} // namespace nj_laser

