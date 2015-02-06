#include <math.h>
#include <vector>
#include <stdio.h>
#include "nlj_laser/hist.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"
#define SHIFT 1 

Hist hist(sensor_msgs::LaserScan scan, int anglebins) {
  int angleHistBin = anglebins;
  double angleHistCoef = angleHistBin / (2 * M_PI);
  Hist hist(angleHistBin,0);
  double px1;
  double px2;
  double py1;
  double py2;
  double distance;
  px1 = scan.ranges[0] * sin(0 * scan.angle_increment + scan.angle_min);
  py1 = -scan.ranges[0] * cos(0 * scan.angle_increment + scan.angle_min);   
  for(unsigned int i = 1; i < scan.ranges.size() - SHIFT ; i++) {
    px2 = scan.ranges[i+SHIFT] * sin((i+SHIFT) * scan.angle_increment + scan.angle_min);
    py2 = -scan.ranges[i+SHIFT] * cos((i+SHIFT) * scan.angle_increment + scan.angle_min);   
    double ath = atan2(py1 - py2, px1 - px2);
    distance = sqrt((py1 - py2) * (py1 - py2) + (px1 - px2) * (px1 - px2));
    if (distance  < 0.1) {
      //if (distance  < 0.051) {
      //hist[(int) floor((M_PI + ath) * angleHistCoef)]+= scan[i];
      hist[(int) floor((M_PI + ath) * angleHistCoef)]+= 1;
    }
    px1 = px2;
    py1 = py2;
    px1 = scan.ranges[i] * sin((i) * scan.angle_increment + scan.angle_min);
    py1 = -scan.ranges[i] * cos((i) * scan.angle_increment + scan.angle_min);   
    }
    return hist;
  }

  int crossCorrelationHist (Hist x , Hist y) {
    double sum;
    double max;
    int maxind;
    max = 0;
    maxind = 0;
    for (unsigned int shift=0; shift<x.size();shift++) {
      //      std::cout <<shift << " " << x[shift] << " " << y[shift] << std::endl;
      sum = 0; 
      for (unsigned int i=0; i<x.size();i++) {
        sum += x[i]*y[(i+shift)%y.size()];
      }
      if (sum > max) {
        max = sum;
        maxind = shift;
      }
    }
    //   std::cout <<"correlation max "<< max << "@" << maxind<< "\n";
    return maxind;
  }


  geometry_msgs::Pose2D localize (sensor_msgs::LaserScan a, sensor_msgs::LaserScan b, int angls) {
    double transCoef = TRANSCOEF;
    //int angleHistBin = ANGLEBIN;
    int angleHistBin = angls;
    double resolution = 2 * M_PI / a.ranges.size();
    double angleHistCoef = angleHistBin / (2 * M_PI);

    Hist histax (HISTSIZE ,-1);
    Hist histay (HISTSIZE ,-1);
    Hist histbx (HISTSIZE ,-1);
    Hist histby (HISTSIZE ,-1);
    Hist histA = hist(a,angleHistBin);
    Hist histB = hist(b,angleHistBin);
    int angleIndex = crossCorrelationHist(histA,histB);

    int histAngleMaxIndex=0; 
    int histMax = 0;
    for (unsigned int i = 0; i < histA.size(); i++) {
      if (histMax < histA[i]) {
        histMax = histA[i];
        histAngleMaxIndex = i;
      }
    }
    /*   std::cout << histAngleMaxIndex << std::endl;
         for (int i = 0 ; i < histA.size(); i++) {
         geometry_msgs::Pose2D point;
         point.py = histA[i]/10.0* cos(-(i+angleIndex)/angleHistCoef) ;
         point.px = histA[i]/10.0 * sin(-(i+angleIndex)/angleHistCoef);
         draw[i] = point;
         }
         staticgp->Color(200,0,0,0);
         staticgp->DrawPolyline(draw, histA.size());
         for (int i = 0 ; i < histB.size(); i++) {
         geometry_msgs::Pose2D point;
         point.py = histB[i]/10.0* cos(-i/angleHistCoef) ;
         point.px = histB[i]/10.0 * sin(-i/angleHistCoef);
         draw[i] = point;
         }
         staticgp->Color(0,200,0,0);
         staticgp->DrawPolyline(draw, histB.size());
         delete [] draw;
         */
    /*    for (int i = 0 ; i < a.size(); i++) {
          adist[i] =   sqrt((a[i]).py * (a[i]).py + (a[i]).px  * (a[i]).px );
          }
          for (int i = 0 ; i < b.size(); i++) {
          bdist[i] =   sqrt((b[i].py * b[i].py) + (b[i].px ) * (b[i].px ));
          }
          */
    // compute projections into two perpendicular planes in direction of biggest peak in angle hist
    for (unsigned int i = 0; i < a.ranges.size(); i++) {
      if (a.ranges[i] < 7.9) {
        histax[(int) floor(transCoef * a.ranges[i] * cos(i * resolution - histAngleMaxIndex / angleHistCoef)) + HISTSIZEHALF]++;
        histay[(int) floor(transCoef * a.ranges[i] * sin(i * resolution - histAngleMaxIndex / angleHistCoef)) + HISTSIZEHALF]++;
      }
    }
    // compute projections into two perpendicular planes also for scan b 
    for (unsigned int i = 0; i < b.ranges.size(); i++) {
      if (b.ranges[i] < 7.9) {
        histbx[(int) floor(transCoef * b.ranges[i] * cos(i * resolution - histAngleMaxIndex / angleHistCoef - (angleIndex) / angleHistCoef)) + HISTSIZEHALF]++;
        histby[(int) floor(transCoef * b.ranges[i] * sin(i * resolution - histAngleMaxIndex / angleHistCoef - (angleIndex) / angleHistCoef)) + HISTSIZEHALF]++;
      }
    }
    int xIndex = crossCorrelationHist(histax, histbx);
    int yIndex = crossCorrelationHist(histay, histby);
    if (xIndex > HISTSIZEHALF) xIndex -= HISTSIZE;
    if (yIndex > HISTSIZEHALF) yIndex -= HISTSIZE;
    //   std::cout << xIndex << " " << yIndex <<"\n";
    geometry_msgs::Pose2D ret ;
    double ty = cos(-histAngleMaxIndex / angleHistCoef) * yIndex / transCoef - sin(-histAngleMaxIndex / angleHistCoef) * xIndex / transCoef;
    double tx = (xIndex / transCoef + sin(-histAngleMaxIndex / angleHistCoef) * ty) / cos(-histAngleMaxIndex / angleHistCoef);
    ret.x = tx;
    ret.y = ty;
    ret.theta= 2*M_PI-angleIndex/angleHistCoef;
    /*  ret.px = ty;
        ret.py = -tx;*/

    return ret;
  }

  double error(sensor_msgs::LaserScan scan,sensor_msgs::LaserScan reference,  geometry_msgs::Pose2D goal) {
    double rx, ry,sx,sy;
    double error = 0;
    double distance ;
    double min = 9000000;
    int count = 0;
    geometry_msgs::PolygonStamped poly;
    poly.header.frame_id="/mf";
    geometry_msgs::PolygonStamped poly2;
    poly2.header.frame_id="/mf";
    geometry_msgs::Point32 p;
    for (unsigned int i = 0 ; i < reference.ranges.size(); i++) {
      min = 90000000;
      if (reference.ranges[i] < 7.9) {
        rx = reference.ranges[i] * sin( (i * reference.angle_increment) + reference.angle_min + goal.theta )  + goal.y;
        ry =  -reference.ranges[i] * cos( (i * reference.angle_increment) + reference.angle_min + goal.theta) - goal.x;

        for (unsigned int  j = 0 ; j < scan.ranges.size(); j++) {
          if (scan.ranges[j] < 7.9){
            sx = scan.ranges[j] * sin( (j * scan.angle_increment) + scan.angle_min);
            sy =  -scan.ranges[j] * cos( (j * scan.angle_increment) + scan.angle_min); 
            distance = sqrt((rx-sx)*(rx-sx) + (ry-sy)*(ry-sy));
            if (distance < min) {
              min = distance;
            }
          }
        }
        error += min;
        count++;
      }
    }
    return error/count;
  }


  double error(sensor_msgs::LaserScan scan,sensor_msgs::LaserScan reference,  geometry_msgs::Pose2D goal, ros::Publisher draw, ros::Publisher drawRef ) {
    double rx, ry,sx,sy;
    double error = 0;
    double distance ;
    double min = 9000000;
    int count = 0;
    geometry_msgs::PolygonStamped poly;
    poly.header.frame_id="/mf";
    geometry_msgs::PolygonStamped poly2;
    poly2.header.frame_id="/mf";
    geometry_msgs::Point32 p;
    if (draw!= 0 )
      for (unsigned int i = 0 ; i < reference.ranges.size(); i++) {
        min = 90000000;
        if (reference.ranges[i] < 7.9) {
          rx = reference.ranges[i] * sin( (i * reference.angle_increment) + reference.angle_min + goal.theta )  + goal.y;
          ry =  -reference.ranges[i] * cos( (i * reference.angle_increment) + reference.angle_min + goal.theta) - goal.x;
          p.x = rx;
          p.y =ry;
          poly.polygon.points.push_back(p);
        }  
        if (scan.ranges[i] < 7.9){
          sx = scan.ranges[i] * sin( (i * scan.angle_increment) + scan.angle_min );
          sy = -  scan.ranges[i] * cos( (i * scan.angle_increment) + scan.angle_min ); 
          p.x = sx;
          p.y =sy;
          poly2.polygon.points.push_back(p);
        }
      }
    for (unsigned int i = 0 ; i < reference.ranges.size(); i++) {
      min = 90000000;
      if (reference.ranges[i] < 7.9) {
        rx = reference.ranges[i] * sin( (i * reference.angle_increment) + reference.angle_min + goal.theta )  + goal.y;
        ry =  -reference.ranges[i] * cos( (i * reference.angle_increment) + reference.angle_min + goal.theta) - goal.x;

        for (unsigned int  j = 0 ; j < scan.ranges.size(); j++) {
          if (scan.ranges[j] < 7.9){
            sx = scan.ranges[j] * sin( (j * scan.angle_increment) + scan.angle_min);
            sy =  -scan.ranges[j] * cos( (j * scan.angle_increment) + scan.angle_min); 
            distance = sqrt((rx-sx)*(rx-sx) + (ry-sy)*(ry-sy));
            if (distance < min) {
              min = distance;
            }
          }
        }
        error += min;
        count++;
      }
    }
    draw.publish(poly2);
    drawRef.publish(poly);
    return error/count;
  }

