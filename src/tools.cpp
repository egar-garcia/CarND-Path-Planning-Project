#include "tools.h"
#include <math.h>

using namespace std;


Tools::Tools() {}

double Tools::normalizeAngle(const double &x) {
  double n_x = x;
  while (n_x < -pi()) {
    n_x += 2 * pi();
  }
  while (n_x > pi()) {
    n_x -= 2 * pi();
  }
  return n_x;
}

void Tools::translate(double &x, double &y, const double &trans_x, const double &trans_y) {
  x -= trans_x;
  y -= trans_y;
}

void Tools::rotate(double &x, double &y, const double &angle) {
  const double original_x = x;
  const double original_y = y;
  x = original_x * cos(angle) + original_y * sin(angle);
  y = -original_x * sin(angle) + original_y * cos(angle);
}

void Tools::translateAndRotate(double &x, double &y,
                               const double &trans_x, const double &trans_y, const double &angle) {
  translate(x, y, trans_x, trans_y);
  rotate(x, y, angle);
}

void Tools::rotateAndTranslate(double &x, double &y,
                               const double &angle, const double &trans_x, const double &trans_y) {
  rotate(x, y, angle);
  translate(x, y, trans_x, trans_y);
}

double Tools::mph2ms(const double &x) {
  return (x * 1609.34) / (60.0 * 60.0);
}

double Tools::distance(const double &x1, const double &y1, const double &x2, const double &y2) {
	return sqrt( (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int Tools::ClosestWaypoint(const double &x, const double &y,
                           const vector<double> &maps_x, const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int Tools::NextWaypoint(const double &x, const double &y, const double &theta,
                        const vector<double> &maps_x, const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Tools::getFrenet(const double &x, const double &y, const double &theta,
                                const vector<double> &maps_x, const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Tools::getXY(const double &s, const double &d,
                            const vector<double> &maps_s,
                            const vector<double> &maps_x, const vector<double> &maps_y) {
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}
